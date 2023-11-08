// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationHelpers_CUDA.h"

using namespace ITMLib;

//device implementations
__global__ void ITMLib::countVisibleBlocks_device(const int *visibleEntryIDs, int noVisibleEntries,
                                                  const ITMHashEntry *hashTable, uint *noBlocks, int minBlockId,
                                                  int maxBlockId) {
  int globalIdx = threadIdx.x + blockIdx.x * blockDim.x;
  if (globalIdx >= noVisibleEntries)
    return;

  int entryId = visibleEntryIDs[globalIdx];
  int blockId = hashTable[entryId].ptr;
  if ((blockId >= minBlockId) && (blockId <= maxBlockId)) // 只统计在指定范围内的可见entry数量
    atomicAdd(noBlocks, 1); // TODO: 应该用归约的方式求和
}

__global__ void ITMLib::buildCompleteVisibleList_device(
    const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/
    int noTotalEntries, int *visibleEntryIDs, int *noVisibleEntries, uchar *entriesVisibleType, Matrix4f M,
    Vector4f projParams, Vector2i imgSize, float voxelSize) {
  int targetIdx = threadIdx.x + blockIdx.x * blockDim.x;
  if (targetIdx > noTotalEntries - 1)
    return;

  __shared__ bool shouldPrefix;
  // 只支持0、1两种可见类型。没有那么多花里胡哨的
  unsigned char hashVisibleType = 0; // entriesVisibleType[targetIdx];
  const ITMHashEntry &hashEntry = hashTable[targetIdx];

  shouldPrefix = false;
  __syncthreads();

  if (hashEntry.ptr >= 0) { // 对存在的voxel block检查可见性（不将图片扩大尺寸）
    shouldPrefix = true;

    bool isVisible, isVisibleEnlarged;
    checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);

    hashVisibleType = isVisible;
  }
  // 可见的就记录一下
  if (hashVisibleType > 0)
    shouldPrefix = true;  // 只要cuda block中有一个线程要记录，整个cuda block就参与后面的前缀和计算
  __syncthreads();

  if (shouldPrefix) { // 通过前缀求和找到全局数组中的位置
    int offset =
        computePrefixSum_device<int>(hashVisibleType > 0, noVisibleEntries, blockDim.x * blockDim.y, threadIdx.x);
    if (offset != -1)
      visibleEntryIDs[offset] = targetIdx;
  }
}

__global__ void ITMLib::projectAndSplitBlocks_device(const ITMHashEntry *hashEntries, const int *visibleEntryIDs,
                                                     int noVisibleEntries, const Matrix4f pose_M,
                                                     const Vector4f intrinsics, const Vector2i imgSize, float voxelSize,
                                                     RenderingBlock *renderingBlocks, uint *noTotalBlocks) {
  int in_offset = threadIdx.x + blockDim.x * blockIdx.x;

  const ITMHashEntry &blockData(hashEntries[visibleEntryIDs[in_offset]]);
  //! 将单个可见的block投影到 当前视角下，并计算包围盒 && 深度范围
  Vector2i upperLeft, lowerRight;   // 包围盒的左上、右下坐标
  Vector2f zRange;
  bool validProjection = false;
  if (in_offset < noVisibleEntries) // TODO:in_offset>noVisibleEntries应该return
    if (blockData.ptr >= 0)         // >=0表示当前block有效 
      validProjection = ProjectSingleBlock(blockData.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, 
                                           lowerRight, zRange);
  //! 将包围盒分小块，每块大小(renderingBlockSizeX,renderingBlockSizeY)=(16,16)。ceilf是向上取整。为啥要分块渲染？？？
  Vector2i requiredRenderingBlocks(ceilf((float)(lowerRight.x - upperLeft.x + 1) / renderingBlockSizeX),
  ceilf((float) (lowerRight.y - upperLeft.y + 1) / renderingBlockSizeY));

  size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y; // 包围盒中小块数量
      // TODO: 按照renderingBlockSizeX和renderingBlockSizeY都为16，不可能有requiredNumBlocks>1
  if (!validProjection) requiredNumBlocks = 0;    // TODO：直接return 就好？还是为了一定要有下面的computePrefixSum_device？
  // 通过前缀和 来找到每个小块 在最终数组里的位置
  int out_offset = computePrefixSum_device<uint>(requiredNumBlocks, noTotalBlocks, blockDim.x, threadIdx.x);
  if (!validProjection) return;
  if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;

  CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);  // 创建小块
}

__global__ void ITMLib::checkProjectAndSplitBlocks_device(const ITMHashEntry *hashEntries, int noHashEntries,
                                                          const Matrix4f pose_M, const Vector4f intrinsics,
                                                          const Vector2i imgSize, float voxelSize,
                                                          RenderingBlock *renderingBlocks, uint *noTotalBlocks) {
  int targetIdx = threadIdx.x + blockDim.x * blockIdx.x;
  if (targetIdx >= noHashEntries)
    return;

  const ITMHashEntry &hashEntry = hashEntries[targetIdx];

  Vector2i upperLeft, lowerRight;
  Vector2f zRange;
  bool validProjection = false;
  if (hashEntry.ptr >= 0)
    validProjection =
        ProjectSingleBlock(hashEntry.pos, pose_M, intrinsics, imgSize, voxelSize, upperLeft, lowerRight, zRange);

  Vector2i requiredRenderingBlocks(ceilf((float)(lowerRight.x - upperLeft.x + 1) / renderingBlockSizeX),
                                   ceilf((float)(lowerRight.y - upperLeft.y + 1) / renderingBlockSizeY));
  size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
  if (!validProjection)
    requiredNumBlocks = 0;

  int out_offset = computePrefixSum_device<uint>(requiredNumBlocks, noTotalBlocks, blockDim.x, threadIdx.x);
  if (requiredNumBlocks == 0)
    return;
  if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS))
    return;

  CreateRenderingBlocks(renderingBlocks, out_offset, upperLeft, lowerRight, zRange);
}

__global__ void ITMLib::fillBlocks_device(uint noTotalBlocks, const RenderingBlock *renderingBlocks,
                                          Vector2i imgSize, Vector2f *minmaxData) {
  int x = threadIdx.x;
  int y = threadIdx.y;
  int block = blockIdx.x * 4 + blockIdx.y;  // TODO: 把4换成blockDim.y
  if (block >= noTotalBlocks) return;

  const RenderingBlock &b(renderingBlocks[block]);
  int xpos = b.upperLeft.x + x;       // NOTE：这里是 深度范围图 的坐标
  if (xpos > b.lowerRight.x) return;
  int ypos = b.upperLeft.y + y;
  if (ypos > b.lowerRight.y) return;

  Vector2f &pixel(minmaxData[xpos + ypos * imgSize.x]);
  atomicMin(&pixel.x, b.zRange.x);  // 取最大的深度范围
  atomicMax(&pixel.y, b.zRange.y);
}

__global__ void ITMLib::findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints,
                                                 const Vector2f *minmaximg, Vector4f *forwardProjection,
                                                 float *currentDepth, Vector2i imgSize) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  int locId = x + y * imgSize.x;
  Vector4f fwdPoint = forwardProjection[locId];
  float depth = currentDepth[locId];
  // 获取当前像素对应ray 在之前获取的深度范围。注意：深度范围图 比 渲染图片 缩小了 minmaximg_subsample倍
  int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;
  Vector2f minmaxval = minmaximg[locId2];

  bool hasPoint = false;

  __shared__ bool shouldPrefix; // 来判断当前cuda block是否要参与前缀和计算
  shouldPrefix = false;
  __syncthreads();
  // 需要重新raycast的像素 = 找不到voxel && (旧的raycast中没有能投影到 || 深度值有效) && 存在表面   // TODO: float不应该用==0
  if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth > 0)) &&
      (minmaxval.x < minmaxval.y)) {  // NOTE：.w是voxel的置信度(<=0说明没有voxel），minmaxval.x < .y表示ray找到了表面
  // if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
    shouldPrefix = true;        // 只要有一个cuda thread要参与，则整个cuda block都要参与前缀和计算
    hasPoint = true;
  }

  __syncthreads();
  // 通过前缀和计算 在数组中的全局坐标
  if (shouldPrefix) {
    int offset = computePrefixSum_device(hasPoint, noMissingPoints, blockDim.x * blockDim.y,
                                         threadIdx.x + threadIdx.y * blockDim.x);
    if (offset != -1)
      fwdProjMissingPoints[offset] = locId;
  }
}

__global__ void ITMLib::forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize,
                                              Matrix4f M, Vector4f projParams, float voxelSize) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  int locId = x + y * imgSize.x;
  Vector4f pixel = pointsRay[locId];  // 第四个维度是从voxel中顺带读出来的置信度

  int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize); // 将三维点投影，获取一维像素坐标
  if (locId_new >= 0)
    forwardProjection[locId_new] = pixel;
}
