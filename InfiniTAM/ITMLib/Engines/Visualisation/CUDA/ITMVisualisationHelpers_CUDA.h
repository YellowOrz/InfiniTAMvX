// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

#include "../Shared/ITMVisualisationEngine_Shared.h"
#include "../../Reconstruction/Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../../Utils/ITMCUDAUtils.h"

namespace ITMLib {
// declaration of device functions
/**
 * 遍历场景中所有entry，构建可见列表
 * @param[in] hashTable           hash table
 * @param[in] noTotalEntries      场景中entry总数
 * @param[out] visibleEntryIDs    到的可见entries列表
 * @param[out] noVisibleEntries   找到的可见entry的数量
 * @param entriesVisibleType      没用到
 * @param[in] M                   当前视角相机位姿
 * @param[in] projParams          相机内参
 * @param[in] imgSize             深度范围图的尺寸。比渲染图片小 (见minmaximg_subsample)
 * @param[in] voxelSize           voxel size。单位米
 */
__global__ void
buildCompleteVisibleList_device(const ITMHashEntry *hashTable, /*ITMHashCacheState *cacheStates, bool useSwapping,*/
                                int noTotalEntries, int *visibleEntryIDs, int *noVisibleEntries,
                                uchar *entriesVisibleType, Matrix4f M, Vector4f projParams, Vector2i imgSize,
                                float voxelSize);

__global__ void countVisibleBlocks_device(const int *visibleEntryIDs,
                                          int noVisibleEntries,
                                          const ITMHashEntry *hashTable,
                                          uint *noBlocks,
                                          int minBlockId,
                                          int maxBlockId);

__global__ void projectAndSplitBlocks_device(const ITMHashEntry *hashEntries,
                                             const int *visibleEntryIDs,
                                             int noVisibleEntries,
                                             const Matrix4f pose_M,
                                             const Vector4f intrinsics,
                                             const Vector2i imgSize,
                                             float voxelSize,
                                             RenderingBlock *renderingBlocks,
                                             uint *noTotalBlocks);

__global__ void checkProjectAndSplitBlocks_device(const ITMHashEntry *hashEntries,
                                                  int noHashEntries,
                                                  const Matrix4f pose_M,
                                                  const Vector4f intrinsics,
                                                  const Vector2i imgSize,
                                                  float voxelSize,
                                                  RenderingBlock *renderingBlocks,
                                                  uint *noTotalBlocks);
/**
 * 遍历小块，确定最后raycasting像素的最大和最小深度值
 * @param[in] noTotalBlocks     需要渲染的小块数量
 * @param[in] renderingBlocks   渲染小块。是单次渲染的最小单元
 * @param[in] imgSize           深度范围图的大小
 * @param[out] minmaxData       深度范围图
 */
__global__ void fillBlocks_device(uint noTotalBlocks, const RenderingBlock *renderingBlocks,
                                  Vector2i imgSize, Vector2f *minmaxData);

__global__ void findMissingPoints_device(int *fwdProjMissingPoints, uint *noMissingPoints, const Vector2f *minmaximg,
                                         Vector4f *forwardProjection, float *currentDepth, Vector2i imgSize);

__global__ void forwardProject_device(Vector4f *forwardProjection, const Vector4f *pointsRay, Vector2i imgSize,
                                      Matrix4f M, Vector4f projParams, float voxelSize);
/**
 * @brief 计算raycast的渲染结果
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam modifyVisibleEntries 是否要顺带修改visile entry列表。为啥弄成模板参数、不弄成函数参数？？？
 * @param[out] out_ptsRay         渲染结果。即ray跟三维场景表面的交点。最后一位是权重
 * @param[out] entriesVisibleType visible entry列表。如果modifyVisibleEntries为true，则顺带更新
 * @param[in] voxelData           voxel block array
 * @param[in] voxelIndex          hash table
 * @param[in] imgSize             渲染的图像尺寸
 * @param[in] invM                相机位姿的逆。即 local to world
 * @param[in] invProjParams       相机内参的逆。fx、fy取倒数，cx、cy取负
 * @param[in] oneOverVoxelSize    voxel size的倒数
 * @param[in] minmaximg           深度范围图
 * @param[in] mu                  TSDF的截断值对应的距离
 */
template <class TVoxel, class TIndex, bool modifyVisibleEntries>
__global__ void genericRaycast_device(Vector4f *out_ptsRay, uchar *entriesVisibleType, const TVoxel *voxelData,
                                      const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM,
                                      Vector4f invProjParams, float oneOverVoxelSize, const Vector2f *minmaximg,
                                      float mu) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  int locId = x + y * imgSize.x;                                        // 渲染图上的一维坐标
  int locId2 = (int)floor((float)x / minmaximg_subsample) + 
               (int)floor((float)y / minmaximg_subsample) * imgSize.x;  // 深度范围图上的一维坐标

  castRay<TVoxel, TIndex, modifyVisibleEntries>(out_ptsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex,
                                                invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
}
// TODO: 下次从这儿开始
template <class TVoxel, class TIndex, bool modifyVisibleEntries>
__global__ void
genericRaycastMissingPoints_device(Vector4f *forwardProjection, uchar *entriesVisibleType, const TVoxel *voxelData,
                                   const typename TIndex::IndexData *voxelIndex, Vector2i imgSize, Matrix4f invM,
                                   Vector4f invProjParams, float oneOverVoxelSize, int *fwdProjMissingPoints,
                                   int noMissingPoints, const Vector2f *minmaximg, float mu) {
  int pointId = threadIdx.x + blockIdx.x * blockDim.x;

  if (pointId >= noMissingPoints)
    return;

  int locId = fwdProjMissingPoints[pointId];
  int y = locId / imgSize.x, x = locId - y * imgSize.x;
  int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

  castRay<TVoxel, TIndex, modifyVisibleEntries>(forwardProjection[locId], entriesVisibleType, x, y, voxelData,
                                                voxelIndex, invM, invProjParams, oneOverVoxelSize, mu,
                                                minmaximg[locId2]);
}
/**
 * 计算 voxel点云 中 单个像素的真实坐标 && 法向量
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] pointsMap 最终点云（真实坐标）
 * @param[out] normalsMap 点云对应的法向量
 * @param[in] pointsRay 有序点云（voxel坐标）。第四个维度是权重
 * @param[in] imgSize 有序点云对应的图像大小（x+y）
 * @param[in] voxelSize   voxel的真实尺寸
 * @param[in] lightSource 相机光心位置。用来计算夹角
 * @note 好像是raycasting专用的。对应CPU版本中有参数useSmoothing，可以使用更大的范围计算，从而达到平滑的目的
 */
template<bool flipNormals>
__global__ void renderICP_device(Vector4f *pointsMap, Vector4f *normalsMap, const Vector4f *pointsRay,
                                 float voxelSize, Vector2i imgSize, Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y) return;

  processPixelICP<true, flipNormals>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
}
/**
 * @brief 从有序点云中渲染 法向量夹角图 的所有像素（灰色）
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] outRendering 灰度图。通过邻域像素插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] pointsRay     有序点云投影到的三维点（voxel坐标）。第4个通道是有效性？？？
 * @param[in] voxelSize     voxel的真实尺寸
 * @param[in] imgSize       渲染图片的大小
 * @param[in] lightSource   相机光心位置。用来计算夹角
 * @note 对应CPU版本中有参数useSmoothing，可以使用更大的范围计算，从而达到平滑的目的
 */
template <bool flipNormals>
__global__ void renderGrey_ImageNormals_device(Vector4u *outRendering, const Vector4f *pointsRay, float voxelSize,
                                               Vector2i imgSize, Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  processPixelGrey_ImageNormals<true, flipNormals>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
}
/**
 * @brief 从有序点云中渲染 法向量图 的所有像素
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] outRendering 伪彩色图。通过邻域像素插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] ptsRay     有序点云投影到的三维点（voxel坐标）。第4个通道是有效性？？？
 * @param[in] imgSize       渲染图片的大小
 * @param[in] voxelSize     voxel的真实尺寸
 * @param[in] lightSource   相机光心位置。用来计算夹角
 * @note 对应CPU版本中有参数useSmoothing，可以使用更大的范围计算，从而达到平滑的目的
 */
template <bool flipNormals>
__global__ void renderNormals_ImageNormals_device(Vector4u *outRendering, const Vector4f *ptsRay, Vector2i imgSize,
                                                  float voxelSize, Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  processPixelNormals_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
}
/**
 * @brief 从有序点云中渲染 置信度图 的所有像素
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] outRendering 伪彩色图。通过邻域像素插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] ptsRay        有序点云投影到的三维点（voxel坐标）。第4个通道是有效性？？？
 * @param[in] imgSize       渲染图片的大小
 * @param[in] voxelSize     voxel的真实尺寸
 * @param[in] lightSource   相机光心位置。用来计算夹角
 * @note 对应CPU版本中有参数useSmoothing，可以使用更大的范围计算，从而达到平滑的目的
 */
template <bool flipNormals>
__global__ void renderConfidence_ImageNormals_device(Vector4u *outRendering, const Vector4f *ptsRay, Vector2i imgSize,
                                                     float voxelSize, Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  processPixelConfidence_ImageNormals<true, flipNormals>(outRendering, ptsRay, imgSize, x, y, voxelSize, lightSource);
}
/**
 * @brief 从三维场景中渲染 法向量夹角图 的所有像素（灰色）
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 灰色图。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] ptsRay        三维点（voxel坐标）
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] imgSize       渲染图片的大小
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <class TVoxel, class TIndex>
__global__ void renderGrey_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
                                  const typename TIndex::IndexData *voxelIndex, Vector2i imgSize,
                                  Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  int locId = x + y * imgSize.x;
  Vector4f ptRay = ptsRay[locId];

  processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
                                   lightSource);
}
/**
 * @brief 从三维场景渲染 法向量图 的所有像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 伪彩色的法向量图。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] ptsRay        三维点（voxel坐标）
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] imgSize       渲染图片的大小
 * @param[in] lightSource   相机光心位置。用来计算夹角
 * @note 是单位法向量
 */
template <class TVoxel, class TIndex>
__global__ void renderColourFromNormal_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
                                              const typename TIndex::IndexData *voxelIndex, Vector2i imgSize,
                                              Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  int locId = x + y * imgSize.x;
  Vector4f ptRay = ptsRay[locId];

  processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
                                     lightSource);
}
/**
 * @brief 从三维场景中渲染 置信度图 的所有像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 伪彩色信息。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] ptsRay        三维点（voxel坐标）
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] imgSize       渲染图片的大小
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <class TVoxel, class TIndex>
__global__ void renderColourFromConfidence_device(Vector4u *outRendering, const Vector4f *ptsRay,
                                                  const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex,
                                                  Vector2i imgSize, Vector3f lightSource) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y)
    return;

  int locId = x + y * imgSize.x;
  Vector4f ptRay = ptsRay[locId];

  processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex, lightSource);
}
/**
 * @brief 将raycast的结果转成三维点云
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] locations      三维点云
 * @param[out] colours        三维点云对应的颜色信息
 * @param[out] noTotalPoints  三维点云总数
 * @param[in] ptsRay          raycast的结果
 * @param[in] voxelData       voxel block array
 * @param[in] voxelIndex      hash table
 * @param[in] skipPoints      是否只渲染1/4的像素
 * @param[in] voxelSize       voxel size。单位米
 * @param[in] imgSize         渲染图片的
 * @param[in] lightSource     相机光心位置。取位姿的最后一列的负数
 */
template <class TVoxel, class TIndex>
__global__ void renderPointCloud_device(/*Vector4u *outRendering, */ Vector4f *locations, Vector4f *colours,
                                        uint *noTotalPoints, const Vector4f *ptsRay, const TVoxel *voxelData,
                                        const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize,
                                        Vector2i imgSize, Vector3f lightSource) {
  __shared__ bool shouldPrefix;
  shouldPrefix = false;
  __syncthreads();

  bool foundPoint = false;
  Vector3f point(0.0f);

  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x < imgSize.x && y < imgSize.y) { // TODO: 超过图像范围的不能return？？？ why???
    //! 计算法向量 && 跟相机光心的夹角
    int locId = x + y * imgSize.x;
    Vector3f outNormal;
    float angle;
    Vector4f pointRay = ptsRay[locId];
    point = pointRay.toVector3();
    foundPoint = pointRay.w > 0;
    computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);
    //! 【可选】只渲染1/4的像素
    if (skipPoints && ((x % 2 == 0) || (y % 2 == 0)))
      foundPoint = false;

    if (foundPoint)   // 只要有一个cuda thread找到了，整个cuda block都要参与前缀和计算
      shouldPrefix = true;
  }

  __syncthreads();

  if (shouldPrefix) {
    //! 通过前缀和找到当前block在最终结果中的全局位置
    int offset = computePrefixSum_device<uint>(foundPoint, noTotalPoints, blockDim.x * blockDim.y,
                                               threadIdx.x + threadIdx.y * blockDim.x);
    //! 只对有效点 获取带RGB的三维点
    if (offset != -1) {
      Vector4f tmp;
      tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
      if (tmp.w > 0.0f) { // NOTE：上面读出来都保证w==1
        tmp.x /= tmp.w;
        tmp.y /= tmp.w;
        tmp.z /= tmp.w;
        tmp.w = 1.0f;
      }
      colours[offset] = tmp;  // RGB信息(0-1)

      Vector4f pt_ray_out;  // 真实坐标
      pt_ray_out.x = point.x * voxelSize;
      pt_ray_out.y = point.y * voxelSize;
      pt_ray_out.z = point.z * voxelSize;
      pt_ray_out.w = 1.0f;
      locations[offset] = pt_ray_out;
    }
  }
}
/**
 * @brief 从三维场景渲染 彩色图 的所有像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 彩色图。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] ptsRay        三维点（voxel坐标）
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] imgSize       渲染图片的大小
 */
template<class TVoxel, class TIndex>
__global__ void renderColour_device(Vector4u *outRendering, const Vector4f *ptsRay, const TVoxel *voxelData,
                                    const typename TIndex::IndexData *voxelIndex, Vector2i imgSize) {
  int x = (threadIdx.x + blockIdx.x * blockDim.x), y = (threadIdx.y + blockIdx.y * blockDim.y);

  if (x >= imgSize.x || y >= imgSize.y) return;

  int locId = x + y * imgSize.x;

  Vector4f ptRay = ptsRay[locId];

  processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
}
}
