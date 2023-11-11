// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine_CPU.h"

#include "../../Reconstruction/Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../Shared/ITMVisualisationEngine_Shared.h"

#include <vector>

using namespace ITMLib;

template <class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u *outRendering, Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay,
                            const TVoxel *voxelData, const typename TIndex::IndexData *voxelIndex, bool skipPoints,
                            float voxelSize, Vector2i imgSize, Vector3f lightSource);

template <class TVoxel, class TIndex>
ITMRenderState *ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateRenderState(const ITMScene<TVoxel, TIndex> *scene,
                                                                              const Vector2i &imgSize) const {
  return new ITMRenderState(imgSize, scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max,
                            MEMORYDEVICE_CPU);
}

template <class TVoxel>
ITMRenderState_VH *ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreateRenderState(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i &imgSize) const {
  return new ITMRenderState_VH(ITMVoxelBlockHash::noTotalEntries, imgSize, scene->sceneParams->viewFrustum_min,
                               scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU);
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene,
                                                                   const ORUtils::SE3Pose *pose,
                                                                   const ITMIntrinsics *intrinsics,
                                                                   ITMRenderState *renderState) const {}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::FindVisibleBlocks(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
    ITMRenderState *renderState) const {
  //! 准备
  const ITMHashEntry *hashTable = scene->index.GetEntries();      // hash table
  int noTotalEntries = scene->index.noTotalEntries;               // 场景中entry总数
  float voxelSize = scene->sceneParams->voxelSize;                // voxel size。单位米
  Vector2i imgSize = renderState->renderingRangeImage->noDims;    // 深度范围图的尺寸。比渲染图片小 (见minmaximg_subsample)

  Matrix4f M = pose->GetM();                                      // 当前视角相机位姿
  Vector4f projParams = intrinsics->projectionParamsSimple.all;   // 相机内参

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *)renderState; // TODO: 父类转子类指针，最好用dynamic_cast

  int noVisibleEntries = 0;                                     // 后面找到的可见entry的数量
  int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();  // 后面找到的可见entries列表

  //! 遍历场景中所有entry，构建可见列表。build visible list
  for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
    unsigned char hashVisibleType = 0; // = entriesVisibleType[targetIdx]; // 只支持0、1两种可见类型。没有那么多花里胡哨的
    const ITMHashEntry &hashEntry = hashTable[targetIdx];

    if (hashEntry.ptr >= 0) { // 对存在的block检查可见性（不将图片扩大尺寸）
      bool isVisible, isVisibleEnlarged;
      checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
      hashVisibleType = isVisible;  // TODO: 直接把hashVisibleType传入不就好了！
    }

    if (hashVisibleType > 0) {  // 可见的就记录一下
      visibleEntryIDs[noVisibleEntries] = targetIdx;
      noVisibleEntries++;
    }
  }

  renderState_vh->noVisibleEntries = noVisibleEntries;
}

template <class TVoxel, class TIndex>
int ITMVisualisationEngine_CPU<TVoxel, TIndex>::CountVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene,
                                                                   const ITMRenderState *renderState, int minBlockId,
                                                                   int maxBlockId) const {
  return 1;
}

template <class TVoxel>
int ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CountVisibleBlocks(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId,
    int maxBlockId) const {
  const ITMRenderState_VH *renderState_vh = (const ITMRenderState_VH *)renderState;

  int noVisibleEntries = renderState_vh->noVisibleEntries;            // 可见entries数量
  const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();  // 可见entries列表

  int ret = 0;
  for (int i = 0; i < noVisibleEntries; ++i) {
    int blockID = scene->index.GetEntries()[visibleEntryIDs[i]].ptr;
    if ((blockID >= minBlockId) && (blockID <= maxBlockId)) // 只统计在指定范围内的可见entry数量
      ++ret;
  }

  return ret;
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene,
                                                                      const ORUtils::SE3Pose *pose,
                                                                      const ITMIntrinsics *intrinsics,
                                                                      ITMRenderState *renderState) const {
  //! 准备 获取彩色图大小 && raycast得到的图片
  Vector2i imgSize = renderState->renderingRangeImage->noDims;  // 深度范围图的尺寸。比渲染图片小 (见minmaximg_subsample)
  Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU); // 深度范围图
  //! 给每个像素赋值最小和最大深度（0.2-3）
  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
    // TODO : this could be improved a bit...
    Vector2f &pixel = minmaxData[locId];
    pixel.x = 0.2f; // 最小值
    pixel.y = 3.0f; // 最大值
  }
}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreateExpectedDepths(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
    ITMRenderState *renderState) const {
  //! 准备
  Vector2i imgSize = renderState->renderingRangeImage->noDims;  // 深度范围图的尺寸。比渲染图片小 (见minmaximg_subsample)
  Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU); // 深度范围图
  //! 给每个像素赋值初始的最小和最大深度
  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
    Vector2f &pixel = minmaxData[locId];
    pixel.x = FAR_AWAY;   // 最小值
    pixel.y = VERY_CLOSE; // 最大值
  }

  //! 获取可见的entry的id   
  // NOTE: UI界面中是在FindVisibleBlocks更新的，raycasting中是在融合中更新的
  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *)renderState; // TODO:为啥要转换？
  const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();
  int noVisibleEntries = renderState_vh->noVisibleEntries;

  // go through list of visible 8x8x8 blocks
  //! 遍历每个可见的entry，找到需要render的block
  std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS); // render小块，看下面就懂了  // TODO：好像没有保存啊？？？
  int numRenderingBlocks = 0;
  float voxelSize = scene->sceneParams->voxelSize;
  for (int blockNo = 0; blockNo < noVisibleEntries; ++blockNo) {
    const ITMHashEntry &blockData(scene->index.GetEntries()[visibleEntryIDs[blockNo]]);
    // 将单个可见的block投影到 当前视角下，并计算包围盒 && 深度范围
    Vector2i upperLeft, lowerRight; // 包围盒的左上、右下坐标
    Vector2f zRange;
    bool validProjection = false;
    if (blockData.ptr >= 0)         // >=0表示当前block有效
      validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize,
                                           voxelSize, upperLeft, lowerRight, zRange);
    if (!validProjection) continue;
    // 将包围盒分小块，每块大小(renderingBlockSizeX,renderingBlockSizeY)=(16,16)。ceilf是向上取整。为啥要分块渲染？？？
    Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX),
                                     (int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
    int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y; // 包围盒中小块数量
        // TODO: 按照renderingBlockSizeX和renderingBlockSizeY都为16，不可能有requiredNumBlocks>1
    if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS)
      continue; // 单帧中小块的数量有限制  // TODO:这里应该换成break,∵一次超过限制了，之后肯定都超过限制
    int offset = numRenderingBlocks;
    numRenderingBlocks += requiredNumBlocks;

    CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);  // 创建小块
  }

  // go through rendering blocks
  //! 遍历小块，确定最后raycasting像素的最大和最小深度值。分小块是为了防止 多个block的包围盒的重叠区域太多，导致浪费吗？？？
  for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
    // fill minmaxData
    const RenderingBlock &b(renderingBlocks[blockNo]);

    for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) { // NOTE：这里是 深度范围图 的坐标
      for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
        Vector2f &pixel(minmaxData[x + y * imgSize.x]);
        if (pixel.x > b.zRange.x)   pixel.x = b.zRange.x;
        if (pixel.y < b.zRange.y)   pixel.y = b.zRange.y;
      }
    }
  }
}
/**
 * @brief raycasting的核心部分，得到点云
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] scene             三维场景信息
 * @param[in] imgSize           raycasting得到的图像大小（x+y）
 * @param[in] invM              当前视角的相机 到 世界坐标系 的变换矩阵？？？
 * @param[in] projParams        相机内参，即fx、fy、cx(px)、cy(py)
 * @param[in, out] renderState 里面的renderingRangeImage给定ray的范围，raycastResult是结果点云（voxel坐标下）
 * @param[in] updateVisibleList 用于跟踪的话，=true来更新可见列表；用于UI界面的自由视角，=false不要更新，以免影响跟踪。
 *                              在CreatePointCloud_common和CreateICPMaps_common设为true，其余都是false
 */
template <class TVoxel, class TIndex>
static void GenericRaycast(const ITMScene<TVoxel, TIndex> *scene, const Vector2i &imgSize, const Matrix4f &invM,
                           const Vector4f &projParams, const ITMRenderState *renderState, bool updateVisibleList) {
  
  const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);  // 深度范围图
  float mu = scene->sceneParams->mu;                                              // SDF的截断值对应的距离
  float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;                  // voxel size的倒数
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);    // 后面要计算的ray的交点（voxel坐标）
  const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();                     // device上的voxel block array
  const typename ITMVoxelBlockHash::IndexData *voxelIndex = scene->index.getIndexData();  // hash table
  uchar *entriesVisibleType = NULL;                                               // visible entry列表
  if (updateVisibleList && (dynamic_cast<const ITMRenderState_VH *>(renderState) != NULL)) {  
    // TODO:上面用啥dynamic_cast？？？应该下面用吧？？？
    entriesVisibleType = ((ITMRenderState_VH *)renderState)->GetEntriesVisibleType();
  }
  //! 遍历每个像素，计算对应ray的值
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
    // 图像上的坐标
    int y = locId / imgSize.x;
    int x = locId - y * imgSize.x;
    // TODO：计算 归属于哪个渲染小块的ID？？？但是渲染小块是16*16的，这里是按照8*8计算的？？？只把结果存在渲染图片左上角的1/8的里面？？？
    int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

    if (entriesVisibleType != NULL)   // 可见entry不为空的话，要顺带修改了
      castRay<TVoxel, TIndex, true>(pointsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM,
                                    InvertProjectionParams(projParams), oneOverVoxelSize, mu, minmaximg[locId2]);
    else
      castRay<TVoxel, TIndex, false>(pointsRay[locId], NULL, x, y, voxelData, voxelIndex, invM,
                                     InvertProjectionParams(projParams), oneOverVoxelSize, mu, minmaximg[locId2]);
  }
}

/**
 * @brief 根据渲染类型，从raycast得到的点云中得到图片
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] scene         三维场景
 * @param[in] pose          当前视角的相机位姿。world to local
 * @param[in] intrinsics    当前视角的相机参数，用于投影图片
 * @param[in] renderState   raycast的结果，主要用到其中的raycastResult
 * @param[out] outputImage  渲染得到的图片
 * @param[in] type          渲染类型
 * @param[in] raycastType   raycast的类型
 */
template <class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                               const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                               ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type,
                               IITMVisualisationEngine::RenderRaycastSelection raycastType) {
  //! 获取raycast的渲染结果
  Vector2i imgSize = outputImage->noDims;   // 渲染图片的分辨率
  Matrix4f invM = pose->GetInvM();          // 相机位姿的逆

  Vector4f *pointsRay;  // raycast的结果
  if (raycastType == IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST)        // 取旧的普通raycast结果(age_pointCloud<=0)
    pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  else if (raycastType == IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ) // 取旧的增量raycast结果
      pointsRay = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
  else {
    // this one is generally done for freeview visualisation, so no, do not update the list of visible blocks
    // 用于UI界面的自由视角，不要更新可见列表，以免影响跟踪。
    GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState, false);
    pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  }
  //! 根据渲染类型，从点云得到图片
  Vector3f lightSource = -Vector3f(invM.getColumn(2));      // 相机光心位置。取位姿的最后一列的负数
  Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);  // 后面渲染得到的图片
  const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();   // voxel block array
  const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData(); // hash table

  if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME) && (!TVoxel::hasColorInformation))
    type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;  // 想要color但是没有，强制设为grey

  switch (type) {
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:            //! 彩色图
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
    }
    break;
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:            //! 单位法向量的伪彩色图
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
                                         lightSource);
    }
    break;
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE:        //! 置信度的伪彩色图
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {     
      Vector4f ptRay = pointsRay[locId];
      processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex,
                                             lightSource);
    }
    break;
  case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS: //! 有序点云的法向量夹角图（灰色）
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) { 
      int y = locId / imgSize.x;
      int x = locId - y * imgSize.x;

      if (intrinsics->FocalLengthSignsDiffer()) {
        processPixelGrey_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, scene->sceneParams->voxelSize,
                                                  lightSource);
      } else {
        processPixelGrey_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y,
                                                   scene->sceneParams->voxelSize, lightSource);
      }
    }
    break;
  case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:              //! 法向量夹角图（灰度）
  default:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
                                       lightSource);
    }
  // TODO: 为啥没有processPixelNormals_ImageNormals和processPixelConfidence_ImageNormals？？？
  }
}

/**
 * 根据跟踪到的当前相机位姿，使用raycast从三维场景中抽取带RGB的点云（用于下一帧的跟踪？？？）
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] scene               三维场景
 * @param[in] view                当前输入帧。主要用到其中的相机内外参
 * @param[in, out] trackingState  跟踪状态。主要用到其中的相机位姿、点云
 * @param[in, out] renderState    渲染结果。主要用到其中的raycastResult
 * @param[in] skipPoints          抽取点云的时候要不要跳过。=true的话，只保留1/4的点云
 */
template <class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                    ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) {
  Vector2i imgSize = renderState->raycastResult->noDims;                                    // 渲染图片大小
  Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib.trafo_rgb_to_depth.calib;  // RGB相机位姿。local 2 world

  // this one is generally done for the colour tracker, so yes, update the list of visible blocks if possible
  // TODO：用于带color的跟踪，∴可以的话，更新可见entry列表？？？
  //! 在RGB相机的视角下，进行raycast
  GenericRaycast(scene, imgSize, invM, view->calib.intrinsics_rgb.projectionParamsSimple.all, renderState, true);
  trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
  //! 将raycast的结果转成带RGB的三维点云
  trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(
      trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU),
      trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU),
      renderState->raycastResult->GetData(MEMORYDEVICE_CPU), scene->localVBA.GetVoxelBlocks(),
      scene->index.getIndexData(), skipPoints, scene->sceneParams->voxelSize, imgSize, -Vector3f(invM.getColumn(2)));
}
/**
 * voxel三维场景中，使用raycasting投影点云（只有几何，无纹理）？？？
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] scene         三维模型
 * @param[in] view          当前输入图像
 * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
 * @param[out] renderState  raycasting的结果
 */
template <class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                 ITMTrackingState *trackingState, ITMRenderState *renderState) {
  Vector2i imgSize = renderState->raycastResult->noDims; // 要raycast的图片大小（x+y）
  Matrix4f invM = trackingState->pose_d->GetInvM();      // 当前视角的相机 到 世界坐标系 的变换矩阵？？？

  //! 计算raycast得到的点云（voxel坐标） && 记录位姿
  // this one is generally done for the ICP tracker, so yes, update the list of visible blocks if possible
  GenericRaycast(scene, imgSize, invM, view->calib.intrinsics_d.projectionParamsSimple.all, renderState, true);
  trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

  //! 计算点云的真实坐标 && 法向量
  Vector3f lightSource = -Vector3f(invM.getColumn(2));    // 相机光心位置。取位姿的最后一列的负数
  Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);  // 最终的点云
  Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);   // 最终点云的法向量
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);            // raycast的结果
  float voxelSize = scene->sceneParams->voxelSize;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++) {
      if (view->calib.intrinsics_d.FocalLengthSignsDiffer()) {  
        processPixelICP<true, true>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
      } else {  // 特殊的数据集，计算出来的法向量要翻转
        processPixelICP<true, false>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    }
}
/**
 * 增量式的raycasting
 * @details 找出旧的raycast结果中在当前视角下找不到的，重新raycast
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] scene             三维场景信息
 * @param[in] view              当前帧。用到其中的深度图
 * @param[in] trackingState     当前帧的跟踪结果。主要用到当前深度相机的位姿
 * @param[in, out] renderState  渲染相关信息。主要用到其中的raycastResult、renderingRangeImage、forwardProjection、
 *                              fwdProjMissingPoints。前俩是in，后俩是out
 */
template <class TVoxel, class TIndex>
static void ForwardRender_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                 ITMTrackingState *trackingState, ITMRenderState *renderState) {
  //! 准备
  Vector2i imgSize = renderState->raycastResult->noDims;  // 要渲染的图像大小
  Matrix4f M = trackingState->pose_d->GetM();             // 当前帧的深度相机位姿
  Matrix4f invM = trackingState->pose_d->GetInvM();       // 位姿的逆，方便后续计算
  const Vector4f &projParams = view->calib.intrinsics_d.projectionParamsSimple.all; // 深度相机的内参

  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);        // 旧的raycast结果(voxel坐标)
  Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);  // 在当前帧能找到的（临时变量）
  int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU); // 在当前帧找不到的（临时变量）
  float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);                             // 当前输入的深度图
  const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);  // 深度范围图
  float voxelSize = scene->sceneParams->voxelSize;                                          // voxel size。单位米
  const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();                               // voxel block array
  const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();               // hash table

  renderState->forwardProjection->Clear();
  //! 遍历 旧的raycast的每一个结果，记录成功投影到当前视角的成像平面
  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++) {
      int locId = x + y * imgSize.x;
      Vector4f pixel = pointsRay[locId];  // 第四个维度是从voxel中顺带读出来的置信度

      int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize); // 将三维点投影，获取一维像素坐标
      if (locId_new >= 0)
        forwardProjection[locId_new] = pixel;
    }
  //! 找到所有需要重新raycast的像素
  int noMissingPoints = 0;
  for (int y = 0; y < imgSize.y; y++) // TODO: 为啥不跟上面的循环合并到一起？？？
    for (int x = 0; x < imgSize.x; x++) {
      int locId = x + y * imgSize.x;
      Vector4f fwdPoint = forwardProjection[locId];
      float depth = currentDepth[locId];
      // 获取当前像素对应ray 在之前获取的深度范围。注意：深度范围图 比 渲染图片 缩小了 minmaximg_subsample倍
      int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;
      Vector2f minmaxval = minmaximg[locId2];  
      // 需要重新raycast的像素 = 找不到voxel && (旧的raycast中没有能投影到 || 深度值有效) && 存在表面   // TODO: float不应该用==0
      if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) &&
          (minmaxval.x < minmaxval.y))  { // NOTE：.w是voxel的置信度(<=0说明没有voxel），minmaxval.x < .y表示ray找到了表面
      // if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
        fwdProjMissingPoints[noMissingPoints] = locId;
        noMissingPoints++;
      }
    }

  renderState->noFwdProjMissingPoints = noMissingPoints;
  const Vector4f invProjParams = InvertProjectionParams(projParams);
  //! 对需要的像素进行raycast
  for (int pointId = 0; pointId < noMissingPoints; pointId++) {
    int locId = fwdProjMissingPoints[pointId];
    int y = locId / imgSize.x, x = locId - y * imgSize.x;
    int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

    castRay<TVoxel, TIndex, false>(forwardProjection[locId], NULL, x, y, voxelData, voxelIndex, invM, invProjParams,
                                   1.0f / scene->sceneParams->voxelSize, scene->sceneParams->mu, minmaximg[locId2]);
  }
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::RenderImage(
    const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
    const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type,
    IITMVisualisationEngine::RenderRaycastSelection raycastType) const {
  RenderImage_common(scene, pose, intrinsics, renderState, outputImage, type, raycastType);
}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::RenderImage(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
    const ITMRenderState *renderState, ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type,
    IITMVisualisationEngine::RenderRaycastSelection raycastType) const {
  RenderImage_common(scene, pose, intrinsics, renderState, outputImage, type, raycastType);
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::FindSurface(const ITMScene<TVoxel, TIndex> *scene,
                                                             const ORUtils::SE3Pose *pose,
                                                             const ITMIntrinsics *intrinsics,
                                                             const ITMRenderState *renderState) const {
  // this one is generally done for freeview visualisation, so no, do not update the list of visible blocks
  // 用于UI界面的自由视角，不要更新可见列表，以免影响跟踪。
  GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all,
                 renderState, false);
}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::FindSurface(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
    const ITMRenderState *renderState) const {
  // this one is generally done for freeview visualisation, so no, do not update the list of visible blocks
  // 用于UI界面的自由视角，不要更新可见列表，以免影响跟踪。
  GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all,
                 renderState, false);
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene,
                                                                  const ITMView *view, ITMTrackingState *trackingState,
                                                                  ITMRenderState *renderState, bool skipPoints) const {
  CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreatePointCloud(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
    ITMRenderState *renderState, bool skipPoints) const {
  CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene,
                                                               const ITMView *view, ITMTrackingState *trackingState,
                                                               ITMRenderState *renderState) const {
  CreateICPMaps_common(scene, view, trackingState, renderState);
}


template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::CreateICPMaps(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
    ITMRenderState *renderState) const {
  CreateICPMaps_common(scene, view, trackingState, renderState);
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::ForwardRender(const ITMScene<TVoxel, TIndex> *scene,
                                                               const ITMView *view, ITMTrackingState *trackingState,
                                                               ITMRenderState *renderState) const {
  ForwardRender_common(scene, view, trackingState, renderState);
}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::ForwardRender(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState,
    ITMRenderState *renderState) const {
  ForwardRender_common(scene, view, trackingState, renderState);
}
/**
 * @brief 将raycast的结果转成三维点云
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] locations  三维点云
 * @param[out] colours    三维点云对应的颜色信息
 * @param[in] ptsRay      raycast的结果
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] skipPoints  是否只渲染1/4的像素
 * @param[in] voxelSize   voxel size。单位米
 * @param[in] imgSize     渲染图片的
 * @param[in] lightSource 相机光心位置。取位姿的最后一列的负数
 * @return int 三维点云中的总点数
 */
template <class TVoxel, class TIndex>
static int RenderPointCloud(Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, const TVoxel *voxelData,
                            const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize,
                            Vector2i imgSize, Vector3f lightSource) {
  int noTotalPoints = 0;
  // 遍历每个要渲染的像素
  for (int y = 0, locId = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++, locId++) {
      //! 计算法向量 && 跟相机光心的夹角
      Vector3f outNormal;   // TODO：没用到哇？？？难道只是为了保证foundPoint的准确性？？？这样的话不用算法向和角度吧？
      float angle;
      Vector4f pointRay = ptsRay[locId];
      Vector3f point = pointRay.toVector3();
      bool foundPoint = pointRay.w > 0;

      computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);
      //! 【可选】只渲染1/4的像素
      if (skipPoints && ((x % 2 == 0) || (y % 2 == 0)))
        foundPoint = false;
      //! 只对有效点 获取带RGB的三维点
      if (foundPoint) {
        Vector4f tmp;       
        tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
        if (tmp.w > 0.0f) { // NOTE：上面读出来都保证w==1
          tmp.x /= tmp.w;
          tmp.y /= tmp.w;
          tmp.z /= tmp.w;
          tmp.w = 1.0f;
        }
        colours[noTotalPoints] = tmp; // RGB信息(0-1)

        Vector4f pt_ray_out;  // 真实坐标
        pt_ray_out.x = point.x * voxelSize;
        pt_ray_out.y = point.y * voxelSize;
        pt_ray_out.z = point.z * voxelSize;
        pt_ray_out.w = 1.0f;
        locations[noTotalPoints] = pt_ray_out;

        noTotalPoints++;
      }
    }

  return noTotalPoints;
}
