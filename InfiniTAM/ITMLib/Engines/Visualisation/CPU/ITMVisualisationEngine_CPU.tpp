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
  const ITMHashEntry *hashTable = scene->index.GetEntries();
  int noTotalEntries = scene->index.noTotalEntries;
  float voxelSize = scene->sceneParams->voxelSize;
  Vector2i imgSize = renderState->renderingRangeImage->noDims;

  Matrix4f M = pose->GetM();
  Vector4f projParams = intrinsics->projectionParamsSimple.all;

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *)renderState;

  int noVisibleEntries = 0;
  int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

  // build visible list
  for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
    unsigned char hashVisibleType = 0; // = entriesVisibleType[targetIdx];
    const ITMHashEntry &hashEntry = hashTable[targetIdx];

    if (hashEntry.ptr >= 0) {
      bool isVisible, isVisibleEnlarged;
      checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M, projParams, voxelSize, imgSize);
      hashVisibleType = isVisible;
    }

    if (hashVisibleType > 0) {
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

  int noVisibleEntries = renderState_vh->noVisibleEntries;
  const int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();

  int ret = 0;
  for (int i = 0; i < noVisibleEntries; ++i) {
    int blockID = scene->index.GetEntries()[visibleEntryIDs[i]].ptr;
    if ((blockID >= minBlockId) && (blockID <= maxBlockId))
      ++ret;
  }

  return ret;
}

template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene,
                                                                      const ORUtils::SE3Pose *pose,
                                                                      const ITMIntrinsics *intrinsics,
                                                                      ITMRenderState *renderState) const {
  //! 获取彩色图大小 && raycast得到的图片
  Vector2i imgSize = renderState->renderingRangeImage->noDims;
  Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
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
  //! 获取彩色图大小 && raycast得到的图片
  Vector2i imgSize = renderState->renderingRangeImage->noDims;
  Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
  //! 给每个像素赋值最小和最大深度
  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
    Vector2f &pixel = minmaxData[locId];
    pixel.x = FAR_AWAY;   // 最小值
    pixel.y = VERY_CLOSE; // 最大值
  }

  //! 获取可见的entry的id   是在哪儿更新的呢？？？
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
    if (blockData.ptr >= 0) {
      validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all, imgSize,
                                           voxelSize, upperLeft, lowerRight, zRange);
    }
    if (!validProjection)
      continue;
    // 分小块，每块大小(renderingBlockSizeX,renderingBlockSizeY)=(16,16)。ceilf是向上取整。为啥要分块渲染？？？
    Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX),
                                     (int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
    printf("=========(renderingBlockSizeX,renderingBlockSizeY)=(%d,%d)\n", renderingBlockSizeX, renderingBlockSizeY);
    printf("=========(requiredRenderingBlocks.x,requiredRenderingBlocks.y)=(%d,%d)\n", requiredRenderingBlocks.x, requiredRenderingBlocks.y);
    int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y; // 分块数量

    if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS)
      continue; // raycast单帧中小块的数量有限制  // TODO:这里应该换成break
    int offset = numRenderingBlocks;
    numRenderingBlocks += requiredNumBlocks;

    CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
  }

  // go through rendering blocks
  //! 遍历小块，确定最后raycasting像素的最大和最小深度值。分小块是为了防止 多个block的包围盒的重叠区域太多，导致浪费吗？？？
  for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
    // fill minmaxData
    const RenderingBlock &b(renderingBlocks[blockNo]);

    for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
      for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
        Vector2f &pixel(minmaxData[x + y * imgSize.x]);
        if (pixel.x > b.zRange.x)
          pixel.x = b.zRange.x;
        if (pixel.y < b.zRange.y)
          pixel.y = b.zRange.y;
      }
    }
  }
}

/**
 * @brief 
 * @tparam TVoxel
 * @tparam TIndex
 * @param[in] scene 三维场景信息
 * @param[in] imgSize raycasting得到的图像大小（x+y）
 * @param[in] invM 当前视角的相机 到 世界坐标系 的变换矩阵？？？
 * @param[in] projParams 相机内参，即fx、fy、cx(px)、cy(py)
 * @param[in, out] renderState 
 * @param updateVisibleList 在CreatePointCloud_common和CreateICPMaps_common设为true，其余都是false
 */
template <class TVoxel, class TIndex>
static void GenericRaycast(const ITMScene<TVoxel, TIndex> *scene, const Vector2i &imgSize, const Matrix4f &invM,
                           const Vector4f &projParams, const ITMRenderState *renderState, bool updateVisibleList) {
  
  const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);  // 之前计算好的渲染小块
  float mu = scene->sceneParams->mu;                                              // voxel size=0.005的话，=0.02？？？
  float oneOverVoxelSize = 1.0f / scene->sceneParams->voxelSize;                  // voxel size的倒数
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);    // 后面要计算的ray的交点
  const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();                     // voxel block array？？？
  const typename ITMVoxelBlockHash::IndexData *voxelIndex = scene->index.getIndexData();  // hash table？？？
  uchar *entriesVisibleType = NULL;
  if (updateVisibleList && (dynamic_cast<const ITMRenderState_VH *>(renderState) != NULL)) {  // TODO:为啥用dynamic_cast？？？
    entriesVisibleType = ((ITMRenderState_VH *)renderState)->GetEntriesVisibleType();   // visible entry列表
  }
  //! 遍历每个像素，计算对应ray的值
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
    // 图像上的坐标
    int y = locId / imgSize.x;
    int x = locId - y * imgSize.x;
    // 计算 归属于哪个渲染小块的ID？？？但是渲染小块是16*16的，这里是按照8*8计算的？？？
    int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

    if (entriesVisibleType != NULL)
      castRay<TVoxel, TIndex, true>(pointsRay[locId], entriesVisibleType, x, y, voxelData, voxelIndex, invM,
                                    InvertProjectionParams(projParams), oneOverVoxelSize, mu, minmaximg[locId2]);
    else
      castRay<TVoxel, TIndex, false>(pointsRay[locId], NULL, x, y, voxelData, voxelIndex, invM,
                                     InvertProjectionParams(projParams), oneOverVoxelSize, mu, minmaximg[locId2]);
  }
}

template <class TVoxel, class TIndex>
static void RenderImage_common(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                               const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                               ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type,
                               IITMVisualisationEngine::RenderRaycastSelection raycastType) {
  Vector2i imgSize = outputImage->noDims;
  Matrix4f invM = pose->GetInvM();

  Vector4f *pointsRay;
  if (raycastType == IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST)
    pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  else {
    if (raycastType == IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ)
      pointsRay = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
    else {
      // this one is generally done for freeview visualisation, so
      // no, do not update the list of visible blocks
      GenericRaycast(scene, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState, false);
      pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
    }
  }

  Vector3f lightSource = -Vector3f(invM.getColumn(2));
  Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
  const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

  if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME) && (!TVoxel::hasColorInformation))
    type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;

  switch (type) {
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex);
    }
    break;
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
                                         lightSource);
    }
    break;
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex,
                                             lightSource);
    }
    break;
  case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS:
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
  case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:
  default:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData, voxelIndex,
                                       lightSource);
    }
  }
}

/**
 *
 * @tparam TVoxel
 * @tparam TIndex
 * @param scene
 * @param view
 * @param trackingState
 * @param renderState
 * @param skipPoints
 */
template <class TVoxel, class TIndex>
static void CreatePointCloud_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                    ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) {
  Vector2i imgSize = renderState->raycastResult->noDims;
  Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib.trafo_rgb_to_depth.calib;

  // this one is generally done for the colour tracker, so yes, update
  // the list of visible blocks if possible
  GenericRaycast(scene, imgSize, invM, view->calib.intrinsics_rgb.projectionParamsSimple.all, renderState, true);
  trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

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
 * @param[in] scene 三维模型
 * @param[in] view 当前输入图像
 * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
 * @param[out] renderState raycasting的结果
 */
template <class TVoxel, class TIndex>
static void CreateICPMaps_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                 ITMTrackingState *trackingState, ITMRenderState *renderState) {
  Vector2i imgSize = renderState->raycastResult->noDims; // 要raycast的图片大小（x+y）
  Matrix4f invM = trackingState->pose_d->GetInvM();      // 当前视角的相机 到 世界坐标系 的变换矩阵？？？

  //! 计算raycast得到的点云 && 记录位姿
  // this one is generally done for the ICP tracker, so yes, update
  // the list of visible blocks if possible
  GenericRaycast(scene, imgSize, invM, view->calib.intrinsics_d.projectionParamsSimple.all, renderState, true);
  trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

  Vector3f lightSource = -Vector3f(invM.getColumn(2));    // 取平移
  Vector4f *normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
  Vector4f *pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  float voxelSize = scene->sceneParams->voxelSize;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++) {
      if (view->calib.intrinsics_d.FocalLengthSignsDiffer()) {
        processPixelICP<true, true>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
      } else {
        processPixelICP<true, false>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    }
}

template <class TVoxel, class TIndex>
static void ForwardRender_common(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                 ITMTrackingState *trackingState, ITMRenderState *renderState) {
  Vector2i imgSize = renderState->raycastResult->noDims;
  Matrix4f M = trackingState->pose_d->GetM();
  Matrix4f invM = trackingState->pose_d->GetInvM();
  const Vector4f &projParams = view->calib.intrinsics_d.projectionParamsSimple.all;

  const Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
  Vector4f *forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
  float *currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
  int *fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
  const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
  float voxelSize = scene->sceneParams->voxelSize;
  const TVoxel *voxelData = scene->localVBA.GetVoxelBlocks();
  const typename TIndex::IndexData *voxelIndex = scene->index.getIndexData();

  renderState->forwardProjection->Clear();

  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++) {
      int locId = x + y * imgSize.x;
      Vector4f pixel = pointsRay[locId];

      int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
      if (locId_new >= 0)
        forwardProjection[locId_new] = pixel;
    }

  int noMissingPoints = 0;
  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++) {
      int locId = x + y * imgSize.x;
      int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

      Vector4f fwdPoint = forwardProjection[locId];
      Vector2f minmaxval = minmaximg[locId2];
      float depth = currentDepth[locId];

      if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) &&
          (minmaxval.x < minmaxval.y))
      // if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
      {
        fwdProjMissingPoints[noMissingPoints] = locId;
        noMissingPoints++;
      }
    }

  renderState->noFwdProjMissingPoints = noMissingPoints;
  const Vector4f invProjParams = InvertProjectionParams(projParams);

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
  // this one is generally done for freeview visualisation, so no, do not
  // update the list of visible blocks
  GenericRaycast(scene, renderState->raycastResult->noDims, pose->GetInvM(), intrinsics->projectionParamsSimple.all,
                 renderState, false);
}

template <class TVoxel>
void ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash>::FindSurface(
    const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
    const ITMRenderState *renderState) const {
  // this one is generally done for freeview visualisation, so no, do not
  // update the list of visible blocks
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

/**
 * voxel三维场景中，使用raycasting投影点云（只有几何，无纹理）？？？
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] scene 三维模型
 * @param[in] view 当前输入图像
 * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
 * @param[out] renderState raycasting的结果
 */
template <class TVoxel, class TIndex>
void ITMVisualisationEngine_CPU<TVoxel, TIndex>::CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene,
                                                               const ITMView *view, ITMTrackingState *trackingState,
                                                               ITMRenderState *renderState) const {
  CreateICPMaps_common(scene, view, trackingState, renderState);
}

/**
 * 使用voxel+哈希的三维场景下，使用raycasting投影点云（只有几何，无纹理）？？？
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in] scene 三维模型
 * @param[in] view 当前输入图像
 * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
 * @param[out] renderState raycasting的结果
 * @note 为啥要单独再弄一个函数呢？？？
 */
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

template <class TVoxel, class TIndex>
static int RenderPointCloud(Vector4f *locations, Vector4f *colours, const Vector4f *ptsRay, const TVoxel *voxelData,
                            const typename TIndex::IndexData *voxelIndex, bool skipPoints, float voxelSize,
                            Vector2i imgSize, Vector3f lightSource) {
  int noTotalPoints = 0;

  for (int y = 0, locId = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++, locId++) {
      Vector3f outNormal;
      float angle;
      Vector4f pointRay = ptsRay[locId];
      Vector3f point = pointRay.toVector3();
      bool foundPoint = pointRay.w > 0;

      computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

      if (skipPoints && ((x % 2 == 0) || (y % 2 == 0)))
        foundPoint = false;

      if (foundPoint) {
        Vector4f tmp;
        tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex, point);
        if (tmp.w > 0.0f) {
          tmp.x /= tmp.w;
          tmp.y /= tmp.w;
          tmp.z /= tmp.w;
          tmp.w = 1.0f;
        }
        colours[noTotalPoints] = tmp;

        Vector4f pt_ray_out;
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
