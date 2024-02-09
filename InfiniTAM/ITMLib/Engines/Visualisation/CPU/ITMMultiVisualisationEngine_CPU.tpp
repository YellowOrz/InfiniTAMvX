// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMultiVisualisationEngine_CPU.h"

#include "../../../Objects/RenderStates/ITMRenderStateMultiScene.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

#include "../Shared/ITMVisualisationEngine_Shared.h"

using namespace ITMLib;

template <class TVoxel, class TIndex>
ITMRenderState *
ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::CreateRenderState(const ITMScene<TVoxel, TIndex> *scene,
                                                                   const Vector2i &imgSize) const {
  return new ITMRenderStateMultiScene<TVoxel, TIndex>(imgSize, scene->sceneParams->viewFrustum_min,
                                                      scene->sceneParams->viewFrustum_max, MEMORYDEVICE_CPU);
}

template <class TVoxel, class TIndex>
void ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::PrepareRenderState(
    const ITMVoxelMapGraphManager<TVoxel, TIndex> &mapManager, ITMRenderState *_state) {
  ITMRenderStateMultiScene<TVoxel, TIndex> *state = (ITMRenderStateMultiScene<TVoxel, TIndex> *)_state;

  state->PrepareLocalMaps(mapManager);
}

template <class TVoxel, class TIndex> // TODO: 下次从这儿开始
void ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const ORUtils::SE3Pose *pose,
                                                                           const ITMIntrinsics *intrinsics,
                                                                           ITMRenderState *_renderState) const {
  ITMRenderStateMultiScene<TVoxel, TIndex> *renderState = (ITMRenderStateMultiScene<TVoxel, TIndex> *)_renderState;

  //! 准备:获取彩色图大小 && raycast得到的图片。reset min max image
  Vector2i imgSize = renderState->renderingRangeImage->noDims;    // 深度范围图的尺寸。比渲染图片小 (见minmaximg_subsample)
  Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU); // 深度范围图
  //! 给每个像素赋值初始的最小和最大深度
  for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
    Vector2f &pixel = minmaxData[locId];
    pixel.x = FAR_AWAY;     // 最小值
    pixel.y = VERY_CLOSE;   // 最大值
  }

  //! 遍历每个子图。add the values from each local map
  for (int localMapId = 0; localMapId < renderState->indexData_host.numLocalMaps; ++localMapId) {
    float voxelSize = renderState->sceneParams.voxelSize;
    const ITMHashEntry *hash_entries = renderState->indexData_host.index[localMapId]; // 当前子图的三维场景的hash entry
    int noHashEntries = ITMVoxelBlockHash::noTotalEntries;                            // 当前子图中的entry总数

    std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);                // render小块，看下面就懂了
    int numRenderingBlocks = 0;

    Matrix4f localPose = pose->GetM() * renderState->indexData_host.posesInv[localMapId]; // 子图到世界的位姿，T_ws
    //! 遍历每个entry，找到可见的
    for (int blockNo = 0; blockNo < noHashEntries; ++blockNo) {
      const ITMHashEntry &blockData(hash_entries[blockNo]);
      // 将单个可见的block投影到 当前视角下，并计算包围盒 && 深度范围
      Vector2i upperLeft, lowerRight; // 包围盒的左上、右下坐标
      Vector2f zRange;                // 包围盒的深度范围
      bool validProjection = false;   // 当前voxel block能否投影到到当前帧（即是否可见）
      if (blockData.ptr >= 0)         // >=0表示当前voxel block有效
        validProjection = ProjectSingleBlock(blockData.pos, localPose, intrinsics->projectionParamsSimple.all, imgSize,
                                             voxelSize, upperLeft, lowerRight, zRange);
      if (!validProjection) continue;
      // 将包围盒分小块，每块大小(renderingBlockSizeX,renderingBlockSizeY)=(16,16)。ceilf是向上取整。为啥要分块渲染？？？
      Vector2i requiredRenderingBlocks(
          (int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)renderingBlockSizeX),
          (int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)renderingBlockSizeY));
      int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;  // 包围盒中小块数量
          // TODO: 按照renderingBlockSizeX和renderingBlockSizeY都为16，不可能有requiredNumBlocks>1
      if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) // 单帧中小块的数量有限制  
        continue;       // TODO:这里应该换成break,∵一次超过限制了，之后肯定都超过限制
      int offset = numRenderingBlocks;
      numRenderingBlocks += requiredNumBlocks;
      // 创建小块
      CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
    }

    // go through rendering blocks
    //! 遍历小块，确定最后raycasting像素的最大和最小深度值。分小块是为了防止 多个block的包围盒的重叠区域太多，导致浪费吗？？？
    for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) {
      // fill minmaxData
      const RenderingBlock &b(renderingBlocks[blockNo]);

      for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) { // NOTE：这里是 深度范围图 的坐标
        for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
          Vector2f &pixel(minmaxData[x + y * imgSize.x]);
          if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
          if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
        }
      }
    }
  }
}

template <class TVoxel, class TIndex>
void ITMMultiVisualisationEngine_CPU<TVoxel, TIndex>::RenderImage(const ORUtils::SE3Pose *pose,
                                                                  const ITMIntrinsics *intrinsics,
                                                                  ITMRenderState *_renderState,
                                                                  ITMUChar4Image *outputImage,
                                                                  IITMVisualisationEngine::RenderImageType type) const {
  ITMRenderStateMultiScene<TVoxel, TIndex> *renderState = (ITMRenderStateMultiScene<TVoxel, TIndex> *)_renderState;

  Vector2i imgSize = outputImage->noDims; // 渲染图片的分辨率
  Matrix4f invM = pose->GetInvM();        // 相机位姿的逆

  //! raycasting的核心部分，得到点云。Generic Raycast
  float voxelSize = renderState->sceneParams.voxelSize;
  {
    Vector4f projParams = intrinsics->projectionParamsSimple.all; // 相机内参，即fx、fy、cx(px)、cy(py)
    Vector4f invProjParams = InvertProjectionParams(projParams);  // 相机内参的逆

    const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);  // 深度范围图
    float mu = renderState->sceneParams.mu;                       // SDF的截断值对应的距离
    float oneOverVoxelSize = 1.0f / voxelSize;                    // voxel size的倒数
    Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);  // 后面要计算的ray的交点（voxel坐标）

    typedef ITMMultiVoxel<TVoxel> VD;
    typedef ITMMultiIndex<TIndex> ID;

    // 遍历每个像素，计算对应ray的值
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
      // 图像上的坐标
      int y = locId / imgSize.x;
      int x = locId - y * imgSize.x;
      // TODO：计算 归属于哪个渲染小块的ID？？但是渲染小块是16*16的，这里是按照8*8计算的？？？只把结果存在渲染图片左上角的1/8的里面？？？
      int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;
      // ray cast
      castRay<VD, ID, false>(pointsRay[locId], NULL, x, y, &renderState->voxelData_host, &renderState->indexData_host,
                             invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
    }
  }
  //! 根据渲染类型，从点云得到图片
  Vector3f lightSource = -Vector3f(invM.getColumn(2));                // 相机光心位置。取位姿的最后一列的负数
  Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);    // 后面渲染得到的图片
  Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);  // 上面raycast得到的点云

  if ((type == IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME) && (!TVoxel::hasColorInformation))
    type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;    // 想要color但是没有，强制设为grey

  switch (type) {
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME:            //! 彩色图，从三维场景中得到
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
      Vector4f ptRay = pointsRay[locId];
      processPixelColour<ITMMultiVoxel<TVoxel>, ITMMultiIndex<TIndex>>(outRendering[locId], ptRay.toVector3(),
                                                                       ptRay.w > 0, &(renderState->voxelData_host),
                                                                       &(renderState->indexData_host));
    }
    break;
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL:            //! 单位法向量的伪彩色图，从上面的点云得到
    if (intrinsics->FocalLengthSignsDiffer()) { // 相机内参的焦距为负，法向量要翻转
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
      for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
        int y = locId / imgSize.x, x = locId - y * imgSize.x; // 一维坐标转二维坐标
        processPixelNormals_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    } else {                                    // 法向量不用翻转              
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
      for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
        int y = locId / imgSize.x, x = locId - y * imgSize.x; // 一维坐标转二维坐标
        processPixelNormals_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    }
    break;
  case IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE:        //! 置信度的伪彩色图，从上面的点云得到
    if (intrinsics->FocalLengthSignsDiffer()) { // 相机内参的焦距为负，法向量要翻转。因为置信度要乘上法向量夹角
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
      for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
        int y = locId / imgSize.x, x = locId - y * imgSize.x; // 一维坐标转二维坐标
        processPixelConfidence_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    } else {                                    // 法向量不用翻转  
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
      for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
        int y = locId / imgSize.x, x = locId - y * imgSize.x; // 一维坐标转二维坐标
        processPixelConfidence_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize,
                                                         lightSource);
      }
    }
    break;
  case IITMVisualisationEngine::RENDER_SHADED_GREYSCALE:              //! 法向量夹角图（灰度），从上面的点云得到
  default:
    if (intrinsics->FocalLengthSignsDiffer()) { // 相机内参的焦距为负，法向量要翻转
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
      for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
        int y = locId / imgSize.x, x = locId - y * imgSize.x; // 一维坐标转二维坐标
        processPixelGrey_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    } else {                                    // 法向量不用翻转
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
      for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
        int y = locId / imgSize.x, x = locId - y * imgSize.x; // 一维坐标转二维坐标
        processPixelGrey_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
      }
    }
    break;
  }
}
