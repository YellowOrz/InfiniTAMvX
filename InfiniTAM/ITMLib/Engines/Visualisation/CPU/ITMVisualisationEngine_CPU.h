// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMVisualisationEngine.h"

namespace ITMLib {
template <class TVoxel, class TIndex> class ITMVisualisationEngine_CPU : public ITMVisualisationEngine<TVoxel, TIndex> {
public:
  explicit ITMVisualisationEngine_CPU(void) {}
  ~ITMVisualisationEngine_CPU(void) {}

  ITMRenderState *CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i &imgSize) const;
  void FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  int CountVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMRenderState *renderState, int minBlockId,
                         int maxBlockId) const;
  /**
   * @brief 将raycasting中每条ray的最小和最大深度（即搜索范围）设置为常数。用来辅助后面 更快地投影 图像
   * @param[in] scene 三维场景信息。主要用到其中的voxel size和hash table
   * @param[in] pose 当前相机位姿。world to local
   * @param[in] intrinsics 相机内参，用于投影图片
   * @param[out] renderState 渲染相关变量。主要用到其中的renderingRangeImage，来记录raycasting中每条ray的深度范围
   * @note 用于UI界面可视化（自由视角） && 跟踪投影的raycast
   */
  void CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  /**
   * @brief 渲染图片  // TODO: 下次从这儿开始
   * 
   * @param scene 
   * @param pose 
   * @param intrinsics 
   * @param renderState 
   * @param outputImage 
   * @param type 
   * @param raycastType 
   * @note 主要是给UI界面用
   */
  void RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                   const ITMRenderState *renderState, ITMUChar4Image *outputImage,
                   IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
                   IITMVisualisationEngine::RenderRaycastSelection raycastType =
                       IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
  void FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                   const ITMRenderState *renderState) const;
  void CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                        ITMRenderState *renderState, bool skipPoints) const;
  void CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                     ITMRenderState *renderState) const;
  void ForwardRender(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                     ITMRenderState *renderState) const;
};

/** 上面ITMVisualisationEngine_CPU的偏特化，针对voxel hashing */
template <class TVoxel>
class ITMVisualisationEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash> {
public:
  explicit ITMVisualisationEngine_CPU(void) {}
  ~ITMVisualisationEngine_CPU(void) {}

  ITMRenderState_VH *CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i &imgSize) const;
  void FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  int CountVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState,
                         int minBlockId, int maxBlockId) const;
  /**
   * @brief 根据可见的block的投影，设置 raycasting中每条ray的搜索范围（最大最小深度）。用来辅助后面 更快地投影 图像
   * @param[in] scene 三维场景信息。主要用到其中的voxel size和hash table
   * @param[in] pose 当前相机位姿。world to local
   * @param[in] intrinsics 相机内参，用于投影图片
   * @param[out] renderState 渲染相关变量。主要用到其中的renderingRangeImage，来记录raycasting中每条ray的深度范围
   * @note 用于UI界面可视化（自由视角） && 跟踪投影的raycast
   */
  void CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  /**
   * @brief 渲染图片  // TODO: 下次从这儿开始
   * 
   * @param scene 
   * @param pose 
   * @param intrinsics 
   * @param renderState 
   * @param outputImage 
   * @param type 
   * @param raycastType 
   * @note 主要是给UI界面用
   */
  void RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                   const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, ITMUChar4Image *outputImage,
                   IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
                   IITMVisualisationEngine::RenderRaycastSelection raycastType =
                       IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
  void FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                   const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
  void CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                        ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
  void CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                     ITMTrackingState *trackingState, ITMRenderState *renderState) const;
  void ForwardRender(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                     ITMTrackingState *trackingState, ITMRenderState *renderState) const;
};
} // namespace ITMLib
