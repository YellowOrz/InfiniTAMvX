// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiVisualisationEngine.h"

namespace ITMLib {
template<class TVoxel, class TIndex>
class ITMMultiVisualisationEngine_CPU : public ITMMultiVisualisationEngine<TVoxel, TIndex> {
 public:
  ITMMultiVisualisationEngine_CPU(void) {}
  ~ITMMultiVisualisationEngine_CPU(void) {}

  ITMRenderState *CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i &imgSize) const;
  /**
   * @brief 准备所有子图的相关信息，比如三维场景、位姿等
   * @param[in] sceneManager  管理所有子图
   * @param[out] state        渲染结果        
   */
  void PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, TIndex> &sceneManager, ITMRenderState *state);
  /**
   * @brief 将raycasting中每条ray的最小和最大深度（即搜索范围）设置为常数。用来辅助后面 更快地投影 图像
   * @param[in] pose          当前相机位姿。world to local
   * @param[in] intrinsics    相机内参，用于投影图片
   * @param[out] renderState  渲染相关变量。主要用到其中的renderingRangeImage，来记录raycasting中每条ray的深度范围
   * @note 用于UI界面可视化（自由视角） && 跟踪投影的raycast
   */
  void CreateExpectedDepths(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                            ITMRenderState *renderState) const;
  /**
   * @brief 根据渲染类型，从raycast得到的点云中得到图片
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[in] renderState   raycast的结果，主要用到其中的raycastResult
   * @param[out] outputImage  渲染得到的图片
   * @param[in] type          渲染类型
   */
  void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState,
                   ITMUChar4Image *outputImage, IITMVisualisationEngine::RenderImageType type) const;
};
}

