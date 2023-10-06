// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib {
/** @brief
 * 重建模块。负责维护一个ITMLib::Objects::ITMScene，将输入图像融入场景中。
    Interface to engines implementing the main KinectFusion depth integration process.
    These classes basically manage an ITMLib::Objects::ITMScene and fuse new image information into them.
*/
template <class TVoxel, class TIndex> class ITMSceneReconstructionEngine {
public:
  /**
   * @brief 重置场景三维模型
   * Clear and reset a scene to set up a new empty one.
   * @param[in,out] scene 场景三维模型
   */
  virtual void ResetScene(ITMScene<TVoxel, TIndex> *scene) = 0;

   /**
   * 根据深度图 找到所有可见的entry（或者叫block），没有分配内存的分配一下（同时更新hash table）
   * Given a view with a new depth image, compute the visible blocks, allocate them and update the hash table so that 
   * the new image data can be integrated.
   * @details 这是fusion的准备工作，找到fusion会用到的block，从而节省计算量
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @param[in,out] scene 三维场景的相关信息。更新其中voxelAllocationList、excessAllocationList、hashTable、swapStates等
   * @param[in] view 当前输入图像
   * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
   * @param[out] renderState 更新其中 可见entry的id数组和数量
   * @param[in] onlyUpdateVisibleList 是否只更新可见列表。=true的话，关闭swap
   * @param[in] resetVisibleList 是否需要重新检查可见性？？？。默认为false
   */
  virtual void AllocateSceneFromDepth(ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                      const ITMTrackingState *trackingState, const ITMRenderState *renderState,
                                      bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

   /**
   * 根据可见列表，将当前输入的单帧融入场景中。
   * Update the voxel blocks by integrating depth and possibly colour information from the given view.
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @param[in,out] scene 三维场景
   * @param[in] view 当前输入图像
   * @param[in] trackingState 存储一些关于当前跟踪状态的内部变量，最重要的是相机姿势
   * @param[in] renderState 渲染相关数据。主要用到其中的可见entry列表
   */
  virtual void IntegrateIntoScene(ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                  const ITMTrackingState *trackingState, const ITMRenderState *renderState) = 0;

  ITMSceneReconstructionEngine(void) {}
  virtual ~ITMSceneReconstructionEngine(void) {}
};
} // namespace ITMLib
