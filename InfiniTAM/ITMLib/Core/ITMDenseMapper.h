// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"
#include "../Engines/Swapping/Interface/ITMSwappingEngine.h"
#include "../Utils/ITMLibSettings.h"

namespace ITMLib {
/** 负责 场景三维模型的融合 && swap in/out */
template <class TVoxel, class TIndex> class ITMDenseMapper {
private:
  ITMSceneReconstructionEngine<TVoxel, TIndex> *sceneRecoEngine; // 用于三维模型融合
  ITMSwappingEngine<TVoxel, TIndex> *swappingEngine;             // 用于CPU和GPU之间的数据交换

  ITMLibSettings::SwappingMode swappingMode; // swap默认关闭。在TIMLibSetting中开启

public:
  /**
   * @brief 重置场景三维模型
   * @param[in,out] scene 场景三维模型
   */
  void ResetScene(ITMScene<TVoxel, TIndex> *scene) const;

  /**
   * @brief 融合单帧到三维模型中。
   * Process a single frame
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] view 要被融合的图像。也是tracking的输入图像
   * @param[in] trackingState 只用到其中的 位姿
   * @param[in, out] scene 三维模型
   * @param[in, out] renderState
   * @param[in] resetVisibleList 是否重置可见列表
   */
  void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel, TIndex> *scene,
                    ITMRenderState *renderState_live, bool resetVisibleList = false);

  /**
   * 更新可见列表
   * Update the visible list (this can be called to update the visible list when fusion is turned off)
   * @param[in] view 当前输入图像
   * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
   * @param[in] scene voxel的三维模型
   * @param[out] renderState raycasting的结果
   * @param resetVisibleList
   * @note 只有在融合成功后才调用???英文说在fusion被关闭后（还是说完成后），可以调用这个函数？？？
   */
  void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, ITMScene<TVoxel, TIndex> *scene,
                         ITMRenderState *renderState, bool resetVisibleList = false);

  /** \brief Constructor
      Ommitting a separate image size for the depth images
      will assume same resolution as for the RGB images.
  */
  explicit ITMDenseMapper(const ITMLibSettings *settings);
  ~ITMDenseMapper();
};
} // namespace ITMLib
