// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMPlainVoxelArray.h"
#include "../Interface/ITMSceneReconstructionEngine.h"

namespace ITMLib {
/**这个模板类都是空的，还要他有啥用？？？*/
template <class TVoxel, class TIndex>
class ITMSceneReconstructionEngine_CPU : public ITMSceneReconstructionEngine<TVoxel, TIndex> {};

/** 上面模板类的偏特化，针对hashing索引的三维模型 */
template <class TVoxel>
class ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>
    : public ITMSceneReconstructionEngine<TVoxel, ITMVoxelBlockHash> {
protected:
  /** 要分配的空间类型（其实是位置）。=1存放于orderneeds allocation，=2存放于excess list*/
  ORUtils::MemoryBlock<unsigned char> *entriesAllocType;
  ORUtils::MemoryBlock<Vector4s> *blockCoords;

public:
  void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);
  /**
   * 根据深度图 计算visible list、分配内存 && 更新hash table  ？？？？？？？？？？？？？？？？？？？？？
   * @tparam TVoxel
   * @param[in,out] scene 更新
   * @param[in] view 当前输入图像
   * @param[in] trackingState 主要用到track得到的位姿
   * @param[out] renderState 获取其中 可见entry的id数组和数量
   * @param[in] onlyUpdateVisibleList 是否只更新可见列表。=true的话，关闭swap
   * @param[in] resetVisibleList 是否需要重新检查可见性？？？。默认为false
   */
  void AllocateSceneFromDepth(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                              const ITMTrackingState *trackingState, const ITMRenderState *renderState,
                              bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

  void IntegrateIntoScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                          const ITMTrackingState *trackingState, const ITMRenderState *renderState);

  ITMSceneReconstructionEngine_CPU(void);
  ~ITMSceneReconstructionEngine_CPU(void);
};

/** 上面模板类的偏特化，针对下标索引的三维模型 */
template <class TVoxel>
class ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>
    : public ITMSceneReconstructionEngine<TVoxel, ITMPlainVoxelArray> {
public:
  void ResetScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

  void AllocateSceneFromDepth(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
                              const ITMTrackingState *trackingState, const ITMRenderState *renderState,
                              bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

  void IntegrateIntoScene(ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view,
                          const ITMTrackingState *trackingState, const ITMRenderState *renderState);

  ITMSceneReconstructionEngine_CPU(void);
  ~ITMSceneReconstructionEngine_CPU(void);
};
} // namespace ITMLib
