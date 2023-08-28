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
  ORUtils::MemoryBlock<unsigned char> *entriesAllocType;
  ORUtils::MemoryBlock<Vector4s> *blockCoords;

public:
  void ResetScene(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);
  /**
   * 根据深度图 计算visible list、分配内存 && 更新hash table
   * @tparam TVoxel
   * @param scene
   * @param view
   * @param trackingState
   * @param renderState
   * @param onlyUpdateVisibleList
   * @param[in] resetVisibleList 重置visible list。默认为false
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
