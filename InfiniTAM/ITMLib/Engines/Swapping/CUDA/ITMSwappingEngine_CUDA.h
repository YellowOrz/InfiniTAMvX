// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSwappingEngine.h"

namespace ITMLib {
template<class TVoxel, class TIndex>
class ITMSwappingEngine_CUDA : public ITMSwappingEngine<TVoxel, TIndex> {
 public:
  void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
  void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
  void CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState) {}
};

/**
 * @brief CPU和GPU之间的数据交换。
 * @note 上面模板类的偏特化。
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 */
template<class TVoxel>
class ITMSwappingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine<TVoxel, ITMVoxelBlockHash> {
 private:
  int *noNeededEntries_device, 
  int *noAllocatedVoxelEntries_device;
  int *entriesToClean_device;
  /**
   * 从host voxel memory中读取swapstate=1的block数据
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @param[in,out] scene 三维场景数据，包含swap所需数据。主要更新了globalCache中的neededEntryIDs和hasSyncedData
   *                      TODO: 这里应该直接传入globalCache更方便
   * @return 读取的entry数量（即swap state=1的数量）
   * @note 该函数是下面IntegrateGlobalIntoLocal()的第一步，所以放在private中
   */
  int LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

 public:
   /**
   * swap in。将Host中swapstate=1的block融入device，
   * @param[in,out] scene 三维场景数据，包含swap所需数据  
   * @param renderState 【没用到】
   */
  void IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
  /**
   * swap out。将device中swap state=2 && 不可见的block转移到host
   * @param[in,out] scene 三维场景数据，包含swap所需数据  
   * @param[in] renderState 主要用到其中的entry可见信息
   */
  void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
  /**
   * swap out。将device中不可见的block转移到host（不管swap state是多少）
   * @param[in,out] scene 三维场景数据，包含swap所需数据  
   * @param[in] renderState 主要用到其中的entry可见信息
   */
  void CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);

  ITMSwappingEngine_CUDA(void);
  ~ITMSwappingEngine_CUDA(void);
};
}
