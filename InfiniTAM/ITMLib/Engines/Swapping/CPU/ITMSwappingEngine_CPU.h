// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSwappingEngine.h"

namespace ITMLib {
template<class TVoxel, class TIndex>
class ITMSwappingEngine_CPU : public ITMSwappingEngine<TVoxel, TIndex> {
 public:
  void IntegrateGlobalIntoLocal(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
  void SaveToGlobalMemory(ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
  void CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState) {}
};

/**
 * @brief CPU和GPU之间的数据交换。
 * @note 上面模板类的偏特化。
 *       这个类目前仅用于调试 —— 将 CPU 内存交换到 CPU 内存？？？
 *       这可能会从其他地方(磁盘、数据库等)流入主机内存。
 *   // This class is currently just for debugging purposes -- swaps CPU memory to CPU memory. 
 *   // Potentially this could stream into the host memory from somwhere else (disk, database, etc.).
 * @tparam TVoxel 
 */
template<class TVoxel>
class ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine<TVoxel, ITMVoxelBlockHash> {
 private:
  /**
   * 从host voxel memory中读取swapstate=1的block数据
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @param[in,out] scene 三维场景数据，包含swap所需数据。主要更新了globalCache中的neededEntryIDs和hasSyncedData
   *                      TODO: 这里应该直接传入globalCache更方便
   * @return 读取的entry数量（即swap state=1的数量）
   */
  int LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

 public:
  /**
   * @brief swap in。将Host中swapstate=1的bloxk跟device进行融合
   * @param[in,out] scene 三维场景数据，包含swap所需数据  
   * @param renderState 【没用到】
   */
  void IntegrateGlobalIntoLocal(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
  /**
   * @brief swap out。将device中所有block跟host进行融合？？？
   * @param[in,out] scene 三维场景数据，包含swap所需数据  
   * @param[in] renderState 主要用到其中的entry可见信息
   */
  void SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
  /**
   * @brief  // TODO: 下次从这儿开始
   * @param scene 
   * @param renderState 
   */
  void CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);

  ITMSwappingEngine_CPU(void);
  ~ITMSwappingEngine_CPU(void);
};
}
