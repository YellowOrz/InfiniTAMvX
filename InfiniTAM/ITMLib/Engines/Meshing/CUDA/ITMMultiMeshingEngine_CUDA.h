// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiMeshingEngine.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

namespace ITMLib {
template<class TVoxel, class TIndex>
class ITMMultiMeshingEngine_CUDA : public ITMMultiMeshingEngine<TVoxel, TIndex> {
 public:
  void MeshScene(ITMMesh *mesh, const ITMVoxelMapGraphManager<TVoxel, TIndex> &sceneManager) {}
};
/** 上面模板类的特例 */
template<class TVoxel>
class ITMMultiMeshingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMMultiMeshingEngine<TVoxel, ITMVoxelBlockHash> {
 private:
  unsigned int *noTriangles_device;       // 所有三角面片的个数，记录在GPU
  Vector4s *visibleBlockGlobalPos_device; // 所有子图中有效（被分配了内存）的voxel block的block坐标

 public:
  // NOTE: ITMVoxelBlockHash是模板中的类型，因此下面必须要加typename
  typedef typename ITMMultiIndex<ITMVoxelBlockHash>::IndexData MultiIndexData;  // 多个子图的基础信息的数据结构
  typedef ITMMultiVoxel<TVoxel> MultiVoxelData;                                 // 多个子图中的voxel数据的数据结构
  typedef ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> MultiSceneManager; // 管理多子图的数据结构

  MultiIndexData *indexData_device, indexData_host; // 多个子图的基础信息
  MultiVoxelData *voxelData_device, voxelData_host; // 多个子图中的voxel数据
  /**
   * @brief 从多个子图中抽取mesh
   * @param[out] mesh
   * @param[in] sceneManager  子图管理器
   */
  void MeshScene(ITMMesh *mesh, const MultiSceneManager &sceneManager);

  ITMMultiMeshingEngine_CUDA(void);
  ~ITMMultiMeshingEngine_CUDA(void);
};
}

