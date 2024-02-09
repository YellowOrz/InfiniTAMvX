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
  unsigned int *noTriangles_device;
  Vector4s *visibleBlockGlobalPos_device;

 public:
 // NOTE: ITMVoxelBlockHash是模板中的类型，因此下面必须要加typename
  typedef typename ITMMultiIndex<ITMVoxelBlockHash>::IndexData MultiIndexData;
  typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
  typedef ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> MultiSceneManager;

  MultiIndexData *indexData_device, indexData_host; // 多个子图的基础信息
  MultiVoxelData *voxelData_device, voxelData_host; // 多个子图中的voxel数据结构

  void MeshScene(ITMMesh *mesh, const MultiSceneManager &sceneManager);

  ITMMultiMeshingEngine_CUDA(void);
  ~ITMMultiMeshingEngine_CUDA(void);
};
}

