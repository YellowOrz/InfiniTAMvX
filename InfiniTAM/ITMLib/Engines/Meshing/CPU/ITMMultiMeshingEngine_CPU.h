// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiMeshingEngine.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

namespace ITMLib {
/** CPU版的多子图抽网格接口 */
template<class TVoxel, class TIndex>
class ITMMultiMeshingEngine_CPU : public ITMMultiMeshingEngine<TVoxel, TIndex> {
 public:
  /**
   * @brief 从多个子图中抽取mesh
   * @param[out] mesh
   * @param[in] sceneManager  子图管理器
   */
  void MeshScene(ITMMesh *mesh, const ITMVoxelMapGraphManager<TVoxel, TIndex> &sceneManager) {}
};

/** CPU版的多子图抽网格接口。上面模板类的偏特化，针对hashing索引的三维模型 */
template<class TVoxel>
class ITMMultiMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMMultiMeshingEngine<TVoxel, ITMVoxelBlockHash> {
 public:
  typedef typename ITMMultiIndex<ITMVoxelBlockHash>::IndexData MultiIndexData;
  typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
  typedef ITMVoxelMapGraphManager<TVoxel, ITMVoxelBlockHash> MultiSceneManager;
  /**
   * @brief 从多个子图中抽取mesh
   * @param[out] mesh
   * @param[in] sceneManager  子图管理器
   */
  void MeshScene(ITMMesh *mesh, const MultiSceneManager &sceneManager);
};
}