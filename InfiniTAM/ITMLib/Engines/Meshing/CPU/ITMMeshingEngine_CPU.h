// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib {
/** CPU版的抽网格接口 */
template<class TVoxel, class TIndex>
class ITMMeshingEngine_CPU : public ITMMeshingEngine<TVoxel, TIndex> {
  /**
   * @brief 从三维场景中抽取mesh
   * @param[out] mesh
   * @param[in] scene  三维场景
   */
  void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, TIndex> *scene) {}
};
/** CPU版的抽网格接口。上面模板类的偏特化，针对hashing索引的三维模型 */
template<class TVoxel>
class ITMMeshingEngine_CPU<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine<TVoxel, ITMVoxelBlockHash> {
 public:
  /**
   * @brief 从三维场景中抽取mesh
   * @param[out] mesh
   * @param[in] scene  三维场景
   */
  void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

  ITMMeshingEngine_CPU(void) {}
  ~ITMMeshingEngine_CPU(void) {}
};
}
// TODO: 没有针对ITMPlainVoxelArray的偏特化
