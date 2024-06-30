// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib {
/** GPU版的抽网格接口（抽象类） */
template<class TVoxel, class TIndex>
class ITMMeshingEngine_CUDA : public ITMMeshingEngine<TVoxel, TIndex> {};

/** GPU版的抽网格接口。上面模板类的偏特化，针对hashing索引的三维模型 */
template<class TVoxel>
class ITMMeshingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine<TVoxel, ITMVoxelBlockHash> {
 private:
  unsigned int *noTriangles_device;
  Vector4s *visibleBlockGlobalPos_device;

 public:
  /**
   * @brief 从三维场景中抽取mesh
   * @param[out] mesh
   * @param[in] scene  三维场景
   */
  void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

  ITMMeshingEngine_CUDA(void);
  ~ITMMeshingEngine_CUDA(void);
};

/** GPU版的抽网格接口。上面模板类的偏特化，针对下标索引的三维模型。但其实没有实现 */
template <class TVoxel>
class ITMMeshingEngine_CUDA<TVoxel, ITMPlainVoxelArray> : public ITMMeshingEngine<TVoxel, ITMPlainVoxelArray> {
 public:
  /**
   * @brief 从三维场景中抽取mesh
   * @param[out] mesh
   * @param[in] scene  三维场景
   */
  void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

  ITMMeshingEngine_CUDA(void);
  ~ITMMeshingEngine_CUDA(void);
};
}
