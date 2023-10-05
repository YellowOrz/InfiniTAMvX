// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMeshingEngine.h"
#include "../../../Objects/Scene/ITMPlainVoxelArray.h"

namespace ITMLib {
template<class TVoxel, class TIndex>
class ITMMeshingEngine_CUDA : public ITMMeshingEngine<TVoxel, TIndex> {};
/** 上面模板类的偏特化，针对hashing索引的三维模型 */
template<class TVoxel>
class ITMMeshingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMMeshingEngine<TVoxel, ITMVoxelBlockHash> {
 private:
  unsigned int *noTriangles_device;
  Vector4s *visibleBlockGlobalPos_device;

 public:
  void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene);

  ITMMeshingEngine_CUDA(void);
  ~ITMMeshingEngine_CUDA(void);
};
/** 上面模板类的偏特化，针对下标索引的三维模型。但其实没有实现 */
template<class TVoxel>
class ITMMeshingEngine_CUDA<TVoxel, ITMPlainVoxelArray> : public ITMMeshingEngine<TVoxel, ITMPlainVoxelArray> {
 public:
  void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene);

  ITMMeshingEngine_CUDA(void);
  ~ITMMeshingEngine_CUDA(void);
};
}
