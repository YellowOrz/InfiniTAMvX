// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../../../Objects/Meshing/ITMMesh.h"
#include "../../../Objects/Scene/ITMScene.h"

namespace ITMLib {
/** 抽网格的接口（抽象类） */
template<class TVoxel, class TIndex>
class ITMMeshingEngine {
 public:
  /**
   * @brief 从三维场景中抽取mesh
   * @param[out] mesh
   * @param[in] scene  三维场景
   */
  virtual void MeshScene(ITMMesh *mesh, const ITMScene<TVoxel, TIndex> *scene) = 0;

  ITMMeshingEngine(void) {}
  virtual ~ITMMeshingEngine(void) {}
};
}
