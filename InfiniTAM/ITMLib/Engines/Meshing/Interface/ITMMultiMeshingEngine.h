// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMeshingEngine.h"
#include "../../../Objects/Meshing/ITMMesh.h"
#include "../../MultiScene/ITMMapGraphManager.h"

namespace ITMLib {
/** 多子图的抽网格的接口（抽象类） */
template<class TVoxel, class TIndex>
class ITMMultiMeshingEngine {
 public:
  virtual ~ITMMultiMeshingEngine(void) {}
  /**
   * @brief 从多个子图中抽取mesh
   * @param[out] mesh
   * @param[in] sceneManager  子图管理器
   */
  virtual void MeshScene(ITMMesh *mesh, const ITMVoxelMapGraphManager<TVoxel, TIndex> &sceneManager) = 0;
};
}