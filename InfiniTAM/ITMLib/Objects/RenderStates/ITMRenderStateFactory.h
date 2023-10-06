// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMSceneParams.h"
#include "ITMRenderState_VH.h"

namespace ITMLib {
template <typename TIndex> struct ITMRenderStateFactory {
  /** Creates a render state, containing rendering info for the scene. */
  static ITMRenderState *CreateRenderState(const Vector2i &imgSize, const ITMSceneParams *sceneParams,
                                           MemoryDeviceType memoryType) {
    return new ITMRenderState(imgSize, sceneParams->viewFrustum_min, sceneParams->viewFrustum_max, memoryType);
  }
};

/** 上面模板类的偏特化，针对hashing索引的三维模型 */
template <> struct ITMRenderStateFactory<ITMVoxelBlockHash> {
  /** Creates a render state, containing rendering info for the scene. */
  static ITMRenderState *CreateRenderState(const Vector2i &imgSize, const ITMSceneParams *sceneParams,
                                           MemoryDeviceType memoryType) {
    return new ITMRenderState_VH(ITMVoxelBlockHash::noTotalEntries, imgSize, sceneParams->viewFrustum_min,
                                 sceneParams->viewFrustum_max, memoryType);
  }
};
} // namespace ITMLib
