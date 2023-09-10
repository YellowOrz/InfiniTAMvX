// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDenseMapper.h"

#include "../Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"
#include "../Engines/Swapping/ITMSwappingEngineFactory.h"
#include "../Objects/RenderStates/ITMRenderState_VH.h"
using namespace ITMLib;

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::ITMDenseMapper(const ITMLibSettings *settings) {
  sceneRecoEngine =
      ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TIndex>(settings->deviceType);
  swappingEngine =
      settings->swappingMode != ITMLibSettings::SWAPPINGMODE_DISABLED ? ITMSwappingEngineFactory::MakeSwappingEngine<
          TVoxel,
          TIndex>(settings->deviceType) : NULL;

  swappingMode = settings->swappingMode;
}

template<class TVoxel, class TIndex>
ITMDenseMapper<TVoxel, TIndex>::~ITMDenseMapper() {
  delete sceneRecoEngine;
  delete swappingEngine;
}

template<class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel, TIndex>::ResetScene(ITMScene<TVoxel, TIndex> *scene) const {
  sceneRecoEngine->ResetScene(scene);
}

template <class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel, TIndex>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState,
                                                  ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState,
                                                  bool resetVisibleList) {
  // 根据深度图 计算visible list、分配内存 && 更新hash table
  sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, false, resetVisibleList);

  // 根据可见列表，将当前输入的单帧融入场景中
  sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);
  // CPU与GPU之间的数据交换
  if (swappingEngine != NULL) {
    // swapping: CPU -> GPU
    if (swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED)
      swappingEngine->IntegrateGlobalIntoLocal(scene, renderState);
      // TODO：这里跟论文好像不太一样。论文Fig.4说的顺序是swap in、raycasting、swap out，这里swap in后面紧跟swap out
    // swapping: GPU -> CPU
    switch (swappingMode) {
      case ITMLibSettings::SWAPPINGMODE_ENABLED: swappingEngine->SaveToGlobalMemory(scene, renderState);   // 进行交换
        break;
      case ITMLibSettings::SWAPPINGMODE_DELETE: swappingEngine->CleanLocalMemory(scene, renderState);      // 删除不可见的
        break;
      case ITMLibSettings::SWAPPINGMODE_DISABLED: break;                                                   // 不进行交换
    }
  }
}

template <class TVoxel, class TIndex>
void ITMDenseMapper<TVoxel, TIndex>::UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState,
                                                       ITMScene<TVoxel, TIndex> *scene, ITMRenderState *renderState,
                                                       bool resetVisibleList) {
  sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, true, resetVisibleList);
}
