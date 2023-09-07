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
  // 配置（给定一个具有新深度图像的视图，计算可见块，分配它们并更新哈希表，以便可以集成新的图像数据。）
  sceneRecoEngine->AllocateSceneFromDepth(scene, view, trackingState, renderState, false, resetVisibleList);

  // 集成（通过整合来自给定视图的深度和可能的颜色信息来更新体素块。）
  sceneRecoEngine->IntegrateIntoScene(scene, view, trackingState, renderState);
  // CPU与GPU之间的内存交换   // TODO： 下次从这儿开始看
  if (swappingEngine != NULL) { // 判断交换引擎接口地址是否为空
    // swapping: CPU -> GPU
    if (swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED)   // 判断交换模式
      swappingEngine->IntegrateGlobalIntoLocal(scene,
                                               renderState);    // 进行交换

    // swapping: GPU -> CPU
    switch (swappingMode) { //判断交换模式
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
