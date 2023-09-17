// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/ITMSwappingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSwappingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/ITMSwappingEngine_Metal.h"
#endif

namespace ITMLib {

/**
 * @brief 初始化Swap模块
 */
struct ITMSwappingEngineFactory {
  // #################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * @brief 根据设备类型 初始化Swap模块
   *
   * \param deviceType  The device on which the swapping engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static ITMSwappingEngine<TVoxel, TIndex> *MakeSwappingEngine(ITMLibSettings::DeviceType deviceType) {
    ITMSwappingEngine<TVoxel, TIndex> *swappingEngine = NULL;

    switch (deviceType) {
    case ITMLibSettings::DEVICE_CPU:
      swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>;
      break;
    case ITMLibSettings::DEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
      swappingEngine = new ITMSwappingEngine_CUDA<TVoxel, TIndex>;
#endif
      break;
    case ITMLibSettings::DEVICE_METAL:
#ifdef COMPILE_WITH_METAL
      swappingEngine = new ITMSwappingEngine_CPU<TVoxel, TIndex>;
#endif
      break;
    }

    return swappingEngine;
  }
};

} // namespace ITMLib
