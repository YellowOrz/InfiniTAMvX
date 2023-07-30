// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Interface/ITMViewBuilder.h"
#include "../../Utils/ITMLibSettings.h"

namespace ITMLib {

/**
 * \brief This struct provides functions that can be used to construct view builders.
 */
struct ITMViewBuilderFactory {
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief 根据 设备类型 构建 图像预处理模块
   * Makes a view builder.
   *
   * \param[in] calib       The joint RGBD calibration parameters.RGBD相机参数
   * \param[in] deviceType  The device on which the view builder should operate.设备类型：CPU or CUDA or Apple
   * @return ITMViewBuilder* view builder，相当于是输入图像的预处理？？？
   */
  static ITMViewBuilder *MakeViewBuilder(const ITMRGBDCalib &calib, ITMLibSettings::DeviceType deviceType);
};

}
