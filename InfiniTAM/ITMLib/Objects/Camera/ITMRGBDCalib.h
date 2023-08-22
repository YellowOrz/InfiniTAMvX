// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMIntrinsics.h"
#include "ITMExtrinsics.h"
#include "ITMDisparityCalib.h"

namespace ITMLib {
/** \brief
    Represents the joint RGBD calibration parameters
    RGBD相机内参，包含rgb、depth的内参、它们之间的外参数 和 深度图转换参数
*/
class ITMRGBDCalib {
 public:
  /// Intrinsic parameters of the RGB camera.
  ITMIntrinsics intrinsics_rgb;

  /// Intrinsic parameters of the depth camera.
  ITMIntrinsics intrinsics_d;

  /** @brief
   *  RGB到depth的外参
      Extrinsic calibration between RGB and depth
      cameras.

      This transformation takes points from the RGB
      camera coordinate system to the depth camera
      coordinate system.
  */
  ITMExtrinsics trafo_rgb_to_depth;

  // 深度图转换的参数（包含2个）。来自相机参数文件的最后一行
  /// Calibration information to compute depth from disparity images.
  ITMDisparityCalib disparityCalib;
};
}
