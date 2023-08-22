// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

namespace ITMLib {
/** \brief
    Represents the calibration information to compute a depth
    image from the disparity image typically received from a
    Kinect.
*/
class ITMDisparityCalib {
  //#################### ENUMERATIONS ####################
 public:
  /** Type of transformation required to get from raw values to depths. */
  enum TrafoType {
    // 深度图的Kinect转换模式，将原始深度图转真实深度图，采用公式 D' = (8*c2*fx)/(c1 - D)，其中c1和c2是相机参数文件中最后一行的两个参数
    // ？？？还没找到相关的介绍资料
    /// Raw values are transformed according to \f$\frac{8c_2f_x}{c_1 - d}\f$
    TRAFO_KINECT,
    // 深度图的线性转换模式，将原始深度图转真实深度图，采用公式 D' = c1*D+c2，
    // c1和c2依次来自相机参数文件中最后一行。c1是最后一行第一个数（表示放大倍数）的倒数，c2是第2个数（表示最小深度值？？？）
    /// Raw values are transformed according to \f$c_1 d + c_2\f$
    TRAFO_AFFINE
  };

  //#################### PRIVATE VARIABLES ####################
 private:
  TrafoType type;     // 深度图转换类型。有两种：KINECT和AFFINE

  /** These are the actual parameters. */
  Vector2f params;    // 原始深度图转真实深度图的两个参数，来自相机参数文件中最后一行。其中第一个参数要取倒数

  //#################### CONSTRUCTORS ####################
 public:
  ITMDisparityCalib(void) {
    SetStandard();
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
 public:
  const Vector2f &GetParams() const {
    return params;
  }

  TrafoType GetType() const {
    return type;
  }

  /** Setup from given arguments. */
  void SetFrom(float a, float b, TrafoType _type) {
    if (a != 0.0f || b != 0.0f) {
      params.x = a;
      params.y = b;
      type = _type;
    } else SetStandard();
  }

  /** 默认使用 AFFINE模式
   * Setup from standard arguments. */
  void SetStandard() {
    // standard calibration parameters - converts mm to metres by dividing by 1000
    SetFrom(1.0f / 1000.0f, 0.0f, TRAFO_AFFINE);
  }
};
}
