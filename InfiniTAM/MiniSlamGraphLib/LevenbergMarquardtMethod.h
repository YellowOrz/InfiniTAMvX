// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "SlamGraphErrorFunction.h"

namespace MiniSlamGraph {
/** 莱文伯格-马夸特（LM）优化方法 */
class LevenbergMarquardtMethod {
 public:
  /** 
    * @brief 最小化误差函数
    * @param[in] function       误差函数
    * @param[in] initialization 优化参数，其实就是位姿图的节点
    * @return int
    */
  static int minimize(const SlamGraphErrorFunction &function, SlamGraphErrorFunction::Parameters &initialization);
};
}

