// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMDepthTracker.h"

namespace ITMLib {
class ITMDepthTracker_CPU : public ITMDepthTracker {
 protected:
  /**
   * @brief 计算point-to-plane ICP的Hessian矩阵、Nabla算子 和 误差
   * 
   * @param[out] f            误差
   * @param[out] nabla        Nabla算子
   * @param[out] hessian      Hessian矩阵
   * @param[in] approxInvPose 初始位姿，=上一帧的位姿
   * @return int              有效像素数
  */
  int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose);

 public:
   ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                       float terminationThreshold, float failureDetectorThreshold,
                       const ITMLowLevelEngine *lowLevelEngine);
   ~ITMDepthTracker_CPU(void);
};
}
