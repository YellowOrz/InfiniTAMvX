// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "GraphEdge.h"

#include "../ORUtils/SE3Pose.h"

namespace MiniSlamGraph {
/** 位姿图中类型为SE3的边 */
class GraphEdgeSE3 : public GraphEdge {
 public:
  typedef ORUtils::SE3Pose SE3;
  /** 测量误差的维度 */
  int getMeasureDimensions(void) const {
    return 6;
  }
  void setMeasurement(const double *v) {
    for (int i = 0; i < 6; ++i) mMeasuredPose[i] = v[i];
  }
  void getMeasurement(double *v) const {
    for (int i = 0; i < 6; ++i) v[i] = mMeasuredPose[i];
  }

  void setMeasurementSE3(const SE3 &pose);
  SE3 getMeasurementSE3(void) const;
  /**
   * @brief 计算 边/约束 的误差向量
   * @param[in] nodes   所有的节点。会根据成员变量 @p idFrom 和 @p idTo 找到相关的节点。
   * @param[out] dest   误差矩阵的MQT形式（四元数+平移向量）
   */
  void computeResidualVector(const NodeIndex &nodes, double *dest) const;
  bool computeJacobian(const NodeIndex &nodes, int id, double *j) const;

 private:
  double mMeasuredPose[6];  // 观测位姿（多次加权平均过的），从from到to，即T_tf。MQT形式（四元数+平移向量）// TODO: 为啥不弄成Vector6?这样可以get的时候引用
};
}