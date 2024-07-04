// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "GraphEdgeSE3.h"
#include "GraphNodeSE3.h"

#include "QuaternionHelpers.h"

#include <stdio.h>

using namespace MiniSlamGraph;

/**
 * @brief 将相机位姿从矩阵形式转换为MQT形式
 * @param[in] m     矩阵形式
 * @param[out] qt   MQT形式（旋转为四元数，平移为向量）
 */
static void MatrixToMQT(const ORUtils::Matrix4<float> &m, double *qt) {
  double R[9];
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) R[r * 3 + c] = m.m[c * 4 + r];
  double qtmp[4];
  QuaternionHelpers::QuaternionFromRotationMatrix(R, qtmp);

  for (int i = 0; i < 3; ++i) qt[i] = qtmp[i + 1];
  for (int i = 0; i < 3; ++i) qt[3 + i] = m.m[3 * 4 + i];
}
/**
 * @brief 将相机位姿从MQT形式转换为矩阵形式
 * @param[in] qt    MQT形式（旋转为四元数，平移为向量）
 * @param[out] m    矩阵形式
 */
static void MQTToMatrix(const double *qt, ORUtils::Matrix4<float> &m) {
  double qtmp[4];
  for (int i = 0; i < 3; ++i) qtmp[i + 1] = qt[i];
  qtmp[0] = sqrt(1.0f - qt[0] * qt[0] - qt[1] * qt[1] - qt[2] * qt[2]);

  double R[9];
  QuaternionHelpers::RotationMatrixFromQuaternion(qtmp, R);
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m.m[c * 4 + r] = (float) R[r * 3 + c];
  for (int i = 0; i < 3; ++i) m.m[3 * 4 + i] = (float) qt[3 + i];
  m.m[0 * 4 + 3] = m.m[1 * 4 + 3] = m.m[2 * 4 + 3] = 0.0f;
  m.m[3 * 4 + 3] = 1.0f;
}
/**
 * @brief
 * @param idx
 * @return
 */
static ORUtils::Matrix4<float> se3_generator(int idx) {
  ORUtils::Matrix4<float> ret;
  ret.setZeros();
  if (idx < 3) {
    ret.m[3 * 4 + idx] = 1.0f;
  } else {
    int r = (idx + 1) % 3;
    int c = (idx + 2) % 3;
    ret.m[c * 4 + r] = -1.0f;
    ret.m[r * 4 + c] = 1.0f;
  }
  return ret;
}
/** 设置 测量位姿。一般来说是多次观测的加权平均？ */
void GraphEdgeSE3::setMeasurementSE3(const SE3 &pose) {
  MatrixToMQT(pose.GetM(), mMeasuredPose);
}

GraphEdgeSE3::SE3 GraphEdgeSE3::getMeasurementSE3(void) const {
  ORUtils::Matrix4<float> m;
  MQTToMatrix(mMeasuredPose, m);
  return SE3(m);
}

void GraphEdgeSE3::computeResidualVector(const GraphEdgeSE3::NodeIndex &nodes, double *dest) const {
  // 得到from和to两个节点的位姿。get poses of "from" and "to" nodes
  const GraphNodeSE3 *fromNode = (const GraphNodeSE3 *) nodes.find(fromNodeId())->second; // T_fw
  const GraphNodeSE3 *toNode = (const GraphNodeSE3 *) nodes.find(toNodeId())->second;     // T_tw
  const SE3 &fromPose = fromNode->getPose();
  const SE3 &toPose = toNode->getPose();

  // 将测量位姿转成矩阵形式。get measured pose as a matrix
  ORUtils::Matrix4<float> m;
  MQTToMatrix(mMeasuredPose, m);  // T_m 约等于 T_tf，因为存在误差

  // 计算from和to到测量位姿的误差？？？compute residual
  ORUtils::Matrix4<float> residualPose(fromPose.GetM() * toPose.GetInvM() * m); // 解释见下面
  MatrixToMQT(residualPose, dest);
  // NOTE: 如果测量绝对精准的话，T_m = T_tw * (T_fw)^-1 = T_tw * T_wf = T_tf（类似视觉SLAM十四讲的公式10.3）
  // 但是存在误差导致等式无法成立，将右边的挪过去，得到误差为 = T_fw * (T_tw)^-1 * T_tf（视觉SLAM十四讲的公式10.4是将左边的挪过去，都一样）
}

bool GraphEdgeSE3::computeJacobian(const NodeIndex &nodes, int id, double *jacobian) const {
  const GraphNodeSE3 *node_f = (const GraphNodeSE3 *) nodes.find(fromNodeId())->second;
  const GraphNodeSE3 *node_t = (const GraphNodeSE3 *) nodes.find(toNodeId())->second;
  const SE3 &fromPose = node_f->getPose();
  const SE3 &toPose = node_t->getPose();

  // get measured pose as a matrix
  ORUtils::Matrix4<float> m;
  MQTToMatrix(mMeasuredPose, m);

  //compute residual
  ORUtils::Matrix4<float> AB(fromPose.GetM() * toPose.GetInvM());

  ORUtils::Matrix4<float> dAB_dx[6];
  if (id == node_f->getId()) {
    for (int i = 0; i < 6; ++i) dAB_dx[i] = se3_generator(i) * AB;
  } else if (id == node_t->getId()) {
    for (int i = 0; i < 6; ++i) dAB_dx[i] = AB * se3_generator(i) * -1.0f;
  } else return false;

  double dQ_dR[4 * 9];
  {
    ORUtils::Matrix4<float> ABm = AB * m;
    double ABm_array[9];
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ABm_array[r * 3 + c] = ABm.m[c * 4 + r];
    QuaternionHelpers::dQuaternion_dRotationMatrix(ABm_array, dQ_dR);
  }
  for (int gi = 0; gi < 6; ++gi) {
    ORUtils::Matrix4<float> d_inner_dx = dAB_dx[gi] * m;
    for (int qi = 0; qi < 3; ++qi) {
      jacobian[qi * 6 + gi] = 0.0f;
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          jacobian[qi * 6 + gi] += dQ_dR[(qi + 1) * 9 + (r * 3 + c)]
              * d_inner_dx.m[c * 4 + r];
    }
    for (int ti = 0; ti < 3; ++ti) {
      jacobian[(ti + 3) * 6 + gi] = d_inner_dx.m[3 * 4 + ti];
    }
  }
  return true;
}