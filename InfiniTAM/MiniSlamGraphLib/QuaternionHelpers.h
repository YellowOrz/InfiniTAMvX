// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace MiniSlamGraph {
class QuaternionHelpers {
 public:
  static void RotationMatrixFromQuaternion(const double *q, double *matrix);

  /**
   * @brief 将旋转从矩阵形式转换为四元数
   * @details Read a row-major 3x3 rotation matrix from @p matrix , compute the corresponding quaternion and store it in
   * the 4-vector @p q. The first element of @p q will be real part, followed by the three imaginary parts.
   * @param[in] matrix  矩阵形式，3*3，按行展开
   * @param[out] q      四元数
   * @note 来源"James Diebel. Representing Attitude: Euler Angles, Quaternions, and Rotation Vectors. Technical Report,
   * Stanford University, Palo Alto, CA."
   */
  static void QuaternionFromRotationMatrix(const double *matrix, double *q);

  /**
   * @brief 将旋转矩阵转四元数后，计算四元数对旋转矩阵的偏导？
   * @details Compute the derivative of the transformation in QuaternionFromRotationMatrix() w.r.t. the elements of the
   * rotation matrix @p matrix and write them to @p dq_dR. The first 9 elements of @p dq_dR will be the derivatives of
   * the real part of the quaternion w.r.t. R11, R12, ..., followed by three similar rows for the imaginary parts.
   * @param[in] matrix  矩阵形式，3*3，按行展开
   * @param dq_dR       ，4*9
   * @note 来源"James Diebel. Representing Attitude: Euler Angles, Quaternions, and Rotation Vectors. Technical Report,
   * Stanford University, Palo Alto, CA."
   *       dq_dR的第0行的9个元素是四元数实部的导数，即r11、r22、r33
   */
  static void dQuaternion_dRotationMatrix(const double *matrix, double *dq_dR);
};
}

