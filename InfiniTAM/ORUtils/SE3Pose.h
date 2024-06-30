// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Vector.h"
#include "Matrix.h"

namespace ORUtils {
/** \brief 相机位姿（旋转+平移）
    Represents a camera pose with rotation and translation parameters
*/
class SE3Pose {
 private:
  /** 相机位姿的SE3形式（6x1），旋转用SO3表示
   * This is the minimal representation of the pose with six parameters. The three rotation parameters are the Lie algebra representation of SO3.
  */
  union {
    float all[6];
    struct {
      float tx, ty, tz;
      float rx, ry, rz;
    } each;
  } params;

  /** 相机位姿的矩阵形式（4x4）
   * The pose as a 4x4 transformation matrix ("modelview matrix).
  */
  Matrix4<float> M;

  /** 相机位姿从 矩阵形式（4x4） 转 SE3形式（6x1）
   * This will update the minimal parameterisation from the current modelview matrix.
   */
  void SetParamsFromModelView();

  /** 相机位姿从 SE3形式（6x1） 转 矩阵形式（4x4） 
   * This will update the "modelview matrix" M from the minimal representation.
  */
  void SetModelViewFromParams();
 public:
  /** 记录相机位姿的矩阵形式（4x4）和SE3形式（6x1），但不会检查二者是否表示同一位姿 */
  void SetBoth(const Matrix4<float> &M, const float params[6]);
  /** 根据SE3形式（6x1）记录相机位姿 */
  void SetFrom(float tx, float ty, float tz, float rx, float ry, float rz);
  /** 根据SE3形式（6x1）记录相机位姿 */
  void SetFrom(const Vector3<float> &translation, const Vector3<float> &rotation);
  /** 根据SE3形式（6x1）记录相机位姿 */
  void SetFrom(const Vector6<float> &tangent);
  /** 根据SE3形式（6x1）记录相机位姿 */
  void SetFrom(const float pose[6]);
  /** 根据其他相机位姿记录相机位姿 */
  void SetFrom(const SE3Pose *pose);
  /** 左乘其他相机位姿。This will multiply a pose @p pose on the right, i.e. this = this * pose. */
  void MultiplyWith(const SE3Pose *pose);
  /** 获取矩阵形式的相机位姿 */
  const Matrix4<float> &GetM(void) const { return M; }
  /** 获取矩阵形式的旋转 */
  Matrix3<float> GetR(void) const;
  /** 获取平移 */
  Vector3<float> GetT(void) const;
  /** 获取SE3形式（6x1）的相机位姿 */
  void GetParams(Vector3<float> &translation, Vector3<float> &rotation) const;
  /** 获取SE3形式（6x1）的相机位姿 */
  const float *GetParams(void) const { return params.all; }
  /** 根据矩阵形式记录相机位姿 */
  void SetM(const Matrix4<float> &M);
  /** 根据矩阵形式记录相机旋转 */
  void SetR(const Matrix3<float> &R);
  /** 记录平移 */
  void SetT(const Vector3<float> &t);
  /** 根据矩阵形式记录相机位姿 */
  void SetRT(const Matrix3<float> &R, const Vector3<float> &t);
  /** 相机位姿的矩阵形式的逆 */
  Matrix4<float> GetInvM(void) const;
  /** 将输入矩阵取逆后作为位姿矩阵 */
  void SetInvM(const Matrix4<float> &invM);
  /** 保证旋转矩阵的正交性。建议在计算 矩阵形式的位姿 之后调用。
   * This will enforce the orthonormality constraints on the rotation matrix. It's recommended to call this function 
   * after manipulating the matrix M.
  */
  void Coerce(void);
  /** 各种构造函数 */
  SE3Pose(const SE3Pose &src);
  SE3Pose(const Matrix4<float> &src);
  SE3Pose(float tx, float ty, float tz, float rx, float ry, float rz);
  SE3Pose(const Vector6<float> &tangent);
  SE3Pose(const Matrix3<float> &R, const Vector3<float> &t);
  explicit SE3Pose(const float pose[6]);
  SE3Pose(void);

  friend std::ostream &operator<<(std::ostream &os, const SE3Pose &dt) {
    os << dt.M;
    return os;
  }

  /** 好像没有地方用到。This builds a Pose based on its exp representation */
  static SE3Pose exp(const Vector6<float> &tangent);
};
}

