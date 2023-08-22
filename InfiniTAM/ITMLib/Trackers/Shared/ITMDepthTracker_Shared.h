// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMPixelUtils.h"
/**
 * 计算point-to-plane ICP转最小二乘问题后的A和b
 * @note A和b的公式见论文《Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration》的(8)和(10)
 * @tparam shortIteration
 * @tparam rotationOnly
 * @param[out] A
 * @param[out] b              距离误差在法向量上的投影
 * @param[in] x               当前像素的x坐标
 * @param[in] y               当前像素的y坐标
 * @param[in] depth           当前像素的深度值
 * @param[in] viewImageSize   当前帧的
 * @param[in] viewIntrinsics
 * @param[in] sceneImageSize  场景的
 * @param[in] sceneIntrinsics 
 * @param[in] approxInvPose   初始位姿，=上一帧的位姿？？？
 * @param[in] scenePose       投影帧位姿？？？
 * @param[in] pointsMap       投影帧出来的三维点？？？
 * @param[in] normalsMap      投影帧对应的法向量
 * @param[in] distThresh      距离阈值，用来剔除误差大的点
 * @return
 */
template <bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth_Ab(
    THREADPTR(float) * A, THREADPTR(float) & b, const THREADPTR(int) & x, const THREADPTR(int) & y,
    const CONSTPTR(float) & depth, const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics,
    const CONSTPTR(Vector2i) & sceneImageSize, const CONSTPTR(Vector4f) & sceneIntrinsics,
    const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose,
    const CONSTPTR(Vector4f) * pointsMap, const CONSTPTR(Vector4f) * normalsMap, float distThresh) {
  if (depth <= 1e-8f) return false;   // 深度太小的跳过     放到computePerPointGH_Depth里面更快？？？

  Vector4f tmp3Dpoint, tmp3Dpoint_reproj;     // CUDA中，先分配会更快吗？
  Vector3f ptDiff;
  Vector4f curr3Dpoint, corr3Dnormal;
  Vector2f tmp2Dpoint;

  //! 计算当前帧中像素的三维坐标
  tmp3Dpoint.x = depth * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);  // 在当前帧下的三维坐标
  tmp3Dpoint.y = depth * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
  tmp3Dpoint.z = depth;
  tmp3Dpoint.w = 1.0f;
  // transform to previous frame coordinates
  tmp3Dpoint = approxInvPose * tmp3Dpoint;  // 变换到上一帧坐标系下
  tmp3Dpoint.w = 1.0f;

  //! 通过重投影找到 投影帧 中的 匹配点
  // project into previous rendered image  
  tmp3Dpoint_reproj = scenePose * tmp3Dpoint;     // 变换到投影帧的坐标系下 ？？？上一帧的坐标投射到之前的渲染图像总区
  if (tmp3Dpoint_reproj.z <= 0.0f) return false;  // 检测深度
  // 投影到scene的成像平面，会是小数
  tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
  tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;
  if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f)
      && (tmp2Dpoint.y <= sceneImageSize.y - 2)))  // 剔除边界附近的点  为什么是2？？？ 
    return false;
  
  curr3Dpoint = interpolateBilinear_withHoles(pointsMap, tmp2Dpoint, sceneImageSize);   // 用双线性插值找对应点
  if (curr3Dpoint.w < 0.0f) return false;

  //! 计算 距离误差 的 平方
  ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
  ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
  ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
  float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z; 

  if (dist > distThresh) return false; 

  //! 计算 b = 误差在法向量上的投影（point-to-plane ICP）
  corr3Dnormal = interpolateBilinear_withHoles(normalsMap, tmp2Dpoint, sceneImageSize);   // 用双线性插值找对应点的法向量
//	if (corr3Dnormal.w < 0.0f) return false;
  b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;

  // TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
  //! 计算 A
  if (shortIteration) {
    if (rotationOnly) {     // = 法向量 与 三维点 的 外积（叉乘），见论文的公式
      A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
      A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
      A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
    } else {                // = 法向量
      A[0] = corr3Dnormal.x; 
      A[1] = corr3Dnormal.y;
      A[2] = corr3Dnormal.z;
    }
  } else {
    A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
    A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
    A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
    A[!shortIteration ? 3 : 0] = corr3Dnormal.x;    // shortIteration肯定为false ？？？
    A[!shortIteration ? 4 : 1] = corr3Dnormal.y;
    A[!shortIteration ? 5 : 2] = corr3Dnormal.z;
  }

  return true;
}

/**
 * 计算单个像素的Hessian矩阵、Nabla算子 和 误差
 * @tparam shortIteration     只求解 旋转 or 平移
 * @tparam rotationOnly       
 * @param[out] localNabla     nabla算子
 * @param[out] localHessian   H矩阵
 * @param[out] localF         误差
 * @param[in] x               像素的x坐标
 * @param[in] y
 * @param[in] depth
 * @param[in] viewImageSize   当前帧的
 * @param[in] viewIntrinsics
 * @param[in] sceneImageSize  场景的
 * @param[in] sceneIntrinsics 
 * @param[in] approxInvPose   初始位姿，=上一帧的位姿？？？
 * @param[in] scenePose       场景位姿？？？
 * @param[in] pointsMap       场景投影出来的三维点？？？
 * @param[in] normalsMap      投影点对应的法向量
 * @param[in] distThresh      距离阈值，用来剔除误差大的点
 * @return                    是否计算成功
 * @note  虽然是_CPU_AND_GPU_CODE_，但是只在CPU中用到了，CUDA中没用
 */
template <bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool
computePerPointGH_Depth(THREADPTR(float) * localNabla, THREADPTR(float) * localHessian, THREADPTR(float) & localF,
                        const THREADPTR(int) & x, const THREADPTR(int) & y, const CONSTPTR(float) & depth,
                        const CONSTPTR(Vector2i) & viewImageSize, const CONSTPTR(Vector4f) & viewIntrinsics,
                        const CONSTPTR(Vector2i) & sceneImageSize, const CONSTPTR(Vector4f) & sceneIntrinsics,
                        const CONSTPTR(Matrix4f) & approxInvPose, const CONSTPTR(Matrix4f) & scenePose,
                        const CONSTPTR(Vector4f) * pointsMap, const CONSTPTR(Vector4f) * normalsMap, float distThresh) {
  //! 转最小二乘问题Ax=b
  const int noPara = shortIteration ? 3 : 6;  // 只计算旋转or平移，维度为3，否则为6
  float A[noPara], b;

  bool ret = computePerPointGH_Depth_Ab<shortIteration, rotationOnly>(A, b, x, y, depth, viewImageSize, viewIntrinsics,
                                                                      sceneImageSize, sceneIntrinsics, approxInvPose,
                                                                      scenePose, pointsMap, normalsMap, distThresh);
  if (!ret) return false;

  //! 计算当前像素的 Hessian矩阵、Nabla算子 和 误差
  localF = b * b;                             //误差
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))  // CUDA，提示编译器展开循环
#pragma unroll
#endif
  for (int r = 0, counter = 0; r < noPara; r++) {
    localNabla[r] = b * A[r];                 // nabla算子
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
    for (int c = 0; c <= r; c++, counter++)   // Hessian矩阵
      localHessian[counter] = A[r] * A[c];
  }

  return true;
}
