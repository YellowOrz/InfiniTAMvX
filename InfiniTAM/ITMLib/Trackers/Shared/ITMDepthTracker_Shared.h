// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMPixelUtils.h"

template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth_Ab(THREADPTR(float) *A,
                                                          THREADPTR(float) &b,
                                                          const THREADPTR(int) &x,
                                                          const THREADPTR(int) &y,
                                                          const CONSTPTR(float) &depth,
                                                          const CONSTPTR(Vector2i) &viewImageSize,
                                                          const CONSTPTR(Vector4f) &viewIntrinsics,
                                                          const CONSTPTR(Vector2i) &sceneImageSize,
                                                          const CONSTPTR(Vector4f) &sceneIntrinsics,
                                                          const CONSTPTR(Matrix4f) &approxInvPose,
                                                          const CONSTPTR(Matrix4f) &scenePose,
                                                          const CONSTPTR(Vector4f) *pointsMap,
                                                          const CONSTPTR(Vector4f) *normalsMap,
                                                          float distThresh) {
  if (depth <= 1e-8f) return false; //check if valid -- != 0.0f  实际的深度图会有飞点等干扰因素

  Vector4f tmp3Dpoint, tmp3Dpoint_reproj;
  Vector3f ptDiff;
  Vector4f curr3Dpoint, corr3Dnormal;
  Vector2f tmp2Dpoint;

  //当前帧三维坐标
  tmp3Dpoint.x = depth * ((float(x) - viewIntrinsics.z) / viewIntrinsics.x);
  tmp3Dpoint.y = depth * ((float(y) - viewIntrinsics.w) / viewIntrinsics.y);
  tmp3Dpoint.z = depth;
  tmp3Dpoint.w = 1.0f;

  // transform to previous frame coordinates  变换到上一帧坐标
  tmp3Dpoint = approxInvPose * tmp3Dpoint;
  tmp3Dpoint.w = 1.0f;

  // project into previous rendered image  上一帧的坐标投射到之前的渲染图像总区
  tmp3Dpoint_reproj = scenePose * tmp3Dpoint;
  if (tmp3Dpoint_reproj.z <= 0.0f) return false;  // 用z坐标来判断？
  //将当前帧投影到相机坐标
  tmp2Dpoint.x = sceneIntrinsics.x * tmp3Dpoint_reproj.x / tmp3Dpoint_reproj.z + sceneIntrinsics.z;
  tmp2Dpoint.y = sceneIntrinsics.y * tmp3Dpoint_reproj.y / tmp3Dpoint_reproj.z + sceneIntrinsics.w;

  //剔除超过边界的点
  if (!((tmp2Dpoint.x >= 0.0f) && (tmp2Dpoint.x <= sceneImageSize.x - 2) && (tmp2Dpoint.y >= 0.0f)
      && (tmp2Dpoint.y <= sceneImageSize.y - 2)))  // 判断是否在这个点是否在一个范围内 （0,0）到sceneImageSize-2之间
    return false;
    /* 线性插值 https://www.cnblogs.com/xpvincent/archive/2013/03/15/2961448.html
     * 邻近插值 https://theailearner.com/2018/12/29/image-processing-nearest-neighbour-interpolation/
     * 双线性插值 https://theailearner.com/2018/12/29/image-processing-bilinear-interpolation/
     */
  //用双线性插值法得到相机坐标系下三维坐标点
  curr3Dpoint = interpolateBilinear_withHoles(pointsMap, tmp2Dpoint, sceneImageSize);
  if (curr3Dpoint.w < 0.0f) return false;

  //当前点的法向量
  ptDiff.x = curr3Dpoint.x - tmp3Dpoint.x;
  ptDiff.y = curr3Dpoint.y - tmp3Dpoint.y;
  ptDiff.z = curr3Dpoint.z - tmp3Dpoint.z;
  float dist = ptDiff.x * ptDiff.x + ptDiff.y * ptDiff.y + ptDiff.z * ptDiff.z; //两个点之间的距离

  if (dist > distThresh) return false;//剔除误差较大的点 // 如果没有达到阈值 无法收敛

  //原模型中得到的坐标点
  corr3Dnormal = interpolateBilinear_withHoles(normalsMap, tmp2Dpoint, sceneImageSize);
//	if (corr3Dnormal.w < 0.0f) return false;

  //点到面的ICP方法，计算当前位姿的误差函数
  b = corr3Dnormal.x * ptDiff.x + corr3Dnormal.y * ptDiff.y + corr3Dnormal.z * ptDiff.z;
  // b为 point-to-plane的误差函数
  // TODO check whether normal matches normal from image, done in the original paper, but does not seem to be required
  // 检查 normal 是否与图像中的 normal 匹配，在原始论文中完成，但似乎不需要
  if (shortIteration) {
    if (rotationOnly) {
      //TODO（h）：下列的操作是？
      A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
      A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
      A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
    } else {
      A[0] = corr3Dnormal.x;
      A[1] = corr3Dnormal.y;
      A[2] = corr3Dnormal.z;
    }
  } else {
    A[0] = +tmp3Dpoint.z * corr3Dnormal.y - tmp3Dpoint.y * corr3Dnormal.z;
    A[1] = -tmp3Dpoint.z * corr3Dnormal.x + tmp3Dpoint.x * corr3Dnormal.z;
    A[2] = +tmp3Dpoint.y * corr3Dnormal.x - tmp3Dpoint.x * corr3Dnormal.y;
    //误差函数对当前求得的法向量的偏导数就是模型点的法向量的本身
    A[!shortIteration ? 3 : 0] = corr3Dnormal.x;
    A[!shortIteration ? 4 : 1] = corr3Dnormal.y;
    A[!shortIteration ? 5 : 2] = corr3Dnormal.z;
  }

  return true;
}

template<bool shortIteration, bool rotationOnly>
_CPU_AND_GPU_CODE_ inline bool computePerPointGH_Depth(THREADPTR(float) *localNabla,
                                                       THREADPTR(float) *localHessian,
                                                       THREADPTR(float) &localF,
                                                       const THREADPTR(int) &x,
                                                       const THREADPTR(int) &y,
                                                       const CONSTPTR(float) &depth,
                                                       const CONSTPTR(Vector2i) &viewImageSize,
                                                       const CONSTPTR(Vector4f) &viewIntrinsics,
                                                       const CONSTPTR(Vector2i) &sceneImageSize,
                                                       const CONSTPTR(Vector4f) &sceneIntrinsics,
                                                       const CONSTPTR(Matrix4f) &approxInvPose,
                                                       const CONSTPTR(Matrix4f) &scenePose,
                                                       const CONSTPTR(Vector4f) *pointsMap,
                                                       const CONSTPTR(Vector4f) *normalsMap,
                                                       float distThresh) {
  const int noPara = shortIteration ? 3 : 6;
  float A[noPara];
  float b;

  bool ret = computePerPointGH_Depth_Ab<shortIteration, rotationOnly>(A,
                                                                      b,
                                                                      x,
                                                                      y,
                                                                      depth,
                                                                      viewImageSize,
                                                                      viewIntrinsics,
                                                                      sceneImageSize,
                                                                      sceneIntrinsics,
                                                                      approxInvPose,
                                                                      scenePose,
                                                                      pointsMap,
                                                                      normalsMap,
                                                                      distThresh);

  if (!ret) return false;

  localF = b * b;//当前帧误差函数

#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
  for (int r = 0, counter = 0; r < noPara; r++) {
    //当前帧雅可比矩阵
    localNabla[r] = b * A[r];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
    //当前帧海塞矩阵
    for (int c = 0; c <= r; c++, counter++) localHessian[counter] = A[r] * A[c];
  }

  return true;
}

