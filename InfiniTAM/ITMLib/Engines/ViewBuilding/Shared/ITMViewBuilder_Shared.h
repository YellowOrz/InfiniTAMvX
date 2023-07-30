// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"
#include "../../../../ORUtils/PlatformIndependence.h"

/**
 * @brief 将Kinect的单个像素的视差转成深度
 *        转换公式为 D' = (8*c2*fx)/(c1 - D)
 * @param[out] d_out 真实深度图
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[in] d_in 原始视差图
 * @param[in] disparityCalibParams 深度图转换参数 c1 和 c2。来自相机参数文件的最后一行
 * @param[in] fx_depth depth内参中的fx
 * @param[in] imgSize 图像大小
 */
_CPU_AND_GPU_CODE_ inline void convertDisparityToDepth(DEVICEPTR(float) * d_out, int x, int y,
                                                       const CONSTPTR(short) * d_in, Vector2f disparityCalibParams,
                                                       float fx_depth, Vector2i imgSize) {
  int locId = x + y * imgSize.x;

  short disparity = d_in[locId];
  float disparity_tmp = disparityCalibParams.x - (float) (disparity);
  float depth;

  if (disparity_tmp == 0) depth = 0.0;
  else depth = 8.0f * disparityCalibParams.y * fx_depth / disparity_tmp;

  d_out[locId] = (depth > 0) ? depth : -1.0f;
}

#ifndef __METALC__

/**
 * @brief Affine模式的深度图，将单个像素的深度值从short转float
 *         为线性转换，即 D' = c1*D + c2
 * @param[out] d_out float类型的深度图
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[in] d_in short类型的深度图
 * @param[in] imgSize 图像大小
 * @param[in] depthCalibParams 深度图转换参数 c1 和 c2。来自相机参数文件的最后一行
 */
_CPU_AND_GPU_CODE_ inline void convertDepthAffineToFloat(DEVICEPTR(float) * d_out, int x, int y,
                                                         const CONSTPTR(short) * d_in, Vector2i imgSize,
                                                         Vector2f depthCalibParams) {
  // 获取short类型的深度值
  int locId = x + y * imgSize.x;
  short depth_in = d_in[locId]; 
  // 线性转float类型
  d_out[locId] =
      ((depth_in <= 0) || (depth_in > 32000)) ? -1.0f : (float) depth_in * depthCalibParams.x + depthCalibParams.y;
}

#define MEAN_SIGMA_L 1.2232f
/**
 * @brief 对图像中的单个像素进行双边滤波
 * @param[out] imageData_out 输出图像
 * @param[in] imageData_in 输入图像
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[in] imgDims 图像大小
 */
_CPU_AND_GPU_CODE_ inline void filterDepth(DEVICEPTR(float) * imageData_out, const CONSTPTR(float) * imageData_in,
                                           int x, int y, Vector2i imgDims) {
  float z, tmpz, dz, final_depth = 0.0f, w, w_sum = 0.0f;

  z = imageData_in[x + y * imgDims.x];
  if (z < 0.0f) {
    imageData_out[x + y * imgDims.x] = -1.0f;
    return;
  }
  // 将周围像素（5*5）的值加权到当前像素
  float sigma_z = 1.0f / (0.0012f + 0.0019f * (z - 0.4f) * (z - 0.4f) + 0.0001f / sqrt(z) * 0.25f);

  for (int i = -2, count = 0; i <= 2; i++)
    for (int j = -2; j <= 2; j++, count++) {
      tmpz = imageData_in[(x + j) + (y + i) * imgDims.x];   // 周围像素的深度值
      if (tmpz < 0.0f) continue;
      dz = (tmpz - z);
      dz *= dz;
      w = exp(-0.5f * ((abs(i) + abs(j)) * MEAN_SIGMA_L * MEAN_SIGMA_L +  // 坐标距离
                           dz * sigma_z * sigma_z));                         // 深度距离
      w_sum += w;   // 求和权重
      final_depth += w * tmpz;  // 加权求和深度值
    }

  final_depth /= w_sum;
  imageData_out[x + y * imgDims.x] = final_depth;
}
/**
 * 计算深度图单个像素的法向量和不确定性（权重），用于衡量深度图噪声
 * @param[in] depth_in
 * @param[out] normal_out 最后一维为1
 * @param[out] sigmaZ_out
 * @param[in] x
 * @param[in] y
 * @param[in] imgDims
 * @param[in] intrinparam 相机内参，即fx、fy、cx、cy
 */
_CPU_AND_GPU_CODE_ inline void computeNormalAndWeight(const CONSTPTR(float) * depth_in,
                                                      DEVICEPTR(Vector4f) * normal_out, DEVICEPTR(float) * sigmaZ_out,
                                                      int x, int y, Vector2i imgDims, Vector4f intrinparam) {
  Vector3f outNormal;   // 不能直接存到normal_out[idx]，因为全局内存比寄存器慢很多

  int idx = x + y * imgDims.x;

  float z = depth_in[x + y * imgDims.x];
  if (z < 0.0f) {
    normal_out[idx].w = -1.0f;
    sigmaZ_out[idx] = -1;
    return;
  }

  // first compute the normal
  //! 计算法向量，用有限差分，这个视频的53:30 https://www.bilibili.com/video/BV117411u7D4
  Vector3f xp1_y, xm1_y, x_yp1, x_ym1;
  Vector3f diff_x(0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f);
  // 获取深度值 并 检查有效性
  xp1_y.z = depth_in[(x + 1) + y * imgDims.x], x_yp1.z = depth_in[x + (y + 1) * imgDims.x];
  xm1_y.z = depth_in[(x - 1) + y * imgDims.x], x_ym1.z = depth_in[x + (y - 1) * imgDims.x];
  if (xp1_y.z <= 0 || x_yp1.z <= 0 || xm1_y.z <= 0 || x_ym1.z <= 0) {
    normal_out[idx].w = -1.0f;
    sigmaZ_out[idx] = -1;
    return;
  }

  // unprojected
  // 通过相机内参矩阵反投影到三维空间   FIXME：下面的公式错了吧？？？每一行最后的*都应该换成除号
  xp1_y.x = xp1_y.z * ((x + 1.0f) - intrinparam.z) * intrinparam.x;   // u=fx*X/Z+cx => X=Z*(u-cx)/fx
  xp1_y.y = xp1_y.z * (y - intrinparam.w) * intrinparam.y;            // v=fy*Y/Z+cy => Y=Z*(v-cy)/fy
  xm1_y.x = xm1_y.z * ((x - 1.0f) - intrinparam.z) * intrinparam.x;
  xm1_y.y = xm1_y.z * (y - intrinparam.w) * intrinparam.y;
  x_yp1.x = x_yp1.z * (x - intrinparam.z) * intrinparam.x;
  x_yp1.y = x_yp1.z * ((y + 1.0f) - intrinparam.w) * intrinparam.y;
  x_ym1.x = x_ym1.z * (x - intrinparam.z) * intrinparam.x;
  x_ym1.y = x_ym1.z * ((y - 1.0f) - intrinparam.w) * intrinparam.y;

  // gradients x and y
  // 计算x、y方向的梯度
  diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

  // cross product
  // 叉乘获得法向量 并 检查有效性
  outNormal.x = (diff_x.y * diff_y.z - diff_x.z * diff_y.y);
  outNormal.y = (diff_x.z * diff_y.x - diff_x.x * diff_y.z);
  outNormal.z = (diff_x.x * diff_y.y - diff_x.y * diff_y.x);

  if (outNormal.x == 0.0f && outNormal.y == 0 && outNormal.z == 0) {
    normal_out[idx].w = -1.0f;
    sigmaZ_out[idx] = -1;
    return;
  }
  // 归一化
  float norm = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
  outNormal *= norm;

  normal_out[idx].x = outNormal.x;
  normal_out[idx].y = outNormal.y;
  normal_out[idx].z = outNormal.z;
  normal_out[idx].w = 1.0f;

  // now compute weight
  //! 计算权重
  float theta = acos(outNormal.z);    // 法向量跟当前视线的夹角，只用法向量的z坐标是因为z轴与视线平行???
  float theta_diff = theta / (PI * 0.5f - theta);   // 夹角越大，diff越大。夹角不可能超过90°

  sigmaZ_out[idx] = (0.0012f + 0.0019f * (z - 0.4f) * (z - 0.4f) + 0.0001f / sqrt(z) * theta_diff * theta_diff);
}

#endif
