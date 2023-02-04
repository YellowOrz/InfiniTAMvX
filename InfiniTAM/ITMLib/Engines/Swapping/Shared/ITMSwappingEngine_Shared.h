// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"
/**
 * @tparam TVoxel 体素信息
 * @param src 上一帧的体素信息
 * @param dst 设备上新一帧的体素信息
 * @param maxW 整体空间的最大尺度
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelDepthInformation(const CONSTPTR(TVoxel) &src,
                                                            DEVICEPTR(TVoxel) &dst,
                                                            int maxW) {
  int newW = dst.w_depth;   // 新一帧的相机平面深度
  int oldW = src.w_depth;   // 上一帧的相机平面深度
  float newF = TVoxel::valueToFloat(dst.sdf);   // 新一帧体素与物体的最近距离
  float oldF = TVoxel::valueToFloat(src.sdf);   // 上一帧体素与物体的最近距离

  if (oldW == 0) return; //若新增深度为0直接返回
  //新增深度不为0，则进行优化
  newF = oldW * oldF + newW * newF;
  newW = oldW + newW;   // 计算新一帧相机的平面距离
  newF /= newW; // 优化后新一帧的体素与物体的最近距离
  newW = MIN(newW, maxW);   // 取最小值作为新一帧相机平面深度

  dst.w_depth = newW;   // 新一帧相机的平面距离
  dst.sdf = TVoxel::floatToValue(newF); // 优化后新一帧的体素与物体的最近距离
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelColorInformation(const CONSTPTR(TVoxel) &src,
                                                            DEVICEPTR(TVoxel) &dst,
                                                            int maxW) {
  int newW = dst.w_color;
  int oldW = src.w_color;
  Vector3f newC = dst.clr.toFloat() / 255.0f;
  Vector3f oldC = src.clr.toFloat() / 255.0f;

  if (oldW == 0) return;

  newC = oldC * (float) oldW + newC * (float) newW;
  newW = oldW + newW;
  newC /= (float) newW;
  newW = MIN(newW, maxW);

  dst.clr = TO_UCHAR3(newC * 255.0f);
  dst.w_color = (uchar) newW;
}

template<bool hasColor, class TVoxel>
struct CombineVoxelInformation;

template<class TVoxel>
struct CombineVoxelInformation<false, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) &src, DEVICEPTR(TVoxel) &dst, int maxW) {
    combineVoxelDepthInformation(src, dst, maxW);
  }
};

template<class TVoxel>
struct CombineVoxelInformation<true, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) &src, DEVICEPTR(TVoxel) &dst, int maxW) {
    combineVoxelDepthInformation(src, dst, maxW);
    combineVoxelColorInformation(src, dst, maxW);
  }
};

