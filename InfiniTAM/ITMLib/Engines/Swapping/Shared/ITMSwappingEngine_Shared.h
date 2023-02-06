// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"
/**
 * 深度图的
 * @tparam TVoxel 体素信息
 * @param src 上一帧的体素信息
 * @param dst 设备上新一帧的体素信息
 * @param maxW 最大深度内的点的个数
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelDepthInformation(const CONSTPTR(TVoxel) &src,
                                                            DEVICEPTR(TVoxel) &dst,
                                                            int maxW) {
  int newW = dst.w_depth;   // 新一帧视角下sdf点下存在的点的个数（sdf点即体素距离物体最近的距离点）
  int oldW = src.w_depth;   // 上一帧视角下sdf点下存在的点的个数
  float newF = TVoxel::valueToFloat(dst.sdf);   // 新一帧体素与物体的最近距离
  float oldF = TVoxel::valueToFloat(src.sdf);   // 上一帧体素与物体的最近距离

  if (oldW == 0) return; // 判断所否有多个的值
  //有，进行优化（获取的深度信息不等于最终的sdf）
  newF = oldW * oldF + newW * newF; // TODO(wangyuren)还不清楚为什么这么算
  newW = oldW + newW;               // 记录
  newF /= newW;                     // 优化后新一帧的体素与物体的最近距离
  newW = MIN(newW, maxW);           // 取最小值作为新一帧视角下sdf点下存在的点的个数

  dst.w_depth = newW;   // 新一帧视角下sdf点下存在的点的个数
  dst.sdf = TVoxel::floatToValue(newF); // 优化后新一帧的体素与物体的最近距离
}
/**
 * 彩色图的
 * @tparam TVoxel 体素信息
 * @param src 上一帧的体素信息
 * @param dst 设备上新一帧的体素信息
 * @param maxW
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelColorInformation(const CONSTPTR(TVoxel) &src,
                                                            DEVICEPTR(TVoxel) &dst,
                                                            int maxW) {
  int newW = dst.w_color;   // 新一帧视角下的体素下存在的点的个数
  int oldW = src.w_color;   // 旧一帧视角下的体素下存在的点的个数
  Vector3f newC = dst.clr.toFloat() / 255.0f;   //新一帧该点的rgb信息
  Vector3f oldC = src.clr.toFloat() / 255.0f;   //旧一帧该点的rgb信息

  if (oldW == 0) return;
  // TODO(wangyuren)同上一个一样的算法，但原理是啥？
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

