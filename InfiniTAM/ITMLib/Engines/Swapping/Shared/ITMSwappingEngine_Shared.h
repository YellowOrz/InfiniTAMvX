// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMMath.h"
/**
 * 融合两个voxel的depth信息
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in] src 
 * @param[in,out] dst 
 * @param[in] maxW 最大观测数量，用来限制voxel的权重
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelDepthInformation(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst,
                                                            int maxW) {
  // 取出depth和权重
  int newW = dst.w_depth;
  int oldW = src.w_depth;
  float newF = TVoxel::valueToFloat(dst.sdf);
  float oldF = TVoxel::valueToFloat(src.sdf);
  if (oldW == 0) return;
  // 加权取平均
  newF = oldW * oldF + newW * newF;
  newW = oldW + newW;
  newF /= newW;
  newW = MIN(newW, maxW);   // 限制权重的上限
  // 记录
  dst.w_depth = newW; // 新一帧视角下sdf点下存在的点的个数
  dst.sdf = TVoxel::floatToValue(newF); // 优化后新一帧的体素与物体的最近距离
}

/**
 * 融合两个voxel的RGB信息
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in] src 
 * @param[in,out] dst 
 * @param[in] maxW 最大观测数量，用来限制voxel的权重
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline void combineVoxelColorInformation(const CONSTPTR(TVoxel) & src, DEVICEPTR(TVoxel) & dst,
                                                            int maxW) {
  // 取出RGB和权重
  int newW = dst.w_color;
  int oldW = src.w_color;
  Vector3f newC = dst.clr.toFloat() / 255.0f;
  Vector3f oldC = src.clr.toFloat() / 255.0f;
  if (oldW == 0) return;
  // 加权取平均
  newC = oldC * (float) oldW + newC * (float) newW;
  newW = oldW + newW;
  newC /= (float) newW;
  newW = MIN(newW, maxW);   // 限制权重的上限
  // 记录
  dst.clr = TO_UCHAR3(newC * 255.0f);
  dst.w_color = (uchar) newW;
}

/**
 * 融合两个voxel的信息
 * @tparam hasColor 是C++的非类型模板参数
 * @tparam TVoxel 
 */
template<bool hasColor, class TVoxel>
struct CombineVoxelInformation;   // TODO：为啥还要写下面两个结构体？？？不能通过if-else判断吗？？？

/**
 * 融合两个voxel的信息（无color）
 * @tparam TVoxel 
 */
template<class TVoxel>
struct CombineVoxelInformation<false, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) &src, DEVICEPTR(TVoxel) &dst, int maxW) {
    combineVoxelDepthInformation(src, dst, maxW);
  }
};

/**
 * 融合两个voxel的信息（有color）
 * @tparam TVoxel 
 */
template<class TVoxel>
struct CombineVoxelInformation<true, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(const CONSTPTR(TVoxel) &src, DEVICEPTR(TVoxel) &dst, int maxW) {
    combineVoxelDepthInformation(src, dst, maxW);
    combineVoxelColorInformation(src, dst, maxW);
  }
};

// TODO: 没有融合confident的信息的吗
