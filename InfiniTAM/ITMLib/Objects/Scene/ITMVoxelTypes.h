// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"

/** \brief
    存储单个voxel的信息，使用float类型 && 有RGB。Stores the information of a single voxel in the volume
*/
struct ITMVoxel_f_rgb {
  _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
  _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
  _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

  static const CONSTPTR(bool) hasColorInformation = true;
  static const CONSTPTR(bool) hasConfidenceInformation = false;
  static const CONSTPTR(bool) hasSemanticInformation = false;

  /** Value of the truncated signed distance transformation. */
  float sdf;
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;  // sdf的观测次数，作为权重
  /** RGB colour information stored for this voxel. */
  Vector3u clr;
  /** Number of observations that made up @p clr. */
  uchar w_color;

  _CPU_AND_GPU_CODE_ ITMVoxel_f_rgb() {
    sdf = SDF_initialValue();
    w_depth = 0;
    clr = Vector3u((uchar) 0);
    w_color = 0;
  }
};

/** \brief
    存储单个voxel的信息，使用short类型 && 有RGB。Stores the information of a single voxel in the volume
    @note 取值范围为0 ~ 2^15-1  // TODO: 为啥不用unsigned short？？？
*/
struct ITMVoxel_s_rgb {
  _CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
  _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float) (x) / 32767.0f; }
  _CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short) ((x) * 32767.0f); }

  static const CONSTPTR(bool) hasColorInformation = true;
  static const CONSTPTR(bool) hasConfidenceInformation = false;
  static const CONSTPTR(bool) hasSemanticInformation = false;

  /** Value of the truncated signed distance transformation. */
  short sdf;
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;  // sdf的观测次数，作为权重
  /** Padding that may or may not improve performance on certain GPUs */
  //uchar pad;
  /** RGB colour information stored for this voxel. */
  Vector3u clr;
  /** Number of observations that made up @p clr. */
  uchar w_color;

  _CPU_AND_GPU_CODE_ ITMVoxel_s_rgb() {
    sdf = SDF_initialValue();
    w_depth = 0;
    clr = Vector3u((uchar) 0);
    w_color = 0;
  }
};

/** 存储单个voxel的信息，使用short类型 && 无RGB。 */
struct ITMVoxel_s {
  _CPU_AND_GPU_CODE_ static short SDF_initialValue() { return 32767; }
  _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return (float) (x) / 32767.0f; }
  _CPU_AND_GPU_CODE_ static short floatToValue(float x) { return (short) ((x) * 32767.0f); }

  static const CONSTPTR(bool) hasColorInformation = false;
  static const CONSTPTR(bool) hasConfidenceInformation = false;
  static const CONSTPTR(bool) hasSemanticInformation = false;

  /** Value of the truncated signed distance transformation. */
  short sdf;  
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;  // sdf的观测次数，作为权重
  /** Padding that may or may not improve performance on certain GPUs */
  //uchar pad;

  _CPU_AND_GPU_CODE_ ITMVoxel_s() {
    sdf = SDF_initialValue();
    w_depth = 0;
  }
};

/** 存储单个voxel的信息，使用float类型 && 无RGB。 */
struct ITMVoxel_f {
  _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
  _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
  _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

  static const CONSTPTR(bool) hasColorInformation = false;
  static const CONSTPTR(bool) hasConfidenceInformation = false;
  static const CONSTPTR(bool) hasSemanticInformation = false;

  /** Value of the truncated signed distance transformation. */
  float sdf;
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;  // sdf的观测次数，作为权重
  /** Padding that may or may not improve performance on certain GPUs */
  //uchar pad;

  _CPU_AND_GPU_CODE_ ITMVoxel_f() {
    sdf = SDF_initialValue();
    w_depth = 0;
  }
};

/** 存储单个voxel的信息，使用float类型 && 无RGB && 有置信度。 */
struct ITMVoxel_f_conf {
  _CPU_AND_GPU_CODE_ static float SDF_initialValue() { return 1.0f; }
  _CPU_AND_GPU_CODE_ static float valueToFloat(float x) { return x; }
  _CPU_AND_GPU_CODE_ static float floatToValue(float x) { return x; }

  static const CONSTPTR(bool) hasColorInformation = false;
  static const CONSTPTR(bool) hasConfidenceInformation = true;
  static const CONSTPTR(bool) hasSemanticInformation = false;

  /** Value of the truncated signed distance transformation. */
  float sdf;
  /** Number of fused observations that make up @p sdf. */
  uchar w_depth;  // sdf的观测次数，作为权重
  /** Padding that may or may not improve performance on certain GPUs */
  //uchar pad;
  float confidence; // sdf的置信度 // TODO: 跟权重有啥区别？？？

  _CPU_AND_GPU_CODE_ ITMVoxel_f_conf() {
    sdf = SDF_initialValue();
    w_depth = 0;
    confidence = 0.0f;
  }
};

