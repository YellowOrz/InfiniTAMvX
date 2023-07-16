// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once
#ifndef __METALC__
#include <math.h>
#endif

#include "ITMMath.h"
#include "../../ORUtils/PlatformIndependence.h"

template<typename T>
_CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear(const CONSTPTR(ORUtils::Vector4<T>) *source,
                                                       const THREADPTR(Vector2f) &position,
                                                       const CONSTPTR(Vector2i) &imgSize) {
  const Vector2i p((int) floor(position.x), (int) floor(position.y));//x，y方向放大倍数
  const Vector2f delta(position.x - (float) p.x, position.y - (float) p.y);

  //获取目标像素周围四个点坐标值
  ORUtils::Vector4<T> a = source[p.x + p.y * imgSize.x];
  ORUtils::Vector4<T> b(T(0)), c(T(0)), d(T(0));

  if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
  if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
  if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

  Vector4f result;
  //通过目标图像在原图对应的几何坐标位置,输出对应位置的几何坐标
  result.x = ((float) a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.x * delta.x * (1.0f - delta.y) +
      (float) c.x * (1.0f - delta.x) * delta.y + (float) d.x * delta.x * delta.y);
  result.y = ((float) a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.y * delta.x * (1.0f - delta.y) +
      (float) c.y * (1.0f - delta.x) * delta.y + (float) d.y * delta.x * delta.y);
  result.z = ((float) a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.z * delta.x * (1.0f - delta.y) +
      (float) c.z * (1.0f - delta.x) * delta.y + (float) d.z * delta.x * delta.y);
  result.w = ((float) a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.w * delta.x * (1.0f - delta.y) +
      (float) c.w * (1.0f - delta.x) * delta.y + (float) d.w * delta.x * delta.y);

  return result;
}

template<typename T>
_CPU_AND_GPU_CODE_ inline float interpolateBilinear_single(const CONSTPTR(T) *source,
                                                           const THREADPTR(Vector2f) &position,
                                                           const CONSTPTR(Vector2i) &imgSize) {
  const Vector2i p((int) floor(position.x), (int) floor(position.y));
  const Vector2f delta(position.x - (float) p.x, position.y - (float) p.y);

  T a = source[p.x + p.y * imgSize.x];
  T b(0), c(0), d(0);

  if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
  if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
  if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

  float result = ((float) a * (1.0f - delta.x) * (1.0f - delta.y) + (float) b * delta.x * (1.0f - delta.y) +
      (float) c * (1.0f - delta.x) * delta.y + (float) d * delta.x * delta.y);

  return result;
}

template<typename T>
_CPU_AND_GPU_CODE_ inline Vector2f interpolateBilinear_Vector2(const CONSTPTR(ORUtils::Vector2<T>) *source,
                                                               const THREADPTR(Vector2f) &position,
                                                               const CONSTPTR(Vector2i) &imgSize) {
  const Vector2i p((int) floor(position.x), (int) floor(position.y));           // position的整数部分
  const Vector2f delta(position.x - (float) p.x, position.y - (float) p.y);     // position的小数部分

  ORUtils::Vector2<T> a = source[p.x + p.y * imgSize.x];
  ORUtils::Vector2<T> b(T(0)), c(T(0)), d(T(0));

  if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
  if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
  if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

  Vector2f result;
  result.x = ((float) a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.x * delta.x * (1.0f - delta.y) +
              (float) c.x * (1.0f - delta.x) * delta.y + (float) d.x * delta.x * delta.y);
  result.y = ((float) a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.y * delta.x * (1.0f - delta.y) +
              (float) c.y * (1.0f - delta.x) * delta.y + (float) d.y * delta.x * delta.y);

  return result;
}

/**
 * 在source上找到position的邻居，对position这个位置进行插值
 * @details position是float，邻居坐标即是整数
 * @tparam T
 * @param[in] source    反投影后的深度图 或者 对应的法向量图。Vetor4的第4维是权重？？？
 * @param[in] position  二维图像上的坐标
 * @param[in] imgSize
 * @return  插值后的三维点
 */
template<typename T>
_CPU_AND_GPU_CODE_ inline Vector4f interpolateBilinear_withHoles(const CONSTPTR(ORUtils::Vector4<T>) *source,
                                                                 const THREADPTR(Vector2f) &position,
                                                                 const CONSTPTR(Vector2i) &imgSize) {
  const Vector2s p((short) floor(position.x), (short) floor(position.y));//x，y方向放大倍数
  const Vector2f delta(position.x - (float) p.x, position.y - (float) p.y);

  //获取目标像素周围四个点坐标值
  const ORUtils::Vector4<T> a = source[p.x + p.y * imgSize.x];
  const ORUtils::Vector4<T> b = source[(p.x + 1) + p.y * imgSize.x];
  const ORUtils::Vector4<T> c = source[p.x + (p.y + 1) * imgSize.x];
  const ORUtils::Vector4<T> d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

  Vector4f result;
  // 超出图片边界则退出
  if (a.w < 0 || b.w < 0 || c.w < 0 || d.w < 0) {
    result.x = 0;
    result.y = 0;
    result.z = 0;
    result.w = -1.0f;
    return result;
  }

  //通过目标图像在原图对应的几何坐标位置,输出对应位置的几何坐标
  result.x = ((float) a.x * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.x * delta.x * (1.0f - delta.y) +
      (float) c.x * (1.0f - delta.x) * delta.y + (float) d.x * delta.x * delta.y);
  result.y = ((float) a.y * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.y * delta.x * (1.0f - delta.y) +
      (float) c.y * (1.0f - delta.x) * delta.y + (float) d.y * delta.x * delta.y);
  result.z = ((float) a.z * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.z * delta.x * (1.0f - delta.y) +
      (float) c.z * (1.0f - delta.x) * delta.y + (float) d.z * delta.x * delta.y);
  result.w = ((float) a.w * (1.0f - delta.x) * (1.0f - delta.y) + (float) b.w * delta.x * (1.0f - delta.y) +
      (float) c.w * (1.0f - delta.x) * delta.y + (float) d.w * delta.x * delta.y);

  return result;
}

template<typename T>
_CPU_AND_GPU_CODE_ inline float interpolateBilinear_withHoles_single(const CONSTPTR(T) *source,
                                                                     const THREADPTR(Vector2f) &position,
                                                                     const CONSTPTR(Vector2i) &imgSize) {
  const Vector2i p((int) floor(position.x), (int) floor(position.y));
  const Vector2f delta(position.x - (float) p.x, position.y - (float) p.y);

  T a = source[p.x + p.y * imgSize.x];
  T b(0), c(0), d(0);

  if (delta.x != 0) b = source[(p.x + 1) + p.y * imgSize.x];
  if (delta.y != 0) c = source[p.x + (p.y + 1) * imgSize.x];
  if (delta.x != 0 && delta.y != 0) d = source[(p.x + 1) + (p.y + 1) * imgSize.x];

  if (a < 0 || b < 0 || c < 0 || d < 0) return -1;

  float result = ((float) a * (1.0f - delta.x) * (1.0f - delta.y) + (float) b * delta.x * (1.0f - delta.y) +
      (float) c * (1.0f - delta.x) * delta.y + (float) d * delta.x * delta.y);

  return result;
}
