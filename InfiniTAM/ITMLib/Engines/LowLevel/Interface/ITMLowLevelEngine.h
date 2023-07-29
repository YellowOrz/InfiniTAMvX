// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Utils/ITMImageTypes.h"

namespace ITMLib {
// 底层的图像处理模块的接口（后面分成CPU和CUDA两种实现）
/// Interface to low level image processing engines.
class ITMLowLevelEngine {
public:
  virtual void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const = 0;
  virtual void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const = 0;
  virtual void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const = 0;
  /** RGB转灰度 */
  virtual void ConvertColourToIntensity(ITMFloatImage *image_out, const ITMUChar4Image *image_in) const = 0;
  /** 对灰度图进行均值滤波，图片大小不变 */
  virtual void FilterIntensity(ITMFloatImage *image_out, const ITMFloatImage *image_in) const = 0;
  /** 将图片缩小一半。通过2x2的均值滤波  */
  virtual void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const = 0;
  /** 将图片缩小一半。通过2x2的均值滤波 */
  virtual void FilterSubsample(ITMFloatImage *image_out, const ITMFloatImage *image_in) const = 0;
  /** 将有洞的图片缩小一半。通过2x2的均值滤波，并检查像素有效性 */
  virtual void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const = 0;
  /** 将有洞的图片缩小一半。通过2x2的均值滤波，并检查像素有效性 */
  virtual void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const = 0;

  /** 计算x方向的梯度 */
  virtual void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const = 0;
  /** 计算y方向的梯度 */
  virtual void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const = 0;
  /** 计算x和y方向的梯度 */
  virtual void GradientXY(ITMFloat2Image *grad_out, const ITMFloatImage *image_in) const = 0;
  /** 统计深度图中有效像素个数。深度>0 即 有效 */
  virtual int CountValidDepths(const ITMFloatImage *image_in) const = 0;

  ITMLowLevelEngine(void) {}
  virtual ~ITMLowLevelEngine(void) {}
};
} // namespace ITMLib
