// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMLowLevelEngine.h"

namespace ITMLib {
class ITMLowLevelEngine_CPU : public ITMLowLevelEngine {
 public:
  void CopyImage(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const;
  void CopyImage(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
  void CopyImage(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const;
  /** RGB转灰度 */
  void ConvertColourToIntensity(ITMFloatImage *image_out, const ITMUChar4Image *image_in) const;
  /** 对灰度图进行均值滤波，图片大小不变 */
  void FilterIntensity(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
  /** 将图片缩小一半。通过2x2的均值滤波  */
  void FilterSubsample(ITMUChar4Image *image_out, const ITMUChar4Image *image_in) const;
  /** 将图片缩小一半。通过2x2的均值滤波
   * @note 不用模板函数是因为Vector4u里面有4个数字
   */
  void FilterSubsample(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
  /** 将有洞的图片缩小一半。通过2x2的均值滤波，并检查像素有效性 */
  void FilterSubsampleWithHoles(ITMFloatImage *image_out, const ITMFloatImage *image_in) const;
  /** 将有洞的图片缩小一半。通过2x2的均值滤波，并检查像素有效性 */
  void FilterSubsampleWithHoles(ITMFloat4Image *image_out, const ITMFloat4Image *image_in) const;
  
  /** 计算x方向的梯度 */
  void GradientX(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const;
  /** 计算y方向的梯度 */
  void GradientY(ITMShort4Image *grad_out, const ITMUChar4Image *image_in) const;
  /** 计算x和y方向的梯度 */
  void GradientXY(ITMFloat2Image *grad_out, const ITMFloatImage *image_in) const;
  /** 统计深度图中有效像素个数。深度>0 即 有效 */
  int CountValidDepths(const ITMFloatImage *image_in) const;

  ITMLowLevelEngine_CPU(void);
  ~ITMLowLevelEngine_CPU(void);
};
}
