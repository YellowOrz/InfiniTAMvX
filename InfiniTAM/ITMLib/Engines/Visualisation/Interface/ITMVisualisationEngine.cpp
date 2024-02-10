// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMVisualisationEngine.h"

using namespace ITMLib;

inline float interpolate(float val, float y0, float x0, float y1, float x1) {
  return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}
/** 函数曲线见说明材料的base.png */
inline float base(float val) {
  if (val <= -0.75f) return 0.0f;
  else if (val <= -0.25f) return interpolate(val, 0.0f, -0.75f, 1.0f, -0.25f);
  else if (val <= 0.25f) return 1.0f;
  else if (val <= 0.75f) return interpolate(val, 1.0f, 0.25f, 0.0f, 0.75f);
  else return 0.0;
}

void IITMVisualisationEngine::DepthToUchar4(ITMUChar4Image *dst, const ITMFloatImage *src) {
  Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
  const float *source = src->GetData(MEMORYDEVICE_CPU);
  int dataSize = static_cast<int>(dst->dataSize);

  memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);  // 全置为0
  //! 找到深度的最大值和最小值
  Vector4u *destUC4;
  float lims[2];    // 最小值＆最大值

  destUC4 = (Vector4u *) dest;  // TODO:　这么又取值一下的意义在哪儿？
  lims[0] = 100000.0f;
  lims[1] = -100000.0f;

  for (int idx = 0; idx < dataSize; idx++) {
    float sourceVal = source[idx];
    if (sourceVal > 0.0f) {
      lims[0] = MIN(lims[0], sourceVal);
      lims[1] = MAX(lims[1], sourceVal);
    }
  }

  float scale = ((lims[1] - lims[0]) != 0) ? 1.0f / (lims[1] - lims[0]) : 1.0f / lims[1]; // 缩放倍数

  if (lims[0] == lims[1]) return;
  //! 将深度值从float转成uchar4
  for (int idx = 0; idx < dataSize; idx++) {
    float sourceVal = source[idx];

    if (sourceVal > 0.0f) {
      // NOTE: 说明材料中有个depth_float2char4.py，将可视化下面的映射
      sourceVal = (sourceVal - lims[0]) * scale;  // 将深度值归一化到0~1

      destUC4[idx].r = (uchar) (base(sourceVal - 0.5f) * 255.0f);
      destUC4[idx].g = (uchar) (base(sourceVal) * 255.0f);
      destUC4[idx].b = (uchar) (base(sourceVal + 0.5f) * 255.0f);
      destUC4[idx].a = 255;
    }
  }
}

void IITMVisualisationEngine::NormalToUchar4(ITMUChar4Image *dst, const ITMFloat4Image *src) {
  Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
  const Vector4f *source = src->GetData(MEMORYDEVICE_CPU);
  int dataSize = static_cast<int>(dst->dataSize);
  //! 从float4转char4，范围从-1~1转换成0~255
  memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);  // 全置为0
  {
    for (int idx = 0; idx < dataSize; idx++) {
      Vector4f sourceVal = source[idx];
      if (sourceVal.w >= 0.0f) {
        dest[idx].r = (uchar) ((0.3f + (sourceVal.r + 1.0f) * 0.35f) * 255.0f);
        dest[idx].g = (uchar) ((0.3f + (sourceVal.g + 1.0f) * 0.35f) * 255.0f);
        dest[idx].b = (uchar) ((0.3f + (sourceVal.b + 1.0f) * 0.35f) * 255.0f); // TODO: 找到公式的来源

      }
    }
  }
}

void IITMVisualisationEngine::WeightToUchar4(ITMUChar4Image *dst, const ITMFloatImage *src) {
  Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
  const float *source = src->GetData(MEMORYDEVICE_CPU);
  int dataSize = static_cast<int>(dst->dataSize);
  //! 找到weight的最小值
  float mindepth = 1000;
  for (size_t i = 0; i < src->dataSize; i++)
    if (source[i] > 0) mindepth = MIN(mindepth, source[i]);
  //! 从float转char4
  memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);  // 全置为0
  {
    for (int idx = 0; idx < dataSize; idx++) {
      float sourceVal = source[idx];
      if (sourceVal > 0) {
        // NOTE: 说明材料中有个weight_float2char4.py，将可视化下面的映射
        sourceVal = mindepth / sourceVal * 0.8f + 0.2f;   // 将weight映射到0.2~1.0。对应.py文件中的line1
        dest[idx].r = (uchar) ((1 - sourceVal) * 255.0f); // 对应.py文件中的line2
        dest[idx].b = 0;  // TODO: 不需要，因为上面初始化成0了
        dest[idx].g = (uchar) (sourceVal * 255.0f);       // 对应.py文件中的line3 // TODO: 找到公式的来源
      }
    }
  }
}
