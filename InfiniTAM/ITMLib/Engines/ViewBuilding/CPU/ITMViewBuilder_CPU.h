// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMViewBuilder.h"

namespace ITMLib {
class ITMViewBuilder_CPU : public ITMViewBuilder {
public:
  /**
   * @brief 将Kinect的视差图转成深度图
   * @param[out] depth_out 真正的深度图
   * @param[in] disp_in Kinect拍摄的视差图
   * @param[in] depthIntrinsics depth的内参
   * @param[in] disparityCalibParams  转换的两个参数。来自相机参数文件的最后一行
   */
  void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in,
                               const ITMIntrinsics *depthIntrinsics, Vector2f disparityCalibParams);
  /**
   * @brief 将原始读取的深度图转成深度图
   * @param[out] depth_out 真正的深度图
   * @param[in] depth_in 读入的原始深度图
   * @param[in] depthCalibParams 转换的两个参数。来自相机参数文件的最后一行
   */
  void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams);
  /** 对深度图进行双边滤波 */
  void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);
  /**
   * @brief 计算深度图的法向量和不确定性（权重），用于衡量深度图噪声
   * @param[out] normal_out 法向量
   * @param[out] sigmaZ_out 不确定性（权重）
   * @param[in] depth_in 
   * @param[in] intrinsic 相机内参，即fx、fy、cx、cy
   */
  void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in,
                               Vector4f intrinsic);
  /**
   * @brief 更新view中的RGB-D数据
   * @param[out] view tracking的输入数据
   * @param[in] rgbImage 
   * @param[in] rawDepthImage 
   * @param[in] useBilateralFilter 
   * @param[in] modelSensorNoise 是否计算深度相机的噪声（法向量和不确定度）
   * @param[in] storePreviousImage 
   */
  void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter,
                  bool modelSensorNoise = false, bool storePreviousImage = true);
  /**
   * @brief 更新view中的RGB-D和IMU数据
   * @param[out] view tracking的输入数据
   * @param[in] rgbImage 
   * @param[in] depthImage 
   * @param[in] useBilateralFilter 
   * @param[in] imuMeasurement 
   * @param[in] modelSensorNoise 是否计算深度相机的噪声（法向量和不确定度）
   * @param[in] storePreviousImage 
   */
  void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter,
                  ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise = false, bool storePreviousImage = true);

  ITMViewBuilder_CPU(const ITMRGBDCalib &calib);
  ~ITMViewBuilder_CPU(void);
};
} // namespace ITMLib
