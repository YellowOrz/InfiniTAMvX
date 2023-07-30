// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CPU.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib &calib) : ITMViewBuilder(calib) {}
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) {}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage,
                                    bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage) {
  //! 防止view还未初始化
  if (*view_ptr == NULL) {
    *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
    if (this->shortImage != NULL)
      delete this->shortImage;
    this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
    if (this->floatImage != NULL)
      delete this->floatImage;
    this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

    if (modelSensorNoise) {
      (*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->noDims, true, false);
      (*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, false);
    }
  }
  ITMView *view = *view_ptr;
  //! 存储上一帧
  if (storePreviousImage) {
    if (!view->rgb_prev)
      view->rgb_prev = new ITMUChar4Image(rgbImage->noDims, true, false);
    else
      view->rgb_prev->SetFrom(view->rgb, MemoryBlock<Vector4u>::CPU_TO_CPU);
  }
  //! 将rgb-d图保存到view中
  view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
  this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);
  //! 将深度图 转成能用的类型。因为读进来的深度图是转换过的，使用short类型存储
  switch (view->calib.disparityCalib.GetType()) {
  case ITMDisparityCalib::TRAFO_KINECT: // Kinect相机专有的转换模式
    this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib.intrinsics_d),
                                  view->calib.disparityCalib.GetParams());
    break;
  case ITMDisparityCalib::TRAFO_AFFINE: // （默认&常用）线性转换模式
    this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib.disparityCalib.GetParams());
    break;
  default: break;
  }
  //! 对 深度图 做5次 双边滤波
  if (useBilateralFilter) {
    // 5 steps of bilateral filtering
    this->DepthFiltering(this->floatImage, view->depth);
    this->DepthFiltering(view->depth, this->floatImage);
    this->DepthFiltering(this->floatImage, view->depth);
    this->DepthFiltering(view->depth, this->floatImage);
    this->DepthFiltering(this->floatImage, view->depth);
    view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CPU_TO_CPU);
  }
  //! 计算深度图的法向量和不确定性（权重），用于衡量深度图噪声
  if (modelSensorNoise) {
    this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth,
                                  view->calib.intrinsics_d.projectionParamsSimple.all);
  }
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage,
                                    bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise,
                                    bool storePreviousImage) {
  //! 防止view还未初始化
  if (*view_ptr == NULL) {
    *view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
    if (this->shortImage != NULL)
      delete this->shortImage;
    this->shortImage = new ITMShortImage(depthImage->noDims, true, false);
    if (this->floatImage != NULL)
      delete this->floatImage;
    this->floatImage = new ITMFloatImage(depthImage->noDims, true, false);

    if (modelSensorNoise) {
      (*view_ptr)->depthNormal = new ITMFloat4Image(depthImage->noDims, true, false);
      (*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->noDims, true, false);
    }
  }
  //! 将IMU保存到view中     // TODO：不用保存上一帧的IMU信息吗？
  ITMViewIMU *imuView = (ITMViewIMU *)(*view_ptr);
  imuView->imu->SetFrom(imuMeasurement);
  //! RGBD数据的保存相同
  this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in,
                                                 const ITMIntrinsics *depthIntrinsics, Vector2f disparityCalibParams) {
  Vector2i imgSize = depth_in->noDims;

  const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
  float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

  float fx_depth = depthIntrinsics->projectionParamsSimple.fx;
  // 对每个像素进行转换
  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++)
      convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in,
                                                   const Vector2f depthCalibParams) {
  Vector2i imgSize = depth_in->noDims;

  const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
  float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);
  // 对每个像素进行转换
  for (int y = 0; y < imgSize.y; y++)
    for (int x = 0; x < imgSize.x; x++)
      convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

void ITMViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) {
  Vector2i imgSize = image_in->noDims;

  image_out->Clear();

  float *imout = image_out->GetData(MEMORYDEVICE_CPU);
  const float *imin = image_in->GetData(MEMORYDEVICE_CPU);
  // 对每个像素进行滤波
  for (int y = 2; y < imgSize.y - 2; y++)
    for (int x = 2; x < imgSize.x - 2; x++)
      filterDepth(imout, imin, x, y, imgSize);
}

void ITMViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out,
                                                 const ITMFloatImage *depth_in, Vector4f intrinsic) {
  Vector2i imgDims = depth_in->noDims;

  const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

  float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
  Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

  for (int y = 2; y < imgDims.y - 2; y++)
    for (int x = 2; x < imgDims.x - 2; x++)
      computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}
