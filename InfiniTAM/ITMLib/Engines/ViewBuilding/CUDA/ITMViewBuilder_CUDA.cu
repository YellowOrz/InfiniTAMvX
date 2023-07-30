// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMViewBuilder_CUDA.h"

#include "../Shared/ITMViewBuilder_Shared.h"
#include "../../../../ORUtils/CUDADefines.h"
#include "../../../../ORUtils/MemoryBlock.h"

using namespace ITMLib;
using namespace ORUtils;

ITMViewBuilder_CUDA::ITMViewBuilder_CUDA(const ITMRGBDCalib &calib) : ITMViewBuilder(calib) {}
ITMViewBuilder_CUDA::~ITMViewBuilder_CUDA(void) {}

//---------------------------------------------------------------------------
//
// kernel function declaration 
//
//---------------------------------------------------------------------------
/** 将Kinect的视差图转成深度图 */
__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams,
                                               float fx_depth, Vector2i imgSize);
/** Affine模式的深度图，将单个像素的深度值从short转float */                                               
__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize,
                                                 Vector2f depthCalibParams);
/** 对图像中的单个像素进行双边滤波 */                                                 
__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
/** 计算深度图单个像素的法向量和不确定性（权重），用于衡量深度图噪声 */
__global__ void ComputeNormalAndWeight_device(const float *depth_in, Vector4f *normal_out, float *sigmaL_out,
                                              Vector2i imgDims, Vector4f intrinsic);

//---------------------------------------------------------------------------
//
// host methods
//
//---------------------------------------------------------------------------

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage,
                                     bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage) {
  //! 防止view还未初始化
  if (*view_ptr == NULL) {
    *view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, true);
    if (this->shortImage != NULL) delete this->shortImage;
    this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, true);
    if (this->floatImage != NULL) delete this->floatImage;
    this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, true);

    if (modelSensorNoise) {
      (*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->noDims, true, true);
      (*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, true);
    }
  }

  ITMView *view = *view_ptr;
  //! 存储上一帧
  if (storePreviousImage) {
    if (!view->rgb_prev) view->rgb_prev = new ITMUChar4Image(rgbImage->noDims, true, true);
    else view->rgb_prev->SetFrom(view->rgb, MemoryBlock<Vector4u>::CUDA_TO_CUDA);
  }
  //! 将rgb-d图保存到view中
  view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CUDA);
  this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CUDA);
  //! 将深度图 转成能用的类型。因为读进来的深度图是转换过的，使用short类型存储
  switch (view->calib.disparityCalib.GetType()) {
  case ITMDisparityCalib::TRAFO_KINECT:
    this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib.intrinsics_d),
                                  view->calib.disparityCalib.GetParams());
    break;
  case ITMDisparityCalib::TRAFO_AFFINE:
    this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib.disparityCalib.GetParams());
    break;
  default: break;
  }
  //! 对 深度图 做5次 双边滤波
  if (useBilateralFilter) {
    //5 steps of bilateral filtering
    this->DepthFiltering(this->floatImage, view->depth);
    this->DepthFiltering(view->depth, this->floatImage);
    this->DepthFiltering(this->floatImage, view->depth);
    this->DepthFiltering(view->depth, this->floatImage);
    this->DepthFiltering(this->floatImage, view->depth);
    view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CUDA_TO_CUDA);
  }
  //! 计算深度图的法向量和不确定性（权重），用于衡量深度图噪声
  if (modelSensorNoise) {
    this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth,
                                  view->calib.intrinsics_d.projectionParamsSimple.all);
  }
}

void ITMViewBuilder_CUDA::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage,
                                     bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement, bool modelSensorNoise,
                                     bool storePreviousImage) {
  //! 防止view还未初始化
  if (*view_ptr == NULL) {
    *view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, true);
    if (this->shortImage != NULL) delete this->shortImage;
    this->shortImage = new ITMShortImage(depthImage->noDims, true, true);
    if (this->floatImage != NULL) delete this->floatImage;
    this->floatImage = new ITMFloatImage(depthImage->noDims, true, true);

    if (modelSensorNoise) {
      (*view_ptr)->depthNormal = new ITMFloat4Image(depthImage->noDims, true, true);
      (*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->noDims, true, true);
    }
  }
  //! 将IMU保存到view中     // TODO：不用保存上一帧的IMU信息吗？
  ITMViewIMU *imuView = (ITMViewIMU *) (*view_ptr);
  imuView->imu->SetFrom(imuMeasurement);
  //! RGBD数据的保存相同
  this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ITMViewBuilder_CUDA::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in,
                                                  const ITMIntrinsics *depthIntrinsics, Vector2f disparityCalibParams) {
  Vector2i imgSize = depth_in->noDims;

  const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
  float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

  float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

  dim3 blockSize(16, 16);
  dim3 gridSize
      ((int) ceil((float) imgSize.x / (float) blockSize.x), (int) ceil((float) imgSize.y / (float) blockSize.y));

  convertDisparityToDepth_device << < gridSize, blockSize >> >(d_out, d_in, disparityCalibParams, fx_depth, imgSize);
  ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ConvertDepthAffineToFloat(ITMFloatImage *depth_out,
                                                    const ITMShortImage *depth_in,
                                                    Vector2f depthCalibParams) {
  Vector2i imgSize = depth_in->noDims;

  const short *d_in = depth_in->GetData(MEMORYDEVICE_CUDA);
  float *d_out = depth_out->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(16, 16);
  dim3 gridSize
      ((int) ceil((float) imgSize.x / (float) blockSize.x), (int) ceil((float) imgSize.y / (float) blockSize.y));

  convertDepthAffineToFloat_device << < gridSize, blockSize >> >(d_out, d_in, imgSize, depthCalibParams);
  ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) {
  Vector2i imgDims = image_in->noDims;

  const float *imageData_in = image_in->GetData(MEMORYDEVICE_CUDA);
  float *imageData_out = image_out->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(16, 16);
  dim3 gridSize
      ((int) ceil((float) imgDims.x / (float) blockSize.x), (int) ceil((float) imgDims.y / (float) blockSize.y));

  filterDepth_device << < gridSize, blockSize >> >(imageData_out, imageData_in, imgDims);
  ORcudaKernelCheck;
}

void ITMViewBuilder_CUDA::ComputeNormalAndWeights(ITMFloat4Image *normal_out,
                                                  ITMFloatImage *sigmaZ_out,
                                                  const ITMFloatImage *depth_in,
                                                  Vector4f intrinsic) {
  Vector2i imgDims = depth_in->noDims;

  const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CUDA);

  float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CUDA);
  Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CUDA);

  dim3 blockSize(16, 16);
  dim3 gridSize
      ((int) ceil((float) imgDims.x / (float) blockSize.x), (int) ceil((float) imgDims.y / (float) blockSize.y));

  ComputeNormalAndWeight_device <<< gridSize, blockSize >> >(depthData_in, normalData_out, sigmaZData_out, imgDims, intrinsic);
  ORcudaKernelCheck;
}

//---------------------------------------------------------------------------
//
// kernel function implementation
//
//---------------------------------------------------------------------------

__global__ void convertDisparityToDepth_device(float *d_out, const short *d_in, Vector2f disparityCalibParams,
                                               float fx_depth, Vector2i imgSize) {
  int x = threadIdx.x + blockIdx.x * blockDim.x;
  int y = threadIdx.y + blockIdx.y * blockDim.y;

  if ((x >= imgSize.x) || (y >= imgSize.y)) return;

  convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize,
                                                 Vector2f depthCalibParams) {
  int x = threadIdx.x + blockIdx.x * blockDim.x;
  int y = threadIdx.y + blockIdx.y * blockDim.y;

  if ((x >= imgSize.x) || (y >= imgSize.y)) return;

  convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims) {
  int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

  if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2) return;

  filterDepth(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void ComputeNormalAndWeight_device(const float *depth_in, Vector4f *normal_out, float *sigmaZ_out,
                                              Vector2i imgDims, Vector4f intrinsic) {
  int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
  int idx = x + y * imgDims.x;

  if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2) {
    normal_out[idx].w = -1.0f;
    sigmaZ_out[idx] = -1;
    return;
  } else {
    computeNormalAndWeight(depth_in, normal_out, sigmaZ_out, x, y, imgDims, intrinsic);
  }
}
