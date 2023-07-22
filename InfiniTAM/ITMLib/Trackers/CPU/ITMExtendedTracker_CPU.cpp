// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMExtendedTracker_CPU.h"
#include "../Shared/ITMExtendedTracker_Shared.h"

using namespace ITMLib;

ITMExtendedTracker_CPU::ITMExtendedTracker_CPU(Vector2i imgSize_d,
                                               Vector2i imgSize_rgb,
                                               bool useDepth,
                                               bool useColour,
                                               float colourWeight,
                                               TrackerIterationType *trackingRegime,
                                               int noHierarchyLevels,
                                               float terminationThreshold,
                                               float failureDetectorThreshold,
                                               float viewFrustum_min,
                                               float viewFrustum_max,
                                               float minColourGradient,
                                               float tukeyCutOff,
                                               int framesToSkip,
                                               int framesToWeight,
                                               const ITMLowLevelEngine *lowLevelEngine)
    : ITMExtendedTracker(imgSize_d,
                         imgSize_rgb,
                         useDepth,
                         useColour,
                         colourWeight,
                         trackingRegime,
                         noHierarchyLevels,
                         terminationThreshold,
                         failureDetectorThreshold,
                         viewFrustum_min,
                         viewFrustum_max,
                         minColourGradient,
                         tukeyCutOff,
                         framesToSkip,
                         framesToWeight,
                         lowLevelEngine,
                         MEMORYDEVICE_CPU) {}

ITMExtendedTracker_CPU::~ITMExtendedTracker_CPU(void) {}

int ITMExtendedTracker_CPU::ComputeGandH_Depth(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) {
  // 从scene中获取
  Vector4f *pointsMap = sceneHierarchyLevel_Depth->pointsMap->GetData(MEMORYDEVICE_CPU);//三维坐标点
  Vector4f *normalsMap = sceneHierarchyLevel_Depth->normalsMap->GetData(MEMORYDEVICE_CPU);//法向量
  Vector4f sceneIntrinsics = sceneHierarchyLevel_Depth->intrinsics;//相机场景内参
  Vector2i sceneImageSize = sceneHierarchyLevel_Depth->pointsMap->noDims;//场景图像大小
  // 从当前帧获取
  float *depth = viewHierarchyLevel_Depth->depth->GetData(MEMORYDEVICE_CPU);//当前帧深度值
  Vector4f viewIntrinsics = viewHierarchyLevel_Depth->intrinsics;
  Vector2i viewImageSize = viewHierarchyLevel_Depth->depth->noDims;  // 当前点法向量

  if (currentIterationType == TRACKER_ITERATION_NONE) return 0;

  bool shortIteration = (currentIterationType == TRACKER_ITERATION_ROTATION)
      || (currentIterationType == TRACKER_ITERATION_TRANSLATION);

  float sumHessian[6 * 6], sumNabla[6], sumF;
  int noValidPoints;
  int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

  noValidPoints = 0;
  sumF = 0.0f;
  memset(sumHessian, 0, sizeof(float) * noParaSQ);
  memset(sumNabla, 0, sizeof(float) * noPara);

  for (int y = 0; y < viewImageSize.y; y++)
    for (int x = 0; x < viewImageSize.x; x++) {
      float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

      for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
      for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

      bool isValidPoint;

      float depthWeight;

      //判断该帧是否为有效点
      if (framesProcessed < 100) {
        switch (currentIterationType) {
        case TRACKER_ITERATION_ROTATION:
          isValidPoint = computePerPointGH_exDepth<true, true, false>(
              localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight, viewImageSize,
              viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap,
              spaceThresh[currentLevelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
          break;
        case TRACKER_ITERATION_TRANSLATION:
          isValidPoint = computePerPointGH_exDepth<true, false, false>(
              localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight, viewImageSize,
              viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap,
              spaceThresh[currentLevelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
          break;
        case TRACKER_ITERATION_BOTH:
          isValidPoint = computePerPointGH_exDepth<false, false, false>(
              localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight, viewImageSize,
              viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap,
              spaceThresh[currentLevelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
          break;
        default:
          isValidPoint = false;
          break;
        }
      } else {
        switch (currentIterationType) {
        case TRACKER_ITERATION_ROTATION:
          isValidPoint = computePerPointGH_exDepth<true, true, true>(
              localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight, viewImageSize,
              viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap,
              spaceThresh[currentLevelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
          break;
        case TRACKER_ITERATION_TRANSLATION:
          isValidPoint = computePerPointGH_exDepth<true, false, true>(
              localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight, viewImageSize,
              viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap,
              spaceThresh[currentLevelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
          break;
        case TRACKER_ITERATION_BOTH:
          isValidPoint = computePerPointGH_exDepth<false, false, true>(
              localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], depthWeight, viewImageSize,
              viewIntrinsics, sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap,
              spaceThresh[currentLevelId], viewFrustum_min, viewFrustum_max, tukeyCutOff, framesToSkip, framesToWeight);
          break;
        default:
          isValidPoint = false;
          break;
        }
      }

      if (isValidPoint) {
        noValidPoints++;//TODO:该点有效，为什么无效点数目加一
        sumF += localF;//对每个点误差进行累加
        for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
        for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
      }
    }

  // Copy the lower triangular part of the matrix.
  for (int r = 0, counter = 0; r < noPara; r++)
    for (int c = 0; c <= r; c++, counter++)
      hessian[r + c * 6] = sumHessian[counter];

  // Transpose to fill the upper triangle.
  for (int r = 0; r < noPara; ++r)
    for (int c = r + 1; c < noPara; c++)
      hessian[r + c * 6] = hessian[c + r * 6];

  memcpy(nabla, sumNabla, noPara * sizeof(float));

  f = sumF;//最终的误差函数

  return noValidPoints;
}

int ITMExtendedTracker_CPU::ComputeGandH_RGB(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) {
  const Vector2i viewImageSize_depth = viewHierarchyLevel_Depth->depth->noDims;//当前帧深度图大小
  const Vector2i viewImageSize_rgb = viewHierarchyLevel_Intensity->intensity_prev->noDims;//当前帧彩色图大小

  const Vector4f *points_curr = reprojectedPointsLevel->data->GetData(MEMORYDEVICE_CPU);
  const float *intensities_prev = viewHierarchyLevel_Intensity->intensity_prev->GetData(MEMORYDEVICE_CPU);//上一帧点的强度
  const float *intensities_current = projectedIntensityLevel->data->GetData(MEMORYDEVICE_CPU);//当前帧点的强度
  const Vector2f *gradients = viewHierarchyLevel_Intensity->gradients->GetData(MEMORYDEVICE_CPU);//梯度

  Vector4f projParams_rgb = viewHierarchyLevel_Intensity->intrinsics;//彩色图项目参数
  Vector4f projParams_depth = viewHierarchyLevel_Depth->intrinsics;//深度图项目参数

  if (currentIterationType == TRACKER_ITERATION_NONE) return 0;

  bool shortIteration = (currentIterationType == TRACKER_ITERATION_ROTATION)
      || (currentIterationType == TRACKER_ITERATION_TRANSLATION);

  float sumHessian[6 * 6], sumNabla[6], sumF;
  int noValidPoints;
  //构建一个上三角矩阵来依次填入海塞矩阵
  int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;

  noValidPoints = 0;
  sumF = 0.0f;
  memset(sumHessian, 0, sizeof(float) * noParaSQ);
  memset(sumNabla, 0, sizeof(float) * noPara);

  for (int y = 0; y < viewImageSize_depth.y; y++)
    for (int x = 0; x < viewImageSize_depth.x; x++) {
      float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;

      for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
      for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

      bool isValidPoint = false;

      //判断该帧是否为有效点
      switch (currentIterationType) {
        case TRACKER_ITERATION_ROTATION:
          isValidPoint = computePerPointGH_exRGB_inv_Ab<true, true>(
              localF,
              localNabla,
              localHessian,
              x,
              y,
              points_curr,
              intensities_current,
              intensities_prev,
              gradients,
              viewImageSize_depth,
              viewImageSize_rgb,
              projParams_depth,
              projParams_rgb,
              approxInvPose,
              depthToRGBTransform * scenePose,
              colourThresh[currentLevelId],
              minColourGradient,
              viewFrustum_min,
              viewFrustum_max,
              tukeyCutOff
          );
          break;
        case TRACKER_ITERATION_TRANSLATION:
          isValidPoint = computePerPointGH_exRGB_inv_Ab<true, false>(
              localF,
              localNabla,
              localHessian,
              x,
              y,
              points_curr,
              intensities_current,
              intensities_prev,
              gradients,
              viewImageSize_depth,
              viewImageSize_rgb,
              projParams_depth,
              projParams_rgb,
              approxInvPose,
              depthToRGBTransform * scenePose,
              colourThresh[currentLevelId],
              minColourGradient,
              viewFrustum_min,
              viewFrustum_max,
              tukeyCutOff
          );
          break;
        case TRACKER_ITERATION_BOTH:
          isValidPoint = computePerPointGH_exRGB_inv_Ab<false, false>(
              localF,
              localNabla,
              localHessian,
              x,
              y,
              points_curr,
              intensities_current,
              intensities_prev,
              gradients,
              viewImageSize_depth,
              viewImageSize_rgb,
              projParams_depth,
              projParams_rgb,
              approxInvPose,
              depthToRGBTransform * scenePose,
              colourThresh[currentLevelId],
              minColourGradient,
              viewFrustum_min,
              viewFrustum_max,
              tukeyCutOff
          );
          break;
        default: isValidPoint = false;
          break;
      }

      if (isValidPoint) {
        noValidPoints++;
        sumF += localF;//对每个点误差进行累加
        for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
        for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
      }
    }

  // Copy the lower triangular part of the matrix.
  for (int r = 0, counter = 0; r < noPara; r++)
    for (int c = 0; c <= r; c++, counter++)
      hessian[r + c * 6] = sumHessian[counter];

  // Transpose to fill the upper triangle.
  for (int r = 0; r < noPara; ++r)
    for (int c = r + 1; c < noPara; c++)
      hessian[r + c * 6] = hessian[c + r * 6];

  //填入雅可比矩阵
  memcpy(nabla, sumNabla, noPara * sizeof(float));

  f = sumF;//最终的误差函数

  return noValidPoints;
}

void ITMExtendedTracker_CPU::ProjectCurrentIntensityFrame(ITMFloat4Image *points_out,
                                                          ITMFloatImage *intensity_out,
                                                          const ITMFloatImage *intensity_in,
                                                          const ITMFloatImage *depth_in,
                                                          const Vector4f &intrinsics_depth,
                                                          const Vector4f &intrinsics_rgb,
                                                          const Matrix4f &scenePose) {
  const Vector2i imageSize_rgb = intensity_in->noDims;//rgb图像大小
  const Vector2i imageSize_depth = depth_in->noDims; // 深度图大小，Also the size of the projected image

  points_out->ChangeDims(imageSize_depth); // Actual reallocation should happen only once per run.
  intensity_out->ChangeDims(imageSize_depth); // Actual reallocation should happen only once per run.

  const float *depths = depth_in->GetData(MEMORYDEVICE_CPU);
  const float *intensityIn = intensity_in->GetData(MEMORYDEVICE_CPU);
  Vector4f *pointsOut = points_out->GetData(MEMORYDEVICE_CPU);//输出的点
  float *intensityOut = intensity_out->GetData(MEMORYDEVICE_CPU);//输出强度

  //计算当前帧输出的点和输出强度
  for (int y = 0; y < imageSize_depth.y; y++)
    for (int x = 0; x < imageSize_depth.x; x++)
      projectPoint_exRGB(x,
                         y,
                         pointsOut,
                         intensityOut,
                         intensityIn,
                         depths,
                         imageSize_rgb,
                         imageSize_depth,
                         intrinsics_rgb,
                         intrinsics_depth,
                         scenePose);
}
