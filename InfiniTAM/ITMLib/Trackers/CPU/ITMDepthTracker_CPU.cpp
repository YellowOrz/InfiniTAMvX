// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CPU.h"
#include "../Shared/ITMDepthTracker_Shared.h"

using namespace ITMLib;

ITMDepthTracker_CPU::ITMDepthTracker_CPU(Vector2i imgSize,
                                         TrackerIterationType *trackingRegime,
                                         int noHierarchyLevels,
                                         float terminationThreshold,
                                         float failureDetectorThreshold,
                                         const ITMLowLevelEngine *lowLevelEngine)
    : ITMDepthTracker(imgSize,
                      trackingRegime,
                      noHierarchyLevels,
                      terminationThreshold,
                      failureDetectorThreshold,
                      lowLevelEngine,
                      MEMORYDEVICE_CPU) {}

ITMDepthTracker_CPU::~ITMDepthTracker_CPU(void) {}

int ITMDepthTracker_CPU::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) {
  Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);//三维坐标点
  Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);//法向量
  Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;//相机场景内参
  Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;//场景图像大小

  float *depth = viewHierarchyLevel->data->GetData(MEMORYDEVICE_CPU);//当前帧深度值
  Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;
  Vector2i viewImageSize = viewHierarchyLevel->data->noDims;//当前帧图像大小

  if (iterationType == TRACKER_ITERATION_NONE) return 0;

  bool shortIteration =
      (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

  float sumHessian[6 * 6], sumNabla[6], sumF;
  int noValidPoints;
  //构建一个上三角矩阵来依次填入海塞矩阵
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

      //判断该帧是否为有效点
      switch (iterationType) {
        case TRACKER_ITERATION_ROTATION:
          isValidPoint = computePerPointGH_Depth<true, true>(localNabla,
                                                             localHessian,
                                                             localF,
                                                             x,
                                                             y,
                                                             depth[x + y * viewImageSize.x],
                                                             viewImageSize,
                                                             viewIntrinsics,
                                                             sceneImageSize,
                                                             sceneIntrinsics,
                                                             approxInvPose,
                                                             scenePose,
                                                             pointsMap,
                                                             normalsMap,
                                                             distThresh[levelId]);
          break;
        case TRACKER_ITERATION_TRANSLATION:
          isValidPoint = computePerPointGH_Depth<true, false>(localNabla,
                                                              localHessian,
                                                              localF,
                                                              x,
                                                              y,
                                                              depth[x + y * viewImageSize.x],
                                                              viewImageSize,
                                                              viewIntrinsics,
                                                              sceneImageSize,
                                                              sceneIntrinsics,
                                                              approxInvPose,
                                                              scenePose,
                                                              pointsMap,
                                                              normalsMap,
                                                              distThresh[levelId]);
          break;
        case TRACKER_ITERATION_BOTH:
          isValidPoint = computePerPointGH_Depth<false, false>(localNabla,
                                                               localHessian,
                                                               localF,
                                                               x,
                                                               y,
                                                               depth[x + y * viewImageSize.x],
                                                               viewImageSize,
                                                               viewIntrinsics,
                                                               sceneImageSize,
                                                               sceneIntrinsics,
                                                               approxInvPose,
                                                               scenePose,
                                                               pointsMap,
                                                               normalsMap,
                                                               distThresh[levelId]);
          break;
        default: isValidPoint = false;
          break;
      }

      if (isValidPoint) {
        noValidPoints++;//TODO:该点有效，为什么无效点数目加一
        sumF += localF;//对每个点误差进行累加
        for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
        for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
      }
    }

  //填充海塞矩阵的下三角
  for (int r = 0, counter = 0; r < noPara; r++)
    for (int c = 0; c <= r; c++, counter++)
      hessian[r + c * 6] = sumHessian[counter];
  //填充海塞矩阵的上三角
  for (int r = 0; r < noPara; ++r)
    for (int c = r + 1; c < noPara; c++)
      hessian[r + c * 6] = hessian[c + r * 6];

  //填入雅可比矩阵
  memcpy(nabla, sumNabla, noPara * sizeof(float));
  f = (noValidPoints > 100) ? sumF / noValidPoints : 1e5f;//最终的误差函数

  return noValidPoints;
}
