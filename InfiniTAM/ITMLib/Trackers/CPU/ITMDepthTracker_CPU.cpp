// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMDepthTracker_CPU.h"
#include "../Shared/ITMDepthTracker_Shared.h"

using namespace ITMLib;

ITMDepthTracker_CPU::ITMDepthTracker_CPU(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                                         float terminationThreshold, float failureDetectorThreshold,
                                         const ITMLowLevelEngine *lowLevelEngine)
    : ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels, terminationThreshold, failureDetectorThreshold,
                      lowLevelEngine, MEMORYDEVICE_CPU) {}

ITMDepthTracker_CPU::~ITMDepthTracker_CPU(void) {}

int ITMDepthTracker_CPU::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) {
  //! 从scene中获取所需信息
  Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);   // 三维坐标点。第4个数字是w？？？
  Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU); // 法向量。第4个数字是w？？？
  Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;                        // 场景相机内参
  Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;                  // 场景图像大小
  //! 从当前帧获取所需信息
  float *depth = viewHierarchyLevel->data->GetData(MEMORYDEVICE_CPU); // 获取当前帧的深度图（以一维存储）
  Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;           // 当前帧相机内参
  Vector2i viewImageSize = viewHierarchyLevel->data->noDims;          // 当前帧图像大小

  if (iterationType == TRACKER_ITERATION_NONE) return 0;      // 应该放到这个函数的第一行？？？

  //! 参数准备
  bool shortIteration = (iterationType == TRACKER_ITERATION_ROTATION) ||  // 只有旋转or平移，则Hessian、Nabla只算一半
                        (iterationType == TRACKER_ITERATION_TRANSLATION);
  int noPara = shortIteration ? 3 : 6,
      noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
  float sumHessian[6 * 6], sumNabla[6], sumF= 0.0f;   // Hessian矩阵、Nabla算子 和 误差
  int noValidPoints = 0;                              //用于记录无意义的点
  memset(sumHessian, 0, sizeof(float) * noParaSQ);    //对sumHessian 清零
  memset(sumNabla, 0, sizeof(float) * noPara);        //对sumNabla 清零

  
  //! 计算每个像素的 误差、Hessian矩阵 和 Nabla算子
  for (int y = 0; y < viewImageSize.y; y++)
    for (int x = 0; x < viewImageSize.x; x++) {
      // 参数准备，针对当前像素
      float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;   // localHessian是上三角矩阵，只存非零？？？
      for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
      for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

      // 根据求解目标（旋转、平移 or 都要），计算单个像素的 Hessian矩阵、Nabla算子 和 误差
      bool isValidPoint;
      switch (iterationType) {
      case TRACKER_ITERATION_ROTATION: // 只求解 旋转
        isValidPoint = computePerPointGH_Depth<true, true>(
            localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize, viewIntrinsics,
            sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
        break;
      case TRACKER_ITERATION_TRANSLATION: // 只求解 平移
        isValidPoint = computePerPointGH_Depth<true, false>(
            localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize, viewIntrinsics,
            sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
        break;
      case TRACKER_ITERATION_BOTH: // 求解 旋转 & 平移
        isValidPoint = computePerPointGH_Depth<false, false>(
            localNabla, localHessian, localF, x, y, depth[x + y * viewImageSize.x], viewImageSize, viewIntrinsics,
            sceneImageSize, sceneIntrinsics, approxInvPose, scenePose, pointsMap, normalsMap, distThresh[levelId]);
        break;
      default:
        isValidPoint = false;
        break;
      }
      // 只累加有效的
      if (isValidPoint) { 
        noValidPoints++;  //TODO:该点有效，为什么无效点数目加一????
        sumF += localF;   //对每个点误差进行累加
        for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];
        for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];
      }
    }

  //! 获得最终的Hessian矩阵、Nabla算子 和 误差
  for (int r = 0, counter = 0; r < noPara; r++) // H的下三角
    for (int c = 0; c <= r; c++, counter++)
      hessian[r + c * 6] = sumHessian[counter]; // 改成hessian[r + c * 6] = hessian[c + r * 6] = sumHessian[counter]？？？
  for (int r = 0; r < noPara; ++r)              // H的上三角
    for (int c = r + 1; c < noPara; c++)
      hessian[r + c * 6] = hessian[c + r * 6];
  memcpy(nabla, sumNabla, noPara * sizeof(float));
  f = (noValidPoints > 100) ? sumF / noValidPoints : 1e5f;  //最终的误差函数。有效像素数量>100

  return noValidPoints;
}
