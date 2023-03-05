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
  Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CPU);//齐次 三维坐标点
  Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CPU);//齐次 法向量
  Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;//场景相机内参
  Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;//场景图像大小

  float *depth = viewHierarchyLevel->data->GetData(MEMORYDEVICE_CPU); // 获取该视角的深度信息
  Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;  // 视角相机内参
  Vector2i viewImageSize = viewHierarchyLevel->data->noDims;  // 图像的大小 用像素来表示

  if (iterationType == TRACKER_ITERATION_NONE) return 0; //用于定义跟踪迭代机制的跟踪器迭代类型

  bool shortIteration =
      (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);
    //  ||或 有一个为true 则为true

  float sumHessian[6 * 6], sumNabla[6], sumF;
  /*
   * Hessian ：黑塞矩阵（Hessian Matrix），是一个多元函数的二阶偏导数构成的方阵，描述了函数的局部曲率。
   * Nabla ： Nabla算子
   */
  int noValidPoints;  //用于记录无意义的点
    //构建一个上三角矩阵来依次填入海塞矩阵
  int noPara = shortIteration ? 3 : 6, noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
   // int 类型=bool

  noValidPoints = 0;
  sumF = 0.0f;  //总的误差初始化为0
  memset(sumHessian, 0, sizeof(float) * noParaSQ);  //对sumHessian 清零
  memset(sumNabla, 0, sizeof(float) * noPara);  //对sumNabla 清零
  /*
   * memset 用于对结构体或者数组清零  memset(void *s, int c, size_t n);
   * memset() 函数用常量字节 c 填充 s 指向的内存区域的前 n 个字节。
   */


  for (int y = 0; y < viewImageSize.y; y++)
    for (int x = 0; x < viewImageSize.x; x++) {
      float localHessian[6 + 5 + 4 + 3 + 2 + 1], localNabla[6], localF = 0;
      // 遍历每一个像素
      for (int i = 0; i < noPara; i++) localNabla[i] = 0.0f;
      for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;

      bool isValidPoint;

      switch (iterationType) {  //根据跟踪器迭代类型 选择函数类型 //判断该帧是否为有效点
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
        default: isValidPoint = false;  //defualt 在上面都不执行时 执行一个任务
          break;
      }

      if (isValidPoint) {
        noValidPoints++;//TODO:该点有效，为什么无效点数目加一
        sumF += localF;//对每个点误差进行累加
        for (int i = 0; i < noPara; i++) sumNabla[i] += localNabla[i];// 累加
        for (int i = 0; i < noParaSQ; i++) sumHessian[i] += localHessian[i];// 累加
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
  /*
   * void *memcpy(void *dest, const void *src, size_t n);
   * 由src指向地址为起始地址的连续n个字节的数据复制到以destin指向地址为起始地址的空间内
   * dest 与 src 不一定需要是数组 是一定的连续内存空间即可 同时 size（src）<size(dest)否则内存溢出
   */
  f = (noValidPoints > 100) ? sumF / noValidPoints : 1e5f;//最终的误差函数

  return noValidPoints;
}
