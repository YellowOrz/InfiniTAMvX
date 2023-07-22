// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMTracker.h"
#include "../../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../../Objects/Tracking/ITMImageHierarchy.h"
#include "../../Objects/Tracking/ITMTemplatedHierarchyLevel.h"
#include "../../Objects/Tracking/ITMSceneHierarchyLevel.h"
#include "../../Objects/Tracking/TrackerIterationType.h"

#include "../../../ORUtils/HomkerMap.h"
#include "../../../ORUtils/SVMClassifier.h"

namespace ITMLib {
/** Base class for engine performing ICP based depth tracking.
    A typical example would be the original KinectFusion
    tracking algorithm.
*/
class ITMDepthTracker : public ITMTracker {
 private:
  const ITMLowLevelEngine *lowLevelEngine;
  ITMImageHierarchy<ITMSceneHierarchyLevel> *sceneHierarchy;
  ITMImageHierarchy<ITMTemplatedHierarchyLevel<ITMFloatImage> > *viewHierarchy;

  ITMTrackingState *trackingState;
  const ITMView *view;

  int *noIterationsPerLevel;  // 金字塔每一层的迭代次数

  float terminationThreshold;
  /**
   * @brief ???
   * 
   */
  void PrepareForEvaluation();
  /**
   * @brief ???
   * 
   * @param levelId 
   */
  void SetEvaluationParams(int levelId);
  /**
   * @brief 使用Cholesky求解增量方程，得到△T
   * 
   * @param[out] delta          △T，即当前帧 与 上一帧的相对变换矩阵
   * @param[in] nabla           nabla算子，维度6*1
   * @param[in] hessian         hessian矩阵，维度6*6
   * @param[in] shortIteration  是否只计算变换矩阵的一部分（旋转 or 平移）
   */
  void ComputeDelta(float *delta, float *nabla, float *hessian, bool shortIteration) const;
  /**
   * @brief 利用delta更新位姿
   * @details delta前3个数字对应rotation见论文《Linear Least-Squares Optimization for Point-to-Plane ICP Surface 
   *          Registration》的公式(6)
   * @param[in] para_old  老位姿
   * @param[in] delta     位姿增量，维度6*1，前3个是旋转向量，后3个是平移
   * @param[out] para_new 新位姿
   */
  void ApplyDelta(const Matrix4f &para_old, const float *delta, Matrix4f &para_new) const;
  /**
   * @brief 跟踪结果是否收敛，根据△T的模长判断
   * 
   * @param[in] step △T
   * @return true，收敛
   */
  bool HasConverged(float *step) const;

  /**
   * @brief 准备跟踪的数据：当前帧 和 场景 的 深度图、法向量、位姿、相机内参等
   * 
   * @param[in] trackingState   
   * @param[in] view            当前帧
   */
  void SetEvaluationData(ITMTrackingState *trackingState, const ITMView *view);
  /**
   * @brief 判别跟踪质量
   * 
   * @param[in] noValidPoints_old ICP过程中匹配到的像素点个数
   * @param[in] hessian_good      ICP完成后的Hessian矩阵
   * @param[in] f_old             ICP完成后的误差
   */
  void UpdatePoseQuality(int noValidPoints_old, float *hessian_good, float f_old);

  ORUtils::HomkerMap *map;               // ？？？
  ORUtils::SVMClassifier *svmClassifier; // 判别跟踪质量的分类器
  Vector4f mu, sigma;                    // ？？？

protected:
  float *distThresh;  // 金字塔每一层的距离阈值，用来筛选匹配

  int levelId;
  TrackerIterationType iterationType;

  Matrix4f scenePose;                          // scene位姿。是三维模型相对世界坐标系的位姿？？？
  ITMSceneHierarchyLevel *sceneHierarchyLevel; // scene的金字塔
  ITMTemplatedHierarchyLevel<ITMFloatImage> *viewHierarchyLevel; // 当前帧的金字塔
  /**
   * @brief 计算point-to-plane ICP的Hessian矩阵、Nabla算子 和 误差
   *
   * @param[out] f            误差
   * @param[out] nabla        Nabla算子
   * @param[out] hessian      Hessian矩阵
   * @param[in] approxInvPose 初始位姿，=上一帧的位姿
   * @return int              有效像素数
   */
  virtual int ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) = 0;

 public:
  /**
   * @brief 使用 高斯牛顿法 求解point-to-plane ICP，跟踪相机
   * 
   * @details 参考资料https://zhuanlan.zhihu.com/p/385414929
   * @param trackingState 
   * @param view 
   */
  void TrackCamera(ITMTrackingState *trackingState, const ITMView *view);
  
  bool requiresColourRendering() const { return false; }
  bool requiresDepthReliability() const { return false; }
  bool requiresPointCloudRendering() const { return true; }
  /**
   * @brief 
   * 
   * @param[in] numIterCoarse     金字塔最大一层的ICP迭代次数
   * @param[in] numIterFine       金字塔最小一层的ICP迭代次数
   * @param[in] distThreshCoarse  金字塔最大一层的距离误差
   * @param[in] distThreshFine    金字塔最小一层的距离误差
   */
  void SetupLevels(int numIterCoarse, int numIterFine, float distThreshCoarse, float distThreshFine);

  ITMDepthTracker(Vector2i imgSize, TrackerIterationType *trackingRegime, int noHierarchyLevels,
                  float terminationThreshold, float failureDetectorThreshold,
                  const ITMLowLevelEngine *lowLevelEngine, MemoryDeviceType memoryType);
  virtual ~ITMDepthTracker(void);
};
}
