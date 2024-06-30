// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../MiniSlamGraphLib/PoseGraph.h"
#include "ITMMapGraphManager.h"

namespace ITMLib {

/** This engine computes global pose adjustments using pose graph optimisation.
    The basic idea is that whenever some "main engine" considers it necessary,
    it should send new information to such a global adjustment step using the
    function updateMeasurements(). Should for whatever reason the engine reject
    these new measurements, this function should return false immediately, and
    the main engine has to keep resubmitting the same data again. On the other
    hand, retrieveNewEstimates() allows the main engine to retrieve the results
    from pose graph optimisation, if there are any, or false, if there aren't.

    The pose graph optimisation itself can be called explicitly using the method
    runGlobalAdjustment(). However, the whole class is also designed to be run
    in a separate thread in the background. The corresponding methods are
    startSeparateThread() and stopSeparateThread(), and whenever new
    measurements are being passed, a call to wakeupSeparateThread() is also
    recommended. The thread will reject new data while a pose graph optimisation
    is currently in progress, and it may go to sleep otherwise.
*/
class ITMGlobalAdjustmentEngine {
 private:
  struct PrivateData;                       // 并行线程中，用到的锁等变量 // TODO：为啥要再弄个结构体？

 public:
  ITMGlobalAdjustmentEngine(void);
  ~ITMGlobalAdjustmentEngine(void);

  bool hasNewEstimates(void) const;

  // Check whether pose graph optimisation has converged and produced a
  // new result. if it hasn't return false, otherwise copy them over
  bool retrieveNewEstimates(ITMMapGraphManager &dest);

  bool isBusyEstimating(void) const;

  /**
   * @brief 将所有子图的位姿和link信息添加到 位姿图中
   * Check whether thread is busy, if it is, return false, otherwise create a copy of all new measurements and make it busy
   * @param[in] src 所有子图的管理器
   * @return        是否更新成功。当优化进行时，更新失败
   */
  bool updateMeasurements(const ITMMapGraphManager &src);
  /**
   * @brief 进行一次全局优化
   * @param[in] blockingWait  当位姿图更新的时候是否等待。不等待的话直接返回false
   * @return                  是否优化成功。当位姿图为空 or 位姿图更新中但是不等待，优化失败    
   */
  bool runGlobalAdjustment(bool blockingWait = false);

  bool startSeparateThread(void);
  bool stopSeparateThread(void);
  void wakeupSeparateThread(void);

 private:
  void estimationThreadMain(void);
  /**
   * @brief 将所有子图的位姿和link信息添加到 位姿图中
   * @param[in] src   所有子图
   * @param[in] dest  位姿图
   */
  static void MultiSceneToPoseGraph(const ITMMapGraphManager &src, MiniSlamGraph::PoseGraph &dest);
  static void PoseGraphToMultiScene(const MiniSlamGraph::PoseGraph &src, ITMMapGraphManager &dest);

  MiniSlamGraph::PoseGraph *workingData;    // 待优化的位姿图
  MiniSlamGraph::PoseGraph *processedData;  // 优化好的位姿图

  PrivateData *privateData;                 // 并行线程以及其中用到的锁等变量
};
}
