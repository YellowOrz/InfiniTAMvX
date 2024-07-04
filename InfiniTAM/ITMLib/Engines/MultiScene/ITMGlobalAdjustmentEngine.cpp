// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMGlobalAdjustmentEngine.h"

#include "../../../MiniSlamGraphLib/GraphNodeSE3.h"
#include "../../../MiniSlamGraphLib/GraphEdgeSE3.h"
#include "../../../MiniSlamGraphLib/SlamGraphErrorFunction.h"
#include "../../../MiniSlamGraphLib/LevenbergMarquardtMethod.h"

#ifndef NO_CPP11  // ∵c++11才开始支持mutex
#include <mutex>
#include <thread>
#include <condition_variable>
#endif

using namespace ITMLib;
struct ITMGlobalAdjustmentEngine::PrivateData {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  PrivateData(void) {
    stopThread = false;
    wakeupSent = false;
  }
  std::mutex workingData_mutex;     // 待优化的位姿图的锁
  std::mutex processedData_mutex;   // 优化好的位姿图的锁
  std::thread processingThread;     // 并行线程。跑estimationThreadMain
  bool stopThread;                  // 并行线程是否要停止

  std::mutex wakeupMutex;
  std::condition_variable wakeupCond;
  bool wakeupSent;
#endif
};

ITMGlobalAdjustmentEngine::ITMGlobalAdjustmentEngine(void) {
  privateData = new PrivateData();
  workingData = NULL;
  processedData = NULL;
}

ITMGlobalAdjustmentEngine::~ITMGlobalAdjustmentEngine(void) {
  stopSeparateThread();
  if (workingData != NULL) delete workingData;
  if (processedData != NULL) delete processedData;
  delete privateData;
}

bool ITMGlobalAdjustmentEngine::hasNewEstimates(void) const {
  return (processedData != NULL);
}

bool ITMGlobalAdjustmentEngine::retrieveNewEstimates(ITMMapGraphManager &dest) {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  if (processedData == NULL) return false;

  privateData->processedData_mutex.lock();
  PoseGraphToMultiScene(*processedData, dest);
  delete processedData;
  processedData = NULL;
  privateData->processedData_mutex.unlock();
#endif
  return true;
}

bool ITMGlobalAdjustmentEngine::isBusyEstimating(void) const {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex  
  // if someone else is currently using the mutex (most likely the
  // consumer thread), we consider the global adjustment engine to
  // be busy
  if (!privateData->workingData_mutex.try_lock()) return true;

  privateData->workingData_mutex.unlock();
#endif
  return false;
}

bool ITMGlobalAdjustmentEngine::updateMeasurements(const ITMMapGraphManager &src) {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  // 正在优化的时候，不能更新位姿图 busy, can't accept new measurements at the moment
  if (!privateData->workingData_mutex.try_lock()) return false;
  // 初始化 待优化的位姿图
  if (workingData == NULL) workingData = new MiniSlamGraph::PoseGraph;
  // 将所有子图的位姿和link信息添加到 位姿图中
  MultiSceneToPoseGraph(src, *workingData);
  privateData->workingData_mutex.unlock();
#endif
  return true;
}

bool ITMGlobalAdjustmentEngine::runGlobalAdjustment(bool blockingWait) {  // TODO：下次从这儿开始
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  //! 保证位姿图不为空。first make sure there is new data and we have exclusive access to it
  if (workingData == NULL) return false;
  //! 如果位姿图正在更新，是否等待
  if (blockingWait) privateData->workingData_mutex.lock();
  else if (!privateData->workingData_mutex.try_lock()) return false;
  //! 进行全局优化。now run the actual global adjustment
  workingData->prepareEvaluations();                                     // 记录变量维度
  MiniSlamGraph::SlamGraphErrorFunction errf(*workingData);              // 准备误差函数
  MiniSlamGraph::SlamGraphErrorFunction::Parameters para(*workingData);  // 准备优化参数，其实就是位姿图的节点
  MiniSlamGraph::LevenbergMarquardtMethod::minimize(errf, para);         // 使用LM方法进行优化
  workingData->setNodeIndex(para.getNodes());                            // 将图中的节点替换成优化好的
  //! 拷贝位姿图，为下一次做准备。copy data to output buffer
  // ??? 不直接更新到processedData是因为扫描还在进行，会访问位姿图（比如添加新的节点）
  privateData->processedData_mutex.lock();
  if (processedData != NULL) delete processedData;
  processedData = workingData;
  workingData = NULL;
  privateData->processedData_mutex.unlock();

  privateData->workingData_mutex.unlock();
#endif
  return true;
}

bool ITMGlobalAdjustmentEngine::startSeparateThread(void) {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  if (privateData->processingThread.joinable()) return false;

  privateData->processingThread = std::thread(&ITMGlobalAdjustmentEngine::estimationThreadMain, this);
#endif
  return true;
}

bool ITMGlobalAdjustmentEngine::stopSeparateThread(void) {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  if (!privateData->processingThread.joinable()) return false;

  privateData->stopThread = true;
  wakeupSeparateThread();
  privateData->processingThread.join();
#endif
  return true;
}

void ITMGlobalAdjustmentEngine::estimationThreadMain(void) {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  while (!privateData->stopThread) {
    runGlobalAdjustment(true);
    std::unique_lock<std::mutex> lck(privateData->wakeupMutex);
    if (!privateData->wakeupSent) privateData->wakeupCond.wait(lck);
    privateData->wakeupSent = false;
  }
#endif
}

void ITMGlobalAdjustmentEngine::wakeupSeparateThread(void) {
#ifndef NO_CPP11  // ∵c++11才开始支持mutex
  std::unique_lock<std::mutex> lck(privateData->wakeupMutex);
  privateData->wakeupSent = true;
  privateData->wakeupCond.notify_all();
#endif
}

void ITMGlobalAdjustmentEngine::MultiSceneToPoseGraph(const ITMMapGraphManager &src, MiniSlamGraph::PoseGraph &dest) {
  //! 把每个子图的位姿作为图的node
  // NOTE：因为是添加到待优化的位姿图中，所以不用清空位姿图
  for (int localMapId = 0; localMapId < (int) src.numLocalMaps(); ++localMapId) {
    MiniSlamGraph::GraphNodeSE3 *pose = new MiniSlamGraph::GraphNodeSE3();

    pose->setId(localMapId);                                // node包含子图的全局id
    pose->setPose(src.getEstimatedGlobalPose(localMapId));  // node包含子图的位姿，即世界坐标系到子图，T_sw
    if (localMapId == 0) pose->setFixed(true);              // 最开始的子图固定，不进行优化

    dest.addNode(pose);
  }
  //! 把每个子图 与其他子图的link作为图的edge
  for (int localMapId = 0; localMapId < (int)src.numLocalMaps(); ++localMapId) {
    const ConstraintList &constraints = src.getConstraints(localMapId);
    for (ConstraintList::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
      MiniSlamGraph::GraphEdgeSE3 *odometry = new MiniSlamGraph::GraphEdgeSE3();

      odometry->setFromNodeId(localMapId);                                  // 设置起点对应子图的全局id
      odometry->setToNodeId(it->first);                                     // 设置终点对应子图的全局id
      odometry->setMeasurementSE3(it->second.GetAccumulatedObservations()); // 获取终点子图 到 起点子图的位姿（加权平均）

      //TODO odometry->setInformation
      dest.addEdge(odometry);
    }
  }
}

void ITMGlobalAdjustmentEngine::PoseGraphToMultiScene(const MiniSlamGraph::PoseGraph &src, ITMMapGraphManager &dest) {
  for (int localMapId = 0; localMapId < (int) dest.numLocalMaps(); ++localMapId) {
    MiniSlamGraph::SlamGraph::NodeIndex::const_iterator it = src.getNodeIndex().find(localMapId);
    if (it == src.getNodeIndex().end()) continue;
    const MiniSlamGraph::GraphNodeSE3 *pose = (const MiniSlamGraph::GraphNodeSE3 *) it->second;
    ORUtils::SE3Pose outpose = pose->getPose();
    dest.setEstimatedGlobalPose(localMapId, outpose);
  }
}

