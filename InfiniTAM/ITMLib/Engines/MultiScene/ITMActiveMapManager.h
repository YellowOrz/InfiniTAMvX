// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMapGraphManager.h"

namespace ITMLib {
/** \brief
*/
class ITMActiveMapManager {
 public:
   typedef enum { 
    PRIMARY_LOCAL_MAP, 
    NEW_LOCAL_MAP, 
    LOOP_CLOSURE, 
    RELOCALISATION, 
    LOST, 
    LOST_NEW 
  } LocalMapActivity;

 private:
  /** 单个活跃子图的相关信息 */
  struct ActiveDataDescriptor {
    int localMapIndex;                  // ??? 全局子图id
    LocalMapActivity type;              // ??? 子图类型
    std::vector<Matrix4f> constraints;  // ??? 有关联的其他活跃子图的id
    ORUtils::SE3Pose estimatedPose;     // ??? 位姿
    int trackingAttempts;               // 跟踪帧数（无论成功与否）
  };

  ITMMapGraphManager *localMapManager;          // 管理所有子图
  std::vector<ActiveDataDescriptor> activeData; // 所有活跃子图的信息

  int CheckSuccess_relocalisation(int dataID) const;
  int CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ORUtils::SE3Pose *inlierPose) const;
  void AcceptNewLink(int dataId, int primaryDataId, const ORUtils::SE3Pose &pose, int weight);

  float visibleOriginalBlocks(int dataID) const;
  bool shouldStartNewArea(void) const;
  bool shouldMovePrimaryLocalMap(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;

 public:
  int initiateNewLocalMap(bool isPrimaryLocalMap = false);
  int initiateNewLink(int sceneID, const ORUtils::SE3Pose &pose, bool isRelocalisation);

  void recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess);

  /**
   * @brief 判断当前子图与主子图的位姿是否发生变化。return whether or not the local map graph has changed
   * @note 什么时候会发生变化？？？
   */
  bool maintainActiveData(void);

  int findPrimaryDataIdx(void) const;
  int findPrimaryLocalMapIdx(void) const;

  int findBestVisualisationDataIdx(void) const;
  int findBestVisualisationLocalMapIdx(void) const;

  int numActiveLocalMaps(void) const { return static_cast<int>(activeData.size()); }
  /**
   * @brief 获取指定活跃子图的全局子图id
   * @param[in] dataIdx   活跃子图id
   * @return int          全局子图id
   */
  int getLocalMapIndex(int dataIdx) const { return activeData[dataIdx].localMapIndex; }
  LocalMapActivity getLocalMapType(int dataIdx) const { return activeData[dataIdx].type; }

  ITMActiveMapManager(ITMMapGraphManager *localMapManager);
  ~ITMActiveMapManager(void) {}
};
}