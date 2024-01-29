// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMapGraphManager.h"

namespace ITMLib {
/** \brief
*/
class ITMActiveMapManager {
 public:
  // 子图的活跃类型
  typedef enum { 
    PRIMARY_LOCAL_MAP,  // 主子图
    NEW_LOCAL_MAP,      // 新建的。最多一个，放在所有子图的最后
    LOOP_CLOSURE,       // 回环的
    RELOCALISATION,     // 重定位的
    LOST,               // 跟丢的
    LOST_NEW            // 新建但跟丢的
  } LocalMapActivity;

 private:
  /** 单个活跃子图的相关信息（包含constraints） */
  struct ActiveDataDescriptor {
    int localMapIndex;                  // 子图的全局id
    LocalMapActivity type;              // 子图的活跃类型
    std::vector<Matrix4f> constraints;  // 与主子图的相对位姿。当前子图=>主子图
    ORUtils::SE3Pose estimatedPose;     // ??? 位姿，没用到？？？
    int trackingAttempts;               // 跟踪次数（无论成功与否，一帧一次）。
  };

  ITMMapGraphManager *localMapManager;          // 管理所有子图
  std::vector<ActiveDataDescriptor> activeData; // 所有活跃子图的信息 // ?一个子图可以被添加多次吗？
  /**
   * @brief 判断指定子图是否重定位成功
   * @param[in] dataID  指定子图的活跃id
   * @return int        0，重定位成功；-1，重定位失败；0，下次再试试看
   */
  int CheckSuccess_relocalisation(int dataID) const;
  /**
   * @brief 查看指定子图与主子图是否存在稳定的连接
   * @param[in] dataID        指定子图的活跃id
   * @param[in] primaryDataID 主子图的活跃id。可以为-1，即没有主子图吗？？？
   * @param[in] inliers       指定子图的约束中的inlier数量
   * @param[in] inlierPose    指定子图的约束中的inlier直接平均得到的位姿，当前子图=>主子图
   * @return int              0，重定位成功；-1，重定位失败；0，下次再试试看
   */
  int CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers, ORUtils::SE3Pose *inlierPose) const;
  /**
   * @brief 在两个子图之间添加link（即观测，也是加权后的位姿）
   * @param[in] fromData    子图1的全局id。// TODO: 应该叫id1
   * @param[in] toData      子图2的全局id。// TODO: 应该叫id2
   * @param[in] pose        toData到fromData的位姿
   * @param[in] weight      上面位姿的权重
   */
  void AcceptNewLink(int fromData, int toData, const ORUtils::SE3Pose &pose, int weight);
  /**
   * @brief 查看子图占用voxel block中可见的比例
   * @param[in] dataID 子图的活跃id
   * @return float      block的可见比例。
   * @note              只在前1000个voxel block中，计算可见的比例
   */
  float visibleOriginalBlocks(int dataID) const;
  /** 判断是否要新建子图。主子图中voxel block可见比例低于阈值 就新建 */
  bool shouldStartNewArea(void) const;
  /**
   * @brief 判断当前子图能否成为下一个主子图的唯一候选者
   * @details 主要看当前子图的可见比例是不是比别人都高
   * @param[in] newDataIdx      当前子图的活跃id
   * @param[in] bestDataIdx     现有候选子图的活跃id
   * @param[in] primaryDataIdx  现有主子图的活跃id
   * @return                    true，当前子图成为下一个主子图的唯一候选者
   */
  bool shouldMovePrimaryLocalMap(int newDataIdx, int bestDataIdx, int primaryDataIdx) const;

 public:
  /**
   * @brief 初始化活跃子图 = 新建子图 + 添加活跃信息
   * @param[in] isPrimaryLocalMap 是否是主子图
   * @return int                  新建子图的全局id
   */
  int initiateNewLocalMap(bool isPrimaryLocalMap = false);
  /**
   * @brief 已有的子图变成活跃子图（类型只能是 重定位 或 回环）。若之前添加过了，不会再次添加
   * @param[in] localMapId        已有的子图的全局id
   * @param[in] pose              约束位姿
   * @param[in] isRelocalisation  子图是否重定位
   * @return int                  子图的活跃id
   */
  int initiateNewLink(int localMapId, const ORUtils::SE3Pose &pose, bool isRelocalisation);
  /**
   * @brief 更新指定子图以及相关联子图的跟踪结果
   * @param[in] dataID                  指定子图的活跃id
   * @param[in] trackingResult          指定跟踪结果。0=failed，1=poor，2=good
   * @param[in] primaryTrackingSuccess  主子图是否跟踪good
   * @note 只要主子图跟踪不good，所有活跃子图都算是跟踪失败
   */
  void recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult, bool primaryTrackingSuccess);
  /** 查看活跃子图是否发生较大变化（即是否存在回环和重建）。return whether or not the local map graph has changed */
  bool maintainActiveData(void);  // TODO: 什么时候会发生变化？？？
  /** 获取主子图的活跃id */
  int findPrimaryDataIdx(void) const;
  /** 获取主子图的全局id */
  int findPrimaryLocalMapIdx(void) const;
  /** 找到用于可视化的子图（优先级依次是主子图、新建子图、约束最多的重定位子图），返回其活跃id */
  int findBestVisualisationDataIdx(void) const;
  /** 找到用于可视化的子图（优先级依次是主子图、新建子图、约束最多的重定位子图），返回其全局id */
  int findBestVisualisationLocalMapIdx(void) const;
  /** 获取活跃子图个数 */
  int numActiveLocalMaps(void) const { return static_cast<int>(activeData.size()); }
  /** 获取指定子图（输入活跃id）的全局子图id */
  int getLocalMapIndex(int dataIdx) const { return activeData[dataIdx].localMapIndex; }
  /** 获取指定子图（输入活跃id）的类型 */
  LocalMapActivity getLocalMapType(int dataIdx) const { return activeData[dataIdx].type; }

  ITMActiveMapManager(ITMMapGraphManager *localMapManager);
  ~ITMActiveMapManager(void) {}
};
}