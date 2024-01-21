// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../../Objects/Scene/ITMLocalMap.h"
#include "../../Core/ITMDenseMapper.h"
#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"

namespace ITMLib {
/** 管理子图的抽象类。用于提供接口，无需关注三维场景的类型。
 * This helpful abstract interface allows you to ignore the fact that scenes are templates. */
class ITMMapGraphManager {
 public:
  virtual ~ITMMapGraphManager(void) {}

  virtual int createNewLocalMap(void) = 0;
  virtual void removeLocalMap(int index) = 0;
  virtual size_t numLocalMaps(void) const = 0;
  /**
   * @brief 获取两个子图之间的约束关系（也叫做link）
   * @param[in] fromLocalMap  第一个子图的全局id
   * @param[in] toLocalMap    第二个子图的全局id
   * @return const ITMPoseConstraint& 从第一个子图中找到与第二个子图的约束信息（即link）。其中的位姿是第二个子图=>第一个子图
   */
  virtual const ITMPoseConstraint &getRelation_const(int fromLocalMap, int toLocalMap) const = 0;
  virtual ITMPoseConstraint &getRelation(int fromLocalMap, int toLocalMap) = 0;
  virtual void eraseRelation(int fromLocalMap, int toLocalMap) = 0;
  virtual const ConstraintList &getConstraints(int localMapId) const = 0;

  virtual void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose &pose) = 0;
  virtual const ORUtils::SE3Pose &getEstimatedGlobalPose(int localMapId) const = 0;

  virtual bool resetTracking(int localMapId, const ORUtils::SE3Pose &pose) = 0;

  virtual const ORUtils::SE3Pose *getTrackingPose(int localMapId) const = 0;
  virtual int getLocalMapSize(int localMapId) const = 0;
  virtual int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
};

/** 基于voxel hashing的子图管理器 */
template<class TVoxel, class TIndex>
class ITMVoxelMapGraphManager : public ITMMapGraphManager {
 private:
  const ITMLibSettings *settings;
  const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
  const ITMDenseMapper<TVoxel, TIndex> *denseMapper;
  Vector2i trackedImageSize;

  std::vector<ITMLocalMap<TVoxel, TIndex> *> allData; // 所有子图。包含 活跃和不活跃的

 public:
  ITMVoxelMapGraphManager(const ITMLibSettings *settings,
                          const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine,
                          const ITMDenseMapper<TVoxel, TIndex> *denseMapper,
                          const Vector2i &trackedImageSize);
  ~ITMVoxelMapGraphManager(void);

  int createNewLocalMap(void);
  void removeLocalMap(int index);
  size_t numLocalMaps(void) const { return allData.size(); }
  /** [const版本] 根据全局id获取子图 */
  const ITMLocalMap<TVoxel, TIndex> *getLocalMap(int localMapId) const { return allData[localMapId]; }
  /** 根据全局id获取子图 */
  ITMLocalMap<TVoxel, TIndex> *getLocalMap(int localMapId) { return allData[localMapId]; }

  const ITMPoseConstraint &getRelation_const(int fromLocalMap, int toLocalMap) const;
  ITMPoseConstraint &getRelation(int fromLocalMap, int toLocalMap);
  void eraseRelation(int fromLocalMap, int toLocalMap);
  const ConstraintList &getConstraints(int localMapId) const { return allData[localMapId]->relations; }

  void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose &pose) {
    allData[localMapId]->estimatedGlobalPose = pose;
  }
  const ORUtils::SE3Pose &getEstimatedGlobalPose(int localMapId) const { return allData[localMapId]->estimatedGlobalPose; }

  bool resetTracking(int localMapId, const ORUtils::SE3Pose &pose);
  /**
   * @brief 获取指定子图的位姿。world to local
   * @param[in] localMapId 指定子图的全局id
   * @return const ORUtils::SE3Pose* 
   */
  const ORUtils::SE3Pose *getTrackingPose(int localMapId) const {
    return getLocalMap(localMapId)->trackingState->pose_d;
  }

  int getLocalMapSize(int localMapId) const;
  int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const;

  ORUtils::SE3Pose findTransformation(int fromlocalMapId, int tolocalMapId) const;
};
}