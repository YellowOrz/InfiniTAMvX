// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../../Core/ITMDenseMapper.h"
#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../../Objects/Scene/ITMLocalMap.h"

namespace ITMLib {
/** 管理子图的抽象类。用于提供接口，无需关注三维场景的类型。
 * This helpful abstract interface allows you to ignore the fact that scenes are templates. */
class ITMMapGraphManager {
public:
  virtual ~ITMMapGraphManager(void) {}
  /**
   * @brief 新建子图（放在最后）&& 初始化三维场景
   * @return int 新建子图的全局id
   */
  virtual int createNewLocalMap(void) = 0;
  /**
   * @brief 删除子图（保证从别的子图的约束中也被删除了）
   * @param[in] localMapId  子图的全局id
   */
  virtual void removeLocalMap(int index) = 0;
  /** 获取子图的总数 */
  virtual size_t numLocalMaps(void) const = 0;
  /**
   * @brief 获取两个子图之间的约束关系（也叫做link）
   * @param[in] fromLocalMap  第一个子图的全局id
   * @param[in] toLocalMap    第二个子图的全局id
   * @return const ITMPoseConstraint&
   * 从第一个子图中找到与第二个子图的约束信息（即link）。其中的位姿是第二个子图=>第一个子图
   */
  virtual const ITMPoseConstraint &getRelation_const(int fromLocalMap, int toLocalMap) const = 0;
  /**
   * @brief 获取两个子图之间的约束关系（也叫做link）
   * @param[in] fromLocalMap  第一个子图的全局id
   * @param[in] toLocalMap    第二个子图的全局id
   * @return const ITMPoseConstraint&
   * 从第一个子图中找到与第二个子图的约束信息（即link）。其中的位姿是第二个子图=>第一个子图
   */
  virtual ITMPoseConstraint &getRelation(int fromLocalMap, int toLocalMap) = 0;
  /**
   * @brief 从fromLocalMap的约束中删除toLocalMap
   * @param[in] fromLocalMap  子图的全局id
   * @param[in] toLocalMap    子图的全局id
   */
  virtual void eraseRelation(int fromLocalMap, int toLocalMap) = 0;
  /** 获取指定子图（输入全局id）的所有link */
  virtual const ConstraintList &getConstraints(int localMapId) const = 0;
  /** 设置指定子图（输入全局id）的位姿，即世界坐标系到子图，T_sw */
  virtual void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose &pose) = 0;
  /** 获取指定子图（输入全局id）的位姿，即世界坐标系到子图，T_sw */
  virtual const ORUtils::SE3Pose &getEstimatedGlobalPose(int localMapId) const = 0;
  /**
   * @brief 将指定子图的跟踪位姿（trackingState）设置为指定位姿
   * @param[in] localMapId  子图的全局id
   * @param[in] pose        指定位姿
   * @return true           成功
   * @return false          失败
   */
  virtual bool resetTracking(int localMapId, const ORUtils::SE3Pose &pose) = 0;
  /**
   * @brief 获取指定子图中相机的位姿（子图坐标系到当前帧），T_ls
   * @param[in] localMapId 指定子图的全局id
   * @return const ORUtils::SE3Pose*
   */
  virtual const ORUtils::SE3Pose *getTrackingPose(int localMapId) const = 0;
  /**
   * @brief 获取指定子图所占用的voxel block数量
   * @param[in] localMapId  子图的全局id
   * @return int            占用的voxel block数量
   */
  virtual int getLocalMapSize(int localMapId) const = 0;
  /**
   * @brief 统计子图占用的voxel block中，指定id范围内可见的数量
   * @param[in] localMapId  子图的全局id
   * @param[in] minBlockId  子图占用的voxel block的最小id
   * @param[in] maxBlockId  子图占用的voxel block的最大id
   * @param[in] invertIds   false，id范围是正数的；true，id范围是倒数的
   * @return int            voxel block 数量
   */
  virtual int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
};

/* ------------------------------------------------------------------------------------------------------------------ */
/*                                            基于voxel hashing的子图管理器                                           */
/* ------------------------------------------------------------------------------------------------------------------ */
template <class TVoxel, class TIndex> class ITMVoxelMapGraphManager : public ITMMapGraphManager {
private:
  const ITMLibSettings *settings;
  const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
  const ITMDenseMapper<TVoxel, TIndex> *denseMapper;                  // 负责 场景三维模型的融合 && swap in/out
  Vector2i trackedImageSize;

  std::vector<ITMLocalMap<TVoxel, TIndex> *> allData; // 所有子图。包含 活跃和不活跃的

public:
  ITMVoxelMapGraphManager(const ITMLibSettings *settings,
                          const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine,
                          const ITMDenseMapper<TVoxel, TIndex> *denseMapper, const Vector2i &trackedImageSize);
  ~ITMVoxelMapGraphManager(void);
  /**
   * @brief 新建子图（放在最后）&& 初始化三维场景
   * @return int 新建子图的全局id
   */
  int createNewLocalMap(void);
  /**
   * @brief 删除子图（保证从别的子图的约束中也被删除了）
   * @param[in] localMapId  子图的全局id
   */
  void removeLocalMap(int index);
  /** 获取子图的总数 */
  size_t numLocalMaps(void) const { return allData.size(); }
  /** [const版本] 根据全局id获取子图 */
  const ITMLocalMap<TVoxel, TIndex> *getLocalMap(int localMapId) const { return allData[localMapId]; }
  /** 根据全局id获取子图 */
  ITMLocalMap<TVoxel, TIndex> *getLocalMap(int localMapId) { return allData[localMapId]; }
  /**
   * @brief 获取两个子图之间的约束关系（也叫做link）
   * @param[in] fromLocalMap  第一个子图的全局id
   * @param[in] toLocalMap    第二个子图的全局id
   * @return const ITMPoseConstraint& 从第一个子图中找到与第二个子图的约束信息（即link）。其中的位姿是第二个子图=>第一个子图
   */
  const ITMPoseConstraint &getRelation_const(int fromLocalMap, int toLocalMap) const;
  /**
   * @brief 获取两个子图之间的约束关系（也叫做link）
   * @param[in] fromLocalMap  第一个子图的全局id
   * @param[in] toLocalMap    第二个子图的全局id
   * @return const ITMPoseConstraint&
   * 从第一个子图中找到与第二个子图的约束信息（即link）。其中的位姿是第二个子图=>第一个子图
   */
  ITMPoseConstraint &getRelation(int fromLocalMap, int toLocalMap);
  /**
   * @brief 从fromLocalMap的约束中删除toLocalMap
   * @param[in] fromLocalMap  子图的全局id
   * @param[in] toLocalMap    子图的全局id
   */
  void eraseRelation(int fromLocalMap, int toLocalMap);
  /** 获取指定子图（输入全局id）的所有link */
  const ConstraintList &getConstraints(int localMapId) const { return allData[localMapId]->relations; }
  /** 设置指定子图（输入全局id）的位姿，即世界坐标系到子图，T_sw */
  void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose &pose) {
    allData[localMapId]->estimatedGlobalPose = pose;
  }
  /** 获取指定子图（输入全局id）的位姿，即世界坐标系到子图，T_sw */
  const ORUtils::SE3Pose &getEstimatedGlobalPose(int localMapId) const {
    return allData[localMapId]->estimatedGlobalPose;
  }
  /**
   * @brief 将指定子图的跟踪位姿（trackingState）设置为指定位姿
   * @param[in] localMapId  子图的全局id
   * @param[in] pose        指定位姿
   * @return true           成功
   * @return false          失败
   */
  bool resetTracking(int localMapId, const ORUtils::SE3Pose &pose);
  /**
   * @brief 获取指定子图中相机的位姿（子图坐标系到当前帧），T_ls
   * @param[in] localMapId 指定子图的全局id
   * @return const ORUtils::SE3Pose*
   */
  const ORUtils::SE3Pose *getTrackingPose(int localMapId) const {
    return getLocalMap(localMapId)->trackingState->pose_d;
  }
  /**
   * @brief 获取指定子图所占用的voxel block数量
   * @param[in] localMapId  子图的全局id
   * @return int            占用的voxel block数量
   */
  int getLocalMapSize(int localMapId) const;
  /**
   * @brief 统计子图占用的voxel block中，指定id范围内可见的数量
   * @param[in] localMapId  子图的全局id
   * @param[in] minBlockId  子图占用的voxel block的最小id
   * @param[in] maxBlockId  子图占用的voxel block的最大id
   * @param[in] invertIds   false，id范围是正数的；true，id范围是倒数的
   * @return int            voxel block 数量
   */
  int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const;
  /**
   * @brief 获取两个子图之间的相对位姿
   * @param[in] fromLocalMapId  一个子图的全局id
   * @param[in] toLocalMapId    另一个子图的全局id
   * @return ORUtils::SE3Pose 从 from子图 到 to子图的位姿，即 T_to_from = T_to_world * (T_from_world)^-1
   */
  ORUtils::SE3Pose findTransformation(int fromlocalMapId, int tolocalMapId) const;
};
} // namespace ITMLib