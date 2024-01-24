// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMapGraphManager.h"

//#include <queue>

namespace ITMLib {
template <class TVoxel, class TIndex>
ITMVoxelMapGraphManager<TVoxel, TIndex>::ITMVoxelMapGraphManager(
    const ITMLibSettings *_settings, const ITMVisualisationEngine<TVoxel, TIndex> *_visualisationEngine,
    const ITMDenseMapper<TVoxel, TIndex> *_denseMapper, const Vector2i &_trackedImageSize)
    : settings(_settings), visualisationEngine(_visualisationEngine), denseMapper(_denseMapper),
      trackedImageSize(_trackedImageSize) {}

template<class TVoxel, class TIndex>
ITMVoxelMapGraphManager<TVoxel, TIndex>::~ITMVoxelMapGraphManager(void) {
  while (allData.size() > 0) {
    delete allData.back();
    allData.pop_back();
  }
}

template<class TVoxel, class TIndex>
int ITMVoxelMapGraphManager<TVoxel, TIndex>::createNewLocalMap(void) {
  int newIdx = (int) allData.size();
  allData.push_back(new ITMLocalMap<TVoxel, TIndex>(settings, visualisationEngine, trackedImageSize));

  denseMapper->ResetScene(allData[newIdx]->scene);
  return newIdx;
}
/**
 * @brief 删除子图（保证从别的子图的约束中也被删除了）
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] localMapId  子图的全局id
 */
template<class TVoxel, class TIndex>
void ITMVoxelMapGraphManager<TVoxel, TIndex>::removeLocalMap(int localMapId) {
  //! 检查有效性
  if ((localMapId < 0) || ((unsigned) localMapId >= allData.size())) return;

  //! 从别的子图中删除跟当前子图的约束。make sure there are no relations anywhere pointing to the local map
  const ConstraintList &l = getConstraints(localMapId);
  for (ConstraintList::const_iterator it = l.begin(); it != l.end(); ++it) eraseRelation(it->first, localMapId);

  //! 删除当前子图。delete the local map
  delete allData[localMapId];                   // 先释放指针指向的内存
  allData.erase(allData.begin() + localMapId);  // 再删除指针
}

template<class TVoxel, class TIndex>
ITMPoseConstraint &ITMVoxelMapGraphManager<TVoxel, TIndex>::getRelation(int fromLocalMap, int toLocalMap) {
  ConstraintList &m = getLocalMap(fromLocalMap)->relations;
  return m[toLocalMap];
}

static const ITMPoseConstraint invalidPoseConstraint;

/**
 * @brief 获取两个子图之间的约束关系（也叫做link）
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] fromLocalMap  第一个子图的全局id
 * @param[in] toLocalMap    第二个子图的全局id
 * @return const ITMPoseConstraint& 从第一个子图中找到与第二个子图的约束信息（即link）。其中的位姿是第二个子图=>第一个子图
 */
template <class TVoxel, class TIndex>
const ITMPoseConstraint &ITMVoxelMapGraphManager<TVoxel, TIndex>::getRelation_const(int fromLocalMap,
                                                                                    int toLocalMap) const {
  // 检查id有效性
  if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size()))
    return invalidPoseConstraint;
  
  const ConstraintList &m = getLocalMap(fromLocalMap)->relations; // 第一个子图的所有link
  // 从第一个子图的所有约束中找到跟第二个子图的link
  ConstraintList::const_iterator it = m.find(toLocalMap);         
  if (it == m.end())
    return invalidPoseConstraint; // 没找到
  return it->second;              // 找到了，返回link
}
/**
 * @brief 从fromLocalMap的约束中删除toLocalMap
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] fromLocalMap  子图的全局id
 * @param[in] toLocalMap    子图的全局id
 */
template<class TVoxel, class TIndex>
void ITMVoxelMapGraphManager<TVoxel, TIndex>::eraseRelation(int fromLocalMap, int toLocalMap) {
  if ((fromLocalMap < 0) || (fromLocalMap >= (int) allData.size())) return;

  std::map<int, ITMPoseConstraint> &m = getLocalMap(fromLocalMap)->relations;
  m.erase(toLocalMap);
}
/**
 * @brief 
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] localMapId  子图的全局id
 * @param[in] pose 
 * @return true 
 * @return false 
 */
template<class TVoxel, class TIndex>
bool ITMVoxelMapGraphManager<TVoxel, TIndex>::resetTracking(int localMapId, const ORUtils::SE3Pose &pose) {
  if ((localMapId < 0) || ((unsigned) localMapId >= allData.size())) return false;
  allData[localMapId]->trackingState->pose_d->SetFrom(&pose);
  allData[localMapId]->trackingState->age_pointCloud = -1;
  return true;
}
/**
 * @brief 获取指定子图所占用的voxel block数量
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] localMapId  子图的全局id
 * @return int            占用的voxel block数量
 */
template<class TVoxel, class TIndex>
int ITMVoxelMapGraphManager<TVoxel, TIndex>::getLocalMapSize(int localMapId) const {
  // 检查id有效性
  if ((localMapId < 0) || ((unsigned) localMapId >= allData.size())) return -1;

  ITMScene<TVoxel, TIndex> *scene = allData[localMapId]->scene;
  return scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
  // NOTE: getNumAllocatedVoxelBlocks是localVBA的总长度，lastFreeBlockId是localVBA中剩余空位数
}
/**
 * @brief 统计子图占用的voxel block中，指定id范围内可见的数量
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] localMapId  子图的全局id
 * @param[in] minBlockId  子图占用的voxel block的最小id
 * @param[in] maxBlockId  子图占用的voxel block的最大id
 * @param[in] invertIds   false，id范围是正数的；true，id范围是倒数的
 * @return int            voxel block 数量
 */
template <class TVoxel, class TIndex>
int ITMVoxelMapGraphManager<TVoxel, TIndex>::countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId,
                                                                bool invertIds) const {
  if ((localMapId < 0) || ((unsigned) localMapId >= allData.size())) return -1;   // 检查id有效性
  const ITMLocalMap<TVoxel, TIndex> *localMap = allData[localMapId];              // 子图

  if (invertIds) {  // 如果voxel block id是倒数的，转换成正数的
    int tmp = minBlockId;             // NOTE：getNumAllocatedVoxelBlocks表示localVBA的长度
    minBlockId = localMap->scene->index.getNumAllocatedVoxelBlocks() - maxBlockId - 1;
    maxBlockId = localMap->scene->index.getNumAllocatedVoxelBlocks() - tmp - 1;
  }

  return visualisationEngine->CountVisibleBlocks(localMap->scene, localMap->renderState, minBlockId, maxBlockId);
}

struct LinkPathComparison {
  bool operator()(const std::vector<int> &a, const std::vector<int> &b) { return a.size() > b.size(); }
};
/**
 * @brief 
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] fromLocalMapId 
 * @param[in] toLocalMapId 
 * @return ORUtils::SE3Pose 
 */
template<class TVoxel, class TIndex>
ORUtils::SE3Pose ITMVoxelMapGraphManager<TVoxel, TIndex>::findTransformation(int fromLocalMapId,
                                                                             int toLocalMapId) const {
  ORUtils::SE3Pose fromLocalMapPose, toLocalMapPose;
  if ((fromLocalMapId >= 0) || ((size_t) fromLocalMapId < allData.size()))
    fromLocalMapPose = allData[fromLocalMapId]->estimatedGlobalPose;
  if ((toLocalMapId >= 0) || ((size_t) toLocalMapId < allData.size()))
    toLocalMapPose = allData[toLocalMapId]->estimatedGlobalPose;
  return ORUtils::SE3Pose(toLocalMapPose.GetM() * fromLocalMapPose.GetInvM());
}
}
