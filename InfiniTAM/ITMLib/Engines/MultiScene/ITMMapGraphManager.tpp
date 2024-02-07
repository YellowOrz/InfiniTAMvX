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
  int newIdx = (int) allData.size();                // 新建子图的全局id
  allData.push_back(new ITMLocalMap<TVoxel, TIndex>(settings, visualisationEngine, trackedImageSize));  // 新建子图

  denseMapper->ResetScene(allData[newIdx]->scene);  // 初始化该子图的三维场景
  return newIdx;
}

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

static const ITMPoseConstraint invalidPoseConstraint; // NOTE：这个静态变量 用于函数返回引用的时候。

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

template<class TVoxel, class TIndex>
void ITMVoxelMapGraphManager<TVoxel, TIndex>::eraseRelation(int fromLocalMap, int toLocalMap) {
  if ((fromLocalMap < 0) || (fromLocalMap >= (int) allData.size())) return;

  std::map<int, ITMPoseConstraint> &m = getLocalMap(fromLocalMap)->relations;
  m.erase(toLocalMap);
}

template<class TVoxel, class TIndex>
bool ITMVoxelMapGraphManager<TVoxel, TIndex>::resetTracking(int localMapId, const ORUtils::SE3Pose &pose) {
  // 检查id有效性
  if ((localMapId < 0) || ((unsigned) localMapId >= allData.size())) return false;
  // 重置trackingState
  allData[localMapId]->trackingState->pose_d->SetFrom(&pose);   // 重置子图中的相机位姿，T_ls
  allData[localMapId]->trackingState->age_pointCloud = -1;      // 重置增量更新raycasting得到的点云的年龄
  return true;
}

template<class TVoxel, class TIndex>
int ITMVoxelMapGraphManager<TVoxel, TIndex>::getLocalMapSize(int localMapId) const {
  // 检查id有效性
  if ((localMapId < 0) || ((unsigned) localMapId >= allData.size())) return -1;

  ITMScene<TVoxel, TIndex> *scene = allData[localMapId]->scene;
  return scene->index.getNumAllocatedVoxelBlocks() - scene->localVBA.lastFreeBlockId - 1;
  // NOTE: getNumAllocatedVoxelBlocks是localVBA的总长度，lastFreeBlockId是localVBA中剩余空位数
}

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

template<class TVoxel, class TIndex>
ORUtils::SE3Pose ITMVoxelMapGraphManager<TVoxel, TIndex>::findTransformation(int fromLocalMapId,
                                                                             int toLocalMapId) const {
  // 获取两个子图的位姿
  ORUtils::SE3Pose fromLocalMapPose, toLocalMapPose;
  if ((fromLocalMapId >= 0) || ((size_t) fromLocalMapId < allData.size()))
    fromLocalMapPose = allData[fromLocalMapId]->estimatedGlobalPose;
  if ((toLocalMapId >= 0) || ((size_t) toLocalMapId < allData.size()))
    toLocalMapPose = allData[toLocalMapId]->estimatedGlobalPose;
  // 计算相对位姿，从 from子图 到 to子图，即 T_tf = T_tw * (T_fw)^-1
  return ORUtils::SE3Pose(toLocalMapPose.GetM() * fromLocalMapPose.GetInvM());
}
}
