// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMActiveMapManager.h"

using namespace ITMLib;
// TODO：为啥要在这里设置一堆静态变量，而不是添加到类里面，或者头文件里面
// try loop closures for this number of frames
static const int N_linktrials = 20;
// at least these many frames have to be tracked successfully
static const int N_linkoverlap = 10;
// 固定n帧指定重定位。try relocalisations for this number of frames
static const int N_reloctrials = 20;
// 有关联的子图数量超过这个阈值，算重定位成功。at least these many tracking attempts have to succeed for relocalisation
static const int N_relocsuccess = 10;
// 查看子图中voxel block的可见性时，只查看前1000个。
// When checking "overlap with original local map", find how many of the first N blocks are still visible
static const int N_originalblocks = 1000;
static const float F_originalBlocksThreshold = 0.2f; //0.4f

ITMActiveMapManager::ITMActiveMapManager(ITMMapGraphManager *_localMapManager) {
  localMapManager = _localMapManager;
}

int ITMActiveMapManager::initiateNewLocalMap(bool isPrimaryLocalMap) {
  int newIdx = localMapManager->createNewLocalMap();

  ActiveDataDescriptor newLink;
  newLink.localMapIndex = newIdx;
  newLink.type = isPrimaryLocalMap ? PRIMARY_LOCAL_MAP : NEW_LOCAL_MAP;
  newLink.trackingAttempts = 0;
  activeData.push_back(newLink);

  return newIdx;
}

int ITMActiveMapManager::initiateNewLink(int localMapId, const ORUtils::SE3Pose &pose, bool isRelocalisation) {
  static const bool ensureUniqueLinks = true;

  // make sure only one relocalisation per local map is attempted at a time
  if (ensureUniqueLinks) {
    for (size_t i = 0; i < activeData.size(); ++i) {
      if (activeData[i].localMapIndex == localMapId) return -1;
    }
  }

  if (!localMapManager->resetTracking(localMapId, pose)) return -1;

  ActiveDataDescriptor newLink;
  newLink.localMapIndex = localMapId;
  newLink.type = isRelocalisation ? RELOCALISATION : LOOP_CLOSURE;
  newLink.trackingAttempts = 0;
  activeData.push_back(newLink);

  return (int) activeData.size() - 1;
}

float ITMActiveMapManager::visibleOriginalBlocks(int dataID) const {
  int localMapId = activeData[dataID].localMapIndex;              // 子图的全局id

  int allocated = localMapManager->getLocalMapSize(localMapId);   // 子图占用的voxel block数量
  int counted = localMapManager->countVisibleBlocks(localMapId, 0, N_originalblocks, true); // 前1000个block中可见数量

  // int tmp = N_originalblocks;
  // if (allocated < tmp) tmp = allocated;
  // return (float) counted / (float) tmp; 
  return (float)counted / (float)std::min(N_originalblocks, allocated);   // 计算可见的比例   // TODO: 确定这么修改没有问题
}

bool ITMActiveMapManager::shouldStartNewArea(void) const {
  int primaryLocalMapIdx = -1;
  int primaryDataIdx = -1;

  // don't start two new local maps at a time
  for (int i = 0; i < (int) activeData.size(); ++i) {
    if (activeData[i].type == NEW_LOCAL_MAP) return false;
    if (activeData[i].type == PRIMARY_LOCAL_MAP) {
      primaryDataIdx = i;
      primaryLocalMapIdx = activeData[i].localMapIndex;
    }
  }

  // TODO: check: if relocalisation fails for some time, start new local map
  if (primaryLocalMapIdx < 0) return false;
  else return visibleOriginalBlocks(primaryDataIdx) < F_originalBlocksThreshold;

  return false;
}

bool ITMActiveMapManager::shouldMovePrimaryLocalMap(int newDataId, int bestDataId, int primaryDataId) const {
  int localMapIdx_primary = primaryDataId >= 0 ? activeData[primaryDataId].localMapIndex : -1;  // 现有主子图的全局id
  int localMapIdx_best = bestDataId >= 0 ? activeData[bestDataId].localMapIndex : -1;           // 候选子图的全局id
  int localMapIdx_new = newDataId >= 0 ? activeData[newDataId].localMapIndex : -1;              // 当前子图的全局id

  int blocksInUse_primary = -1;         // 主子图占用的voxel block数量
  float visibleRatio_primary = 1.0f;    // 主子图占用的voxel block中可见的比例  // TODO: 初始化成0更好吧
  int blocksInUse_best = -1;            // 候选子图占用的voxel block数量
  float visibleRatio_best = 1.0f;       // 候选子图占用的voxel block中可见的比例
  bool isNewLocalMap_best = false;      // 候选子图是否是新建的
  int blocksInUse_new = -1;             // 当前子图占用的voxel block数量
  float visibleRatio_new = 1.0f;        // 当前子图占用的voxel block中可见的比例
  bool isNewLocalMap_new = false;       // 当前子图是否是新建的

  //! 统计每个子图中的voxel block。count blocks in all relevant localMaps
  if (localMapIdx_new >= 0) {       // 当前子图
    isNewLocalMap_new = (activeData[newDataId].type == NEW_LOCAL_MAP);
    blocksInUse_new = localMapManager->getLocalMapSize(localMapIdx_new);
    if (blocksInUse_new < 0) return false;    // TODO: 怎么可能占用voxel block数量<0？除非当前子图的全局id非法
    visibleRatio_new = visibleOriginalBlocks(newDataId);
  }

  if (localMapIdx_primary >= 0) {   // 主子图
    blocksInUse_primary = localMapManager->getLocalMapSize(localMapIdx_primary);
    visibleRatio_primary = visibleOriginalBlocks(primaryDataId);
  }

  if (localMapIdx_best >= 0) {      // 候选子图
    isNewLocalMap_best = (activeData[bestDataId].type == NEW_LOCAL_MAP);
    blocksInUse_best = localMapManager->getLocalMapSize(localMapIdx_best);
    visibleRatio_best = visibleOriginalBlocks(bestDataId);
  }

  //! 判断 是否要把 当前子图 替换 主子图
  if (blocksInUse_primary < 0)                  // 主子图还没用voxel block（那它怎么成为主子图的？！），替换
    // TODO: if relocalisation fails, a new local map gets started, and is eventually accepted, this case will get relevant
    return true;

  // step 1: is "new" better than "primary" ?

  // don't continue a local map that is already full
/* 	if (blocksInUse_new >= N_maxblocknum) return false;

	if (blocksInUse_new >= blocksInUse_primary) return false; */
  if (visibleRatio_new <= visibleRatio_primary)   // 当前子图的voxel block可见比例 < 主子图，不替换
    return false;

  // step 2: is there any contender for a new local map to move to?
  if (blocksInUse_best < 0)                       // 候选子图还没用voxel block（那它怎么成为候选子图的？！），替换
    return true;

  // if this is a new local map, but we previously found that we can loop close, don't accept the new local map!
  if (isNewLocalMap_new && !isNewLocalMap_best)   // 当前子图是新的，但候选子图可以用于回环（为啥？？？），不替换
    return false;
  // if this is a loop closure and we have not found any alternative loop closure before, accept the new one!
  if (!isNewLocalMap_new && isNewLocalMap_best)   // 存在回环，但是没有其他回环选项，替换  // ?只要不是NEW_LOCAL_MAP都是回环
    return true;

  // if the two are equal, take the smaller one
  // return (blocksInUse_new < blocksInUse_best);
  return (visibleRatio_new > visibleRatio_best);  // 当前子图的voxel block可见比例 > 候选子图，替换
}

int ITMActiveMapManager::findPrimaryDataIdx(void) const {
  for (int i = 0; i < (int) activeData.size(); ++i) // 主子图肯定是活跃子图
    if (activeData[i].type == PRIMARY_LOCAL_MAP) return i;

  return -1;
}

int ITMActiveMapManager::findPrimaryLocalMapIdx(void) const {
  // TODO: 直接跟findPrimaryDataIdx一样遍历activeData的不就好了吗
  int id = findPrimaryDataIdx();
  if (id < 0) return -1;
  return activeData[id].localMapIndex;
}

int ITMActiveMapManager::findBestVisualisationDataIdx(void) const {
  int bestIdx = -1;
  for (int i = 0; i < static_cast<int>(activeData.size()); ++i) {
    if (activeData[i].type == PRIMARY_LOCAL_MAP) return i;
    else if (activeData[i].type == NEW_LOCAL_MAP) bestIdx = i;
    else if (activeData[i].type == RELOCALISATION) {
      if (bestIdx < 0) {
        bestIdx = i;
        continue;
      }
      if (activeData[bestIdx].type == NEW_LOCAL_MAP) continue;
      if (activeData[bestIdx].constraints.size() < activeData[i].constraints.size()) bestIdx = i;
    }
  }
  return bestIdx;
}

int ITMActiveMapManager::findBestVisualisationLocalMapIdx(void) const {
  int id = findBestVisualisationDataIdx();
  if (id < 0) return -1;
  return activeData[id].localMapIndex;
}

void ITMActiveMapManager::recordTrackingResult(int dataID, ITMTrackingState::TrackingResult trackingResult,
                                               bool primaryTrackingSuccess) {
  ActiveDataDescriptor &data = activeData[dataID];

  int primaryLocalMapID = findPrimaryLocalMapIdx();   // 主子图的全局id
  int localMapId = data.localMapIndex;                // 当前子图的全局id
  data.trackingAttempts++;

  if (trackingResult == ITMTrackingState::TRACKING_GOOD) {            //! 跟踪good
    if (data.type == RELOCALISATION)        // 重定位的话，位姿是当前子图=>主子图（因为重定位是以主子图为世界坐标系？），直接记录
      data.constraints.push_back(localMapManager->getTrackingPose(dataID)->GetM()); // ?为啥根据活跃id到全局子图中去找？
    else if (primaryTrackingSuccess &&      // 新建子图 or 回环的话，位姿是世界坐标系=>当前子图，要转换成 当前子图=>主子图
             ((data.type == NEW_LOCAL_MAP) || (data.type == LOOP_CLOSURE))) {
      Matrix4f Tnew_inv = localMapManager->getTrackingPose(localMapId)->GetInvM();  // 当前子图 to 世界坐标系= 的位姿
      Matrix4f Told = localMapManager->getTrackingPose(primaryLocalMapID)->GetM();  // 世界坐标系 to 主子图  的位姿
      Matrix4f Told_to_new = Tnew_inv * Told;                                       // 当前子图 to 主子图 的位姿

      data.constraints.push_back(Told_to_new);
    }
  } else if (trackingResult == ITMTrackingState::TRACKING_FAILED) {   //! 跟踪失败
    if (data.type == PRIMARY_LOCAL_MAP) {   // 主子图的话，所有活跃的子图都设置type为lost
      for (size_t j = 0; j < activeData.size(); ++j) {
        if (activeData[j].type == NEW_LOCAL_MAP)      // 新建的子图要单独对待，因为一次性只能有一个
          activeData[j].type = LOST_NEW;
        else
          activeData[j].type = LOST;
      }
    }
  }                                                                   // NOTE: 除了主子图，跟踪为poor都算是fail
}

/**
 * @brief 使用huber函数计算新的权重
 * @param[in] residual  权重残差
 * @param[in] b         阈值。<b的权重都设为1
 * @return float        新的权重 = 1                       r<b
 *                              = sqrt(2b|r| - b^2)/|r|, r>=b
 * @note b=0.1的曲线图见 https://www.wolframalpha.com/input?i=sqrt%280.2*x-0.01%29%2Fx
 * TODO：应该改名叫做huber_loss
 */
static float huber_weight(float residual, float b) {  //? 在cpp文件中定义的函数都要加static？？？
  float r_abs = fabs(residual);
  if (r_abs < b) return 1.0f;
  return (float) (sqrt(2.f * b * r_abs - b * b) / r_abs);
}

/**
 * @brief 从新旧位姿中，估计更准的相对位姿
 * @details 用到了huber函数
 * @param[in] observations            新的位姿（一堆）
 * @param[in] previousEstimate        旧的位姿（一个）
 * @param[in] previousEstimate_weight 旧的位姿的权重。若为0表示没有旧的位姿
 * @param[out] out_numInliers         新位姿中inlier的个数
 * @param[out] out_inlierPose         用所有inlier直接平均得到的位姿
 * @return ORUtils::SE3Pose           用所有位姿（包含outlier）加权平均得到的位姿
 * @note estimate a relative pose, taking into account a previous estimate (weight 0 indicates that no previous estimate is available). out_numInliers and out_inlierPose is the number of inliers from amongst the new observations and the pose computed from them.
 */
static ORUtils::SE3Pose estimateRelativePose(const std::vector<Matrix4f> &observations,
                                             const ORUtils::SE3Pose &previousEstimate, float previousEstimate_weight,
                                             int *out_numInliers, ORUtils::SE3Pose *out_inlierPose) {
  //! 准备：定义超参、转换格式
  static const float huber_b = 0.1f;                          // huber函数的阈值
  static const float weightsConverged = 0.01f;                // 判断是否收敛的权重变化阈值。<则为收敛
  static const int maxIter = 10;                              // 迭代次数
  static const float inlierThresholdForFinalResult = 0.8f;    // 判断位姿是否是inlier的权重阈值。>则为inlier

  std::vector<float> weights(observations.size() + 1, 1.0f);  // 刚开始用固定权重
  std::vector<ORUtils::SE3Pose> poses;                        // 将输入的observations转成SE3格式
  for (size_t i = 0; i < observations.size(); ++i) 
    poses.push_back(ORUtils::SE3Pose(observations[i]));

  // NOTE: 循环
  float params[6];
  for (int iter = 0; iter < maxIter; ++iter) {
    //! 先用固定权重，对所有位姿做加权平均。estimate with fixed weights
    float sumweight = previousEstimate_weight;  // 初始权重
    for (int j = 0; j < 6; ++j)     // 加上旧的位姿（权重为它的初始权重）
      params[j] = weights.back() * previousEstimate_weight * previousEstimate.GetParams()[j];
    for (size_t i = 0; i < poses.size(); ++i) {
      for (int j = 0; j < 6; ++j)   // 加上新的位姿（权重为1）
        params[j] += weights[i] * poses[i].GetParams()[j];
      sumweight += weights[i];
    }
    for (int j = 0; j < 6; ++j)     // 取平均
      params[j] /= sumweight;

    //! 使用huber函数计算更新每个位姿（包含新旧位姿）的权重 compute new weights
    float avgweightchange = 0.0f;   // 权重的平均变化量，用来判断迭代是否收敛
    for (size_t i = 0; i < weights.size(); ++i) {
      const ORUtils::SE3Pose *p;
      float w = 1.0f;
      if (i < poses.size()) p = &(poses[i]);  // 输入的新位姿
      else {
        p = &(previousEstimate);              // 输入的旧位姿
        w = previousEstimate_weight;
      }

      float residual = 0.0f;                  // 残差=加权平均后的位姿与原始位姿的欧式距离（因为位姿用SE3表示，维度6x1）
      for (int j = 0; j < 6; ++j) {
        float r = p->GetParams()[j] - params[j];
        residual += r * r;
      }
      residual = sqrt(residual);

      float newweight = huber_weight(residual, huber_b);    // 使用huber函数计算新的权重
      avgweightchange += w * fabs(newweight - weights[i]);  // 记录变化量，后续判断是否收敛
      weights[i] = newweight;
    }

    //! 收敛的话提前退出
    avgweightchange = avgweightchange / (weights.size() - 1 + previousEstimate_weight);
    if (avgweightchange < weightsConverged) break;
  }
  
  //! 找到inlier，计算平均位姿
  int inliers = 0;
  Matrix4f inlierTrafo;
  inlierTrafo.setZeros();
  for (size_t i = 0; i < poses.size(); ++i) // 对所有inlier计算均值
    if (weights[i] > inlierThresholdForFinalResult) {
      inlierTrafo += observations[i];
      ++inliers;
    }
  if (out_inlierPose) out_inlierPose->SetM(inlierTrafo / (float) MAX(inliers, 1));
  if (out_numInliers) *out_numInliers = inliers;

  return ORUtils::SE3Pose(params);
}

int ITMActiveMapManager::CheckSuccess_relocalisation(int dataID) const {
  //! 跟踪成功数量>阈值，重定位成功。sucessfully relocalised
  if (activeData[dataID].constraints.size() >= N_relocsuccess) return 1;

  //! 跟踪失败的数量太多，重定位失败。relocalisation failed: declare as LOST
  // trackingAttempts表示子图跟踪的总次数（不管跟踪成功与否），constraints的数量就是跟踪成功次数，
  if ((N_reloctrials - N_relocsuccess)
      < (activeData[dataID].trackingAttempts - (int) activeData[dataID].constraints.size()))
    return -1;

  //! 下次再试试看。keep trying
  return 0;
}

int ITMActiveMapManager::CheckSuccess_newlink(int dataID, int primaryDataID, int *inliers,
                                              ORUtils::SE3Pose *inlierPose) const {
  const ActiveDataDescriptor &link = activeData[dataID];

  //! 从主子图中找到与指定子图的约束信息。take previous data from local map relations into account!
  // ORUtils::SE3Pose previousEstimate;
  // int previousEstimate_weight = 0;
  int primaryLocalMapIndex = -1;
  if (primaryDataID >= 0)
    primaryLocalMapIndex = activeData[primaryDataID].localMapIndex;
  // NOTE: 没有主子图的话不能return -1，因为可能是
  const ITMPoseConstraint &previousInformation =                // 主子图中记录的与当前子图的link
      localMapManager->getRelation_const(primaryLocalMapIndex, link.localMapIndex);
  /* 进入这个函数的dataID都对应回环or新建的子图。因为回环检测可能存在的错误是无法确定的，所以从主子图中找到link，而不是从当前子图中找link。
  hmm... do we want the "Estimate" (i.e. the pose corrected by pose graph optimization) or the "Observations" (i.e. the accumulated poses seen in previous frames? This should only really make a difference, if there is a large disagreement between the two, in which case one might argue that most likely something went wrong with a loop-closure, and we are not really sure the "Estimate" is true or just based on an erroneous loop closure. We therefore want to be consistent with previous observations not estimations...
  */
  
  ORUtils::SE3Pose previousEstimate = previousInformation.GetAccumulatedObservations(); // 约束信息中的位姿，当前子图=>主子图
  int previousEstimate_weight = previousInformation.GetNumAccumulatedObservations();    // 约束信息中的权重

  //! 估计主子图与指定子图的相对位姿，并找到inlier
  int inliers_local;
  ORUtils::SE3Pose inlierPose_local;
  if (inliers == NULL)    // 输入为空的话新建一个
    inliers = &inliers_local;
  if (inlierPose == NULL) // 输入为空的话新建一个
    inlierPose = &inlierPose_local;
  estimateRelativePose(link.constraints, previousEstimate, (float)previousEstimate_weight, inliers, inlierPose);

  //! inlier的个数超过阈值，添加link。accept link
  if (*inliers >= N_linkoverlap)
    return 1;

  //! outlier的个数超过阈值，拒绝link。reject link
  if ((N_linktrials - N_linkoverlap) < (link.trackingAttempts - *inliers))
    return -1;

  //! 下次再试试看。keep trying
  return 0;
}

void ITMActiveMapManager::AcceptNewLink(int fromData, int toData, const ORUtils::SE3Pose &pose, int weight) {
  int fromLocalMapIdx = activeData[fromData].localMapIndex;
  int toLocalMapIdx = activeData[toData].localMapIndex;

  { //! fromData里添加toData到fromData的位姿
    ITMPoseConstraint &c = localMapManager->getRelation(fromLocalMapIdx, toLocalMapIdx);
    c.AddObservation(pose, weight);
  }
  { //! toData里添加fromData到toData的位姿
    ORUtils::SE3Pose invPose(pose.GetInvM());
    ITMPoseConstraint &c = localMapManager->getRelation(toLocalMapIdx, fromLocalMapIdx);
    c.AddObservation(invPose, weight);
  }
}

bool ITMActiveMapManager::maintainActiveData(void) { 
  bool localMapGraphChanged = false;

  int primaryDataIdx = findPrimaryDataIdx();    // 主子图的活跃id
  int moveToDataIdx = -1;                       // 候选成主子图的子图的活跃id
  //! 处理每个活跃子图中类型为 、回环、新建的
  for (int i = 0; i < (int) activeData.size(); ++i) {
    ActiveDataDescriptor &link = activeData[i];

    if (link.type == RELOCALISATION) {                                      //! 处理 重定位 的活跃子图
      int success = CheckSuccess_relocalisation(i);
      if (success == 1) {
        if (moveToDataIdx < 0)    // 第一次重定位成功，则该子图就是新的主子图
          moveToDataIdx = i;
        else                      // 再次重定位成功，不是之前的主子图，就设置当前子图为lost
          link.type = LOST;  
      } else if (success == -1)   // 重定位失败
        link.type = LOST;
    }
    
    if ((link.type == LOOP_CLOSURE) || (link.type == NEW_LOCAL_MAP)) {      //! 处理 回环和新建 的活跃子图
      ORUtils::SE3Pose inlierPose;        // 当前子图的约束中的inlier直接平均得到的位姿，当前子图=>主子图
      int inliers;                        // 当前子图的约束中的inlier数量
      // TODO: 可能存在primaryDataIdx=-1吗？？？
      int success = CheckSuccess_newlink(i, primaryDataIdx, &inliers, &inlierPose); // 检查当前子图与主子图的关联是否可靠
      if (success == 1) {         // 当前子图与主子图的关联 可靠
        AcceptNewLink(primaryDataIdx, i, inlierPose, inliers);  // 建立当前子图与主子图的link
        link.constraints.clear();                               // 建立当前子图与主子图的link后，就不需要这些约束了
        link.trackingAttempts = 0;
        if (shouldMovePrimaryLocalMap(i, moveToDataIdx, primaryDataIdx))  // 查看当前子图是否要作为新的主子图
          moveToDataIdx = i;
        localMapGraphChanged = true;
      } else if (success == -1) { // 当前子图与主子图的关联 不可靠，设置当前子图为lost
        if (link.type == NEW_LOCAL_MAP) link.type = LOST_NEW;
        else link.type = LOST;
      }
    }
  }
  // add  // TODO: 下次从这儿开始
  std::vector<int> restartLinksToLocalMaps;
  primaryDataIdx = -1;
  for (int i = 0; i < (int) activeData.size(); ++i) {
    ActiveDataDescriptor &link = activeData[i];

    if ((signed) i == moveToDataIdx) link.type = PRIMARY_LOCAL_MAP;

    if ((link.type == PRIMARY_LOCAL_MAP) && (moveToDataIdx >= 0) && ((signed) i != moveToDataIdx)) {
      link.type = LOST;
      restartLinksToLocalMaps.push_back(link.localMapIndex);
    }

    if ((link.type == NEW_LOCAL_MAP) && (moveToDataIdx >= 0)) link.type = LOST_NEW;

    if ((link.type == LOOP_CLOSURE) && (moveToDataIdx >= 0)) {
      link.type = LOST;
      restartLinksToLocalMaps.push_back(link.localMapIndex);
    }

    if ((link.type == RELOCALISATION) && (moveToDataIdx >= 0)) link.type = LOST;

    if (link.type == PRIMARY_LOCAL_MAP) {
      if (primaryDataIdx >= 0) fprintf(stderr, "OOOPS, two or more primary localMaps...\n");
      primaryDataIdx = i;
    }
  }

  for (size_t i = 0; i < activeData.size();) {
    ActiveDataDescriptor &link = activeData[i];
    // 丢弃最新跟丢的子图
    if (link.type == LOST_NEW) {
      // NOTE: 最多只有一个新的子图，并保证其是localMapManager的最后一个。所以删除后不需要重新排列索引!
      // NOTE: there will only be at most one new local map at any given time and it's guaranteed to be the last in the list. Removing this new local map will therefore not require rearranging indices!
      localMapManager->removeLocalMap(link.localMapIndex);
      link.type = LOST;
    }
    if (link.type == LOST) activeData.erase(activeData.begin() + i);
    else i++;
  }

  for (std::vector<int>::const_iterator it = restartLinksToLocalMaps.begin(); it != restartLinksToLocalMaps.end();
       ++it) {
    initiateNewLink(*it, *(localMapManager->getTrackingPose(*it)), false);
  }

  // NOTE: this has to be done AFTER removing any previous new local map
  if (shouldStartNewArea()) {
    int newIdx = initiateNewLocalMap();

    if (primaryDataIdx >= 0) {
      int primaryLocalMapIdx = activeData[primaryDataIdx].localMapIndex;
      localMapManager->setEstimatedGlobalPose(newIdx,
                                              ORUtils::SE3Pose(
                                                  localMapManager->getTrackingPose(primaryLocalMapIdx)->GetM()
                                                      * localMapManager->getEstimatedGlobalPose(primaryLocalMapIdx).GetM()));
    }
  }

  return localMapGraphChanged;
}
