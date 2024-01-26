// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "../../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../../Objects/RenderStates/ITMRenderState.h"
#include "../../Objects/Scene/ITMScene.h"
#include "../../Objects/Tracking/ITMTrackingState.h"
#include "../../Utils/ITMLibSettings.h"

namespace ITMLib {
/** 两个子图之间的位姿约束信息 */
struct ITMPoseConstraint {  // TODO: 应该改名叫link，不会与ActiveDataDescriptor中的constrain搞混！
 public:
  ITMPoseConstraint(void) {
    accu_num = 0;
  }
  /**
   * @brief 通过加权平均的方式，添加观测（即相机位姿）
   * @param[in] relative_pose 相机位姿
   * @param[in] weight        权重
   */
  void AddObservation(const ORUtils::SE3Pose &relative_pose, int weight = 1) {
    Matrix4f tmp = accu_poses.GetM() * (float) accu_num + relative_pose.GetM() * (float) weight;  // ?位姿可以加权求和？
    accu_num += weight;
    accu_poses.SetM(tmp / (float) accu_num);  // 对位姿进行加权平均
    accu_poses.Coerce();
    //	accu_poses = (accu_poses * (float)accu_num + relative_pose)/(float)(accu_num+1);
    accu_num++;
  }
  /** 获取 加权平均后的位姿 */
  ORUtils::SE3Pose GetAccumulatedObservations(void) const { return accu_poses; }
  /** 获取 权重 */
  int GetNumAccumulatedObservations(void) const { return accu_num; }

 private:
  ORUtils::SE3Pose accu_poses;  // 加权平均后的位姿
  int accu_num;                 // 权重
};

/** 跟当前子图相关的所有子图id 以及 对应的位姿（也叫做link，关联子图=>当前子图） */
typedef std::map<int, ITMPoseConstraint> ConstraintList;
/** 子图 */
template<class TVoxel, class TIndex>
class ITMLocalMap {
 public:
  ITMScene<TVoxel, TIndex> *scene;      // 只包含当前子图的三维场景
  ITMRenderState *renderState;          // 当前子图的渲染结果的指针
  ITMTrackingState *trackingState;      // 当前子图参与跟踪的变量的指针。包含子图中的相机位姿（即子图坐标系到当前帧的位姿，T_ls
  ConstraintList relations;             // 当前子图相关的所有子图id 以及 对应的位姿（即link）
  ORUtils::SE3Pose estimatedGlobalPose; // 子图位姿，即世界坐标系下到子图的第1帧（对应子图坐标系） 的位姿，T_sw
  ITMLocalMap(const ITMLibSettings *settings, const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine,
              const Vector2i &trackedImageSize) {
    MemoryDeviceType
        memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
    scene = new ITMScene<TVoxel, TIndex>(&settings->sceneParams,
                                         settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
                                         memoryType);
    renderState = visualisationEngine->CreateRenderState(scene, trackedImageSize);
    trackingState = new ITMTrackingState(trackedImageSize, memoryType);
  }
  ~ITMLocalMap(void) {
    delete scene;
    delete renderState;
    delete trackingState;
  }
};
}

