// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>

#include "../Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Trackers/Interface/ITMTracker.h"
#include "../Utils/ITMLibSettings.h"

namespace ITMLib {
/** \brief 调用tracking模块的模块？？？
*/
class ITMTrackingController {
 private:
  const ITMLibSettings *settings;
  ITMTracker *tracker;

 public:
  void Track(ITMTrackingState *trackingState, const ITMView *view) {
    tracker->TrackCamera(trackingState, view);
  }

  template<typename TSurfel>
  void Prepare(ITMTrackingState *trackingState, const ITMSurfelScene<TSurfel> *scene, const ITMView *view,
               const ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine, ITMSurfelRenderState *renderState) {
    if (!tracker->requiresPointCloudRendering())
      return;

    //render for tracking
    bool requiresColourRendering = tracker->requiresColourRendering();
    bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

    if (requiresColourRendering) {
      // TODO: This should be implemented at some point.
      throw std::runtime_error("The surfel engine doesn't yet support colour trackers");
    } else {
      const bool useRadii = true;
      visualisationEngine->FindSurface(scene,
                                       trackingState->pose_d,
                                       &view->calib.intrinsics_d,
                                       useRadii,
                                       USR_FAUTEDEMIEUX,
                                       renderState);
      trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

      if (requiresFullRendering) {
        visualisationEngine->CreateICPMaps(scene, renderState, trackingState);
        trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
        if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
        else trackingState->age_pointCloud = 0;
      } else {
        trackingState->age_pointCloud++;
      }
    }
  }
  /**
   * @brief 均匀划分的voxel场景的raycast
   *
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
   * @param[in] scene 三维模型
   * @param[in] view 当前输入图像
   * @param[in] visualisationEngine 包含渲染相关的函数。比如可见block的寻找
   * @param[out] renderState raycasting的到的结果
   */
  template<typename TVoxel, typename TIndex>
  void Prepare(ITMTrackingState *trackingState, const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
               const ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine, ITMRenderState *renderState) {
    if (!tracker->requiresPointCloudRendering())  // TODO： 什么时候会不需要raycasting？？？
      return;

    //! render for tracking
    bool requiresColourRendering = tracker->requiresColourRendering();  // depth tracking为false，rgb tracking为true
    bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;
    printf("requiresColourRendering=%d, requiresFullRendering=%d\n",requiresColourRendering,requiresFullRendering);
    if (requiresColourRendering) {
      // 计算rgb图的坐标系下的位姿 = T_d2r * T_w2d  ？？？这里反了吧？？？应该是T_w2d * T_d2r
      ORUtils::SE3Pose pose_rgb(view->calib.trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
      // 估计raycasting的深度范围
      visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib.intrinsics_rgb), renderState);
      // raycasting
      visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
      trackingState->age_pointCloud = 0;
    } else {
      // 估计raycasting的深度范围
      visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib.intrinsics_d), renderState);
      // raycasting
      if (requiresFullRendering) {  // 普通的raycasting
        visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
        trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
        if (trackingState->age_pointCloud == -1) trackingState->age_pointCloud = -2;
        else trackingState->age_pointCloud = 0;
      } else {                      // 增量式的raycasting
        visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
        trackingState->age_pointCloud++;
      }
    }
  }

  ITMTrackingController(ITMTracker *tracker, const ITMLibSettings *settings) {
    this->tracker = tracker;
    this->settings = settings;
  }

  const Vector2i &GetTrackedImageSize(const Vector2i &imgSize_rgb, const Vector2i &imgSize_d) const {
    return tracker->requiresColourRendering() ? imgSize_rgb : imgSize_d;
  }

  // Suppress the default copy constructor and assignment operator
  ITMTrackingController(const ITMTrackingController &);
  ITMTrackingController &operator=(const ITMTrackingController &);
};
}
