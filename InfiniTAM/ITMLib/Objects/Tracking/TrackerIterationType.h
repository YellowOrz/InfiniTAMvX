// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib {
/// The tracker iteration type used to define the tracking iteration regime
/** 迭代的跟踪类型。在ITMLibSettings => trackerConfig => levels里设置 */
enum TrackerIterationType {
  TRACKER_ITERATION_ROTATION,     // 只有旋转
  TRACKER_ITERATION_TRANSLATION,  // 只有平移
  TRACKER_ITERATION_BOTH,         // 都要
  TRACKER_ITERATION_NONE          // 都不要。
};
}
