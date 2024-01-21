// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"
#include "../../FernRelocLib/Relocaliser.h"

#include "../Engines/MultiScene/ITMActiveMapManager.h"
#include "../Engines/MultiScene/ITMGlobalAdjustmentEngine.h"
#include "../Engines/Visualisation/Interface/ITMMultiVisualisationEngine.h"
#include "../Engines/Meshing/ITMMultiMeshingEngineFactory.h"

#include <vector>

namespace ITMLib {
/** \brief
*/
template<typename TVoxel, typename TIndex>
class ITMMultiEngine : public ITMMainEngine {
/* ----------------------------------------------------- private ---------------------------------------------------- */
 private:
  const ITMLibSettings *settings;

  ITMLowLevelEngine *lowLevelEngine;                                  // 底层的图像处理模块（拷贝、彩色转灰色等操作，不是预处理）
  ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;        // 负责渲染（可视化）
  ITMMultiVisualisationEngine<TVoxel, TIndex> *multiVisualisationEngine;  

  ITMMultiMeshingEngine<TVoxel, TIndex> *meshingEngine;   // 负责mesh

  ITMViewBuilder *viewBuilder;                      // 负责输入图像 && 预处理
  ITMTrackingController *trackingController;        // 负责调用raycsting，为下一帧的跟踪做准备
  ITMTracker *tracker;                              // 负责跟踪
  ITMIMUCalibrator *imuCalibrator;                  // 负责IMU预积分
  ITMDenseMapper<TVoxel, TIndex> *denseMapper;      // 负责 场景三维模型的融合 && swap in/out

  FernRelocLib::Relocaliser<float> *relocaliser;    // 负责重定位

  ITMVoxelMapGraphManager<TVoxel, TIndex> *mapManager;  // 负责管理所有子图
  ITMActiveMapManager *mActiveDataManager;              // 负责管理活跃子图
  ITMGlobalAdjustmentEngine *mGlobalAdjustmentEngine;   // 负责全局优化
  bool mScheduleGlobalAdjustment;                       // 是否进行全局优化

  Vector2i trackedImageSize;                        // 用于跟踪的图像分辨率
  ITMRenderState *renderState_freeview;             // 渲染结果：自由视角
  ITMRenderState *renderState_multiscene;           // 渲染结果：固定视角
  int freeviewLocalMapIdx;

  ITMView *view;                                    // 当前帧的指针。Pointer for storing the current input frame
/* ----------------------------------------------------- public ----------------------------------------------------- */
 public:
  ITMView *GetView() { return view; }

  ITMTrackingState *GetTrackingState(void);

  /// Process a frame with rgb and depth images and (optionally) a corresponding imu measurement
  ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage,
                                                ITMIMUMeasurement *imuMeasurement = NULL);

  /// Get a result image as output
  Vector2i GetImageSize(void) const;

  void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL,
                ITMIntrinsics *intrinsics = NULL);

  void changeFreeviewLocalMapIdx(ORUtils::SE3Pose *pose, int newIdx);
  void setFreeviewLocalMapIdx(int newIdx) {
    freeviewLocalMapIdx = newIdx;
  }
  int getFreeviewLocalMapIdx(void) const {
    return freeviewLocalMapIdx;
  }
  int findPrimaryLocalMapIdx(void) const {
    return mActiveDataManager->findPrimaryLocalMapIdx();
  }

  /// Extracts a mesh from the current scene and saves it to the model file specified by the file name
  void SaveSceneToMesh(const char *fileName);

  /// save and load the full scene and relocaliser (if any) to/from file
  void SaveToFile();
  void LoadFromFile();

  //void writeFullTrajectory(void) const;
  //void SaveSceneToMesh(const char *objFileName);

  /** \brief Constructor
      Ommitting a separate image size for the depth images will assume same resolution as for the RGB images.
  */
  ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib, Vector2i imgSize_rgb,
                 Vector2i imgSize_d = Vector2i(-1, -1));
  ~ITMMultiEngine(void);
};
}
