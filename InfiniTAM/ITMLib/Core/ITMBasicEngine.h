// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMDenseMapper.h"
#include "ITMMainEngine.h"
#include "ITMTrackingController.h"
#include "../Engines/LowLevel/Interface/ITMLowLevelEngine.h"
#include "../Engines/Meshing/Interface/ITMMeshingEngine.h"
#include "../Engines/ViewBuilding/Interface/ITMViewBuilder.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Objects/Misc/ITMIMUCalibrator.h"

#include "../../FernRelocLib/Relocaliser.h"

namespace ITMLib {
/**
 * @brief 最基础的SLAM系统
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 */
template<typename TVoxel, typename TIndex>
class ITMBasicEngine : public ITMMainEngine {
 private:
  const ITMLibSettings *settings;

  bool trackingActive, fusionActive, mainProcessingActive, 
       trackingInitialised; // 是否初始化成功。融合帧数>50就成功
  int framesProcessed, relocalisationCount;

  ITMLowLevelEngine *lowLevelEngine;
  ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;

  ITMMeshingEngine<TVoxel, TIndex> *meshingEngine;

  ITMViewBuilder *viewBuilder;    
  ITMDenseMapper<TVoxel, TIndex> *denseMapper;  // 用来更新三维模型：包含融合、更新可见列表
  ITMTrackingController *trackingController;

  ITMScene<TVoxel, TIndex> *scene;    
  ITMRenderState *renderState_live;
  ITMRenderState *renderState_freeview;

  ITMTracker *tracker;
  ITMIMUCalibrator *imuCalibrator;

  FernRelocLib::Relocaliser<float> *relocaliser;
  ITMUChar4Image *kfRaycast;

  /// Pointer for storing the current input frame
  ITMView *view;    // 指向 当前输入图像

  /// Pointer to the current camera pose and additional tracking information
  ITMTrackingState *trackingState;    // 包含跟踪得到的相机位姿、跟踪的分数等

 public:
  ITMView *GetView(void) { return view; }
  ITMTrackingState *GetTrackingState(void) { return trackingState; }

  /// Gives access to the internal world representation
  ITMScene<TVoxel, TIndex> *GetScene(void) { return scene; }
  /**
   * @brief 跟踪单帧
   * 
   * @param[in] rgbImage 彩色图
   * @param[in] rawDepthImage 深度图
   * @param[in] imuMeasurement IMU数据
   * @return ITMTrackingState::TrackingResult 
   */
  ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage,
                                                ITMIMUMeasurement *imuMeasurement = NULL);

  // 使用marching cube从当前场景中导出mesh三维模型
  /// Extracts a mesh from the current scene and saves it to the model file specified by the file name
  void SaveSceneToMesh(const char *fileName);

  // 将整个场景（hash table等）和重定位的信息保存到硬盘上指定位置
  /// save and load the full scene and relocaliser (if any) to/from file
  void SaveToFile();
  // 读取SaveToFile()保存的场景和重定位信息
  void LoadFromFile();

  /// Get a result image as output
  Vector2i GetImageSize(void) const;

  void GetImage(ITMUChar4Image *out,
                GetImageType getImageType,
                ORUtils::SE3Pose *pose = NULL,
                ITMIntrinsics *intrinsics = NULL);

  /// switch for turning tracking on/off
  void turnOnTracking();
  void turnOffTracking();

  /// switch for turning integration on/off
  void turnOnIntegration();
  void turnOffIntegration();

  /// switch for turning main processing on/off
  void turnOnMainProcessing();
  void turnOffMainProcessing();

  /// resets the scene and the tracker
  void resetAll();

  /** \brief Constructor
      Omitting a separate image size for the depth images
      will assume same resolution as for the RGB images.
  */
  /**
   * 构建base SLAM系统：使用Voxel & 无回环
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] settings  相关设置
   * @param[in] calib 相机参数
   * @param[in] imgSize_rgb 彩色图大小
   * @param[in] imgSize_d 深度图大小
   */
  ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib, Vector2i imgSize_rgb,
                 Vector2i imgSize_d = Vector2i(-1, -1));
  ~ITMBasicEngine();
};
}
