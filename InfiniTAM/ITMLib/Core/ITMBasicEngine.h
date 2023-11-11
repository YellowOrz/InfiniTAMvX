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

  bool trackingActive;        // 是否开启跟踪
  bool fusionActive;          // 是否开启三维场景的融合
  bool mainProcessingActive;  // 是否开启主线程。只在移动端使用
  bool trackingInitialised;   // 是否初始化成功。融合帧数>50就成功
  int framesProcessed;        // 跟踪成功帧数
  int relocalisationCount;    // 重定位后要求的连续跟踪成功帧数

  ITMLowLevelEngine *lowLevelEngine;                            // 底层的图像处理模块（拷贝、彩色转灰色等操作，不是预处理）
  ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;  // 渲染（可视化）

  ITMMeshingEngine<TVoxel, TIndex> *meshingEngine;              // mesh

  ITMViewBuilder *viewBuilder;                                  // 输入图像 && 预处理
  ITMDenseMapper<TVoxel, TIndex> *denseMapper;                  // 负责 场景三维模型的融合 && swap in/out

  ITMScene<TVoxel, TIndex> *scene;                              // 三维场景
  ITMRenderState *renderState_live;                             // 渲染结果：固定视角
  ITMRenderState *renderState_freeview;                         // 渲染结果：自由视角

  ITMTracker *tracker;                                          // 跟踪
  ITMIMUCalibrator *imuCalibrator;                              // IMU预积分
  ITMTrackingController *trackingController;                    // 负责调用raycsting，为下一帧的跟踪做准备

  FernRelocLib::Relocaliser<float> *relocaliser;                // 重定位
  ITMUChar4Image *kfRaycast;                                    // raycast图片。从渲染结果里面来的

  /// Pointer for storing the current input frame
  ITMView *view;                                                // 当前输入图像

  /// Pointer to the current camera pose and additional tracking information
  ITMTrackingState *trackingState;                              // 包含跟踪得到的相机位姿、跟踪的分数等

 public:
  /** 获取当前输入图片。Gives access to the current input frame */
  ITMView *GetView(void) { return view; }
  /** 获取当前跟踪状态（比如相机位姿等）。Gives access to the current camera pose and additional tracking information */
  ITMTrackingState *GetTrackingState(void) { return trackingState; }

  /// Gives access to the internal world representation
  ITMScene<TVoxel, TIndex> *GetScene(void) { return scene; }
  /**
   * @brief 跟踪单帧
   * @param[in] rgbImage 输入的彩色图
   * @param[in] rawDepthImage 输入的深度图
   * @param[in] imuMeasurement 输入的IMU数据（可选）
   * @return ITMTrackingState::TrackingResult 
   * @note Process a frame with rgb and depth images and optionally a corresponding imu measurement
   */
  ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage,
                                                ITMIMUMeasurement *imuMeasurement = NULL);

  /** 使用marching cube从当前场景中导出mesh三维模型
      Extracts a mesh from the current scene and saves it to the model file specified by the file name */
  void SaveSceneToMesh(const char *fileName);

  /** 将整个场景（hash table等）和重定位的信息保存到硬盘上指定位置
     save and load the full scene and relocaliser (if any) to/from file */
  void SaveToFile();
  /** 读取SaveToFile()保存的场景和重定位信息 */
  void LoadFromFile();

  /** 获取raycast图片的尺寸。Get a result image as output */
  Vector2i GetImageSize(void) const;
  /**
   * @brief 获取要显示的图片（基本上就是raycast）
   * @param[out] out          要显示的图片
   * @param[in] getImageType  图片类型
   * @param[in] pose          相机位姿
   * @param[in] intrinsics    相机内参
   */
  void GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose = NULL, 
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

  /**
   * 构建base SLAM系统：使用Voxel & 无回环
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] settings  相关设置
   * @param[in] calib 相机参数
   * @param[in] imgSize_rgb 彩色图大小
   * @param[in] imgSize_d 深度图大小
   * @details Constructor
   *          Omitting a separate image size for the depth images will assume same resolution as for the RGB images.
   */
  ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib, Vector2i imgSize_rgb,
                 Vector2i imgSize_d = Vector2i(-1, -1));
  ~ITMBasicEngine();
};
}
