// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMBasicEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/Meshing/ITMMeshingEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Objects/RenderStates/ITMRenderStateFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../ORUtils/NVTimer.h"
#include "../../ORUtils/FileUtils.h"

//#define OUTPUT_TRAJECTORY_QUATERNIONS

using namespace ITMLib;
// TODO: 下次从这儿开始
template <typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel, TIndex>::ITMBasicEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib,
                                               Vector2i imgSize_rgb, Vector2i imgSize_d) {
  // 系统设置
  this->settings = settings;
  // 深度图和彩色图的尺寸
  if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;
  // 内存类型 && 场景构建
  MemoryDeviceType memoryType = settings->GetMemoryType();
  this->scene = new ITMScene<TVoxel, TIndex>(&settings->sceneParams,
                                             settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
                                             memoryType);
  // 设备类型
  const ITMLibSettings::DeviceType deviceType = settings->deviceType;
  // 底层的图像处理模块（拷贝、彩色转灰色等操作，不是预处理）
  lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
  // 输入图像 && 预处理
  viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
  // 渲染（可视化）
  visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);
  // mesh
  meshingEngine = NULL;
  if (settings->createMeshingEngine)
    meshingEngine = ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(deviceType);
  // 负责 场景三维模型的融合 && swap in/out
  denseMapper = new ITMDenseMapper<TVoxel, TIndex>(settings);
  denseMapper->ResetScene(scene);
  // 初始化IMU预积分器和跟踪器
  imuCalibrator = new ITMIMUCalibrator_iPad();
  tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator,
                                               scene->sceneParams);
  trackingController = new ITMTrackingController(tracker, settings);
  // 获取图像大小
  Vector2i trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);
  // 初始哈跟踪状态和位姿
  trackingState = new ITMTrackingState(trackedImageSize, memoryType);
  tracker->UpdateInitialPose(trackingState);
  // 渲染结果：固定视角 && 自由视角
  renderState_live = ITMRenderStateFactory<TIndex>::CreateRenderState(trackedImageSize, scene->sceneParams, memoryType);
  renderState_freeview = NULL; //will be created if needed

  view = NULL;                                              // 当前输入图像。will be allocated by the view builder
  kfRaycast = new ITMUChar4Image(imgSize_d, memoryType);    // raycast图片

  // 重定位
  if (settings->behaviourOnFailure == settings->FAILUREMODE_RELOCALISE)
    relocaliser = new FernRelocLib::Relocaliser<float>(imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, 
                                                       settings->sceneParams.viewFrustum_max), 0.2f, 500, 4);
  else relocaliser = NULL;

  // 各模块状态
  trackingActive = true;        // 开启跟踪
  fusionActive = true;          // 开启三维场景的融合
  mainProcessingActive = true;  // 主线程。主要在移动端会关闭
  trackingInitialised = false;  // 跟踪未初始化
  relocalisationCount = 0;      // 重定位后要求的连续跟踪成功帧数
  framesProcessed = 0;          // 跟踪成功帧数
}

template<typename TVoxel, typename TIndex>
ITMBasicEngine<TVoxel, TIndex>::~ITMBasicEngine() {
  delete renderState_live;
  if (renderState_freeview != NULL) delete renderState_freeview;

  delete scene;

  delete denseMapper;
  delete trackingController;

  delete tracker;
  delete imuCalibrator;

  delete lowLevelEngine;
  delete viewBuilder;

  delete trackingState;
  if (view != NULL) delete view;

  delete visualisationEngine;

  if (relocaliser != NULL) delete relocaliser;
  delete kfRaycast;

  if (meshingEngine != NULL) delete meshingEngine;
}

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::SaveSceneToMesh(const char *objFileName) {
  if (meshingEngine == NULL) return;

  ITMMesh *mesh = new ITMMesh(settings->GetMemoryType());

  meshingEngine->MeshScene(mesh, scene);
  mesh->WriteSTL(objFileName);

  delete mesh;
}

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::SaveToFile() {
  // throws error if any of the saves fail

  std::string saveOutputDirectory = "State/";
  std::string relocaliserOutputDirectory = saveOutputDirectory + "Relocaliser/",
      sceneOutputDirectory = saveOutputDirectory + "Scene/";

  MakeDir(saveOutputDirectory.c_str());
  MakeDir(relocaliserOutputDirectory.c_str());
  MakeDir(sceneOutputDirectory.c_str());

  if (relocaliser) relocaliser->SaveToDirectory(relocaliserOutputDirectory);

  scene->SaveToDirectory(sceneOutputDirectory);
}

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::LoadFromFile() {
  std::string saveInputDirectory = "State/";
  std::string relocaliserInputDirectory = saveInputDirectory + "Relocaliser/",
      sceneInputDirectory = saveInputDirectory + "Scene/";

  ////TODO: add factory for relocaliser and rebuild using config from relocaliserOutputDirectory + "config.txt"
  ////TODO: add proper management of case when scene load fails (keep old scene or also reset relocaliser)

  this->resetAll();

  try { // 读取重定位的信息。load relocaliser
    FernRelocLib::Relocaliser<float> *relocaliser_temp = new FernRelocLib::Relocaliser<float>(
        view->depth->noDims, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max),
        0.2f, 500, 4);

    relocaliser_temp->LoadFromDirectory(relocaliserInputDirectory);

    delete relocaliser;
    relocaliser = relocaliser_temp;
  } catch (std::runtime_error &e) {
    throw std::runtime_error("Could not load relocaliser: " + std::string(e.what()));
  }

  try { // 读取三维场景的信息。load scene
    scene->LoadFromDirectory(sceneInputDirectory);
  } catch (std::runtime_error &e) {
    denseMapper->ResetScene(scene);
    throw std::runtime_error("Could not load scene:" + std::string(e.what()));
  }
}

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::resetAll() {
  denseMapper->ResetScene(scene);
  trackingState->Reset();
}

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
static int QuaternionFromRotationMatrix_variant(const double *matrix)
{
    int variant = 0;
    if
        ((matrix[4]>-matrix[8]) && (matrix[0]>-matrix[4]) && (matrix[0]>-matrix[8]))
    {
        variant = 0;
    }
    else if ((matrix[4]<-matrix[8]) && (matrix[0]>
        matrix[4]) && (matrix[0]> matrix[8])) {
        variant = 1;
    }
    else if ((matrix[4]> matrix[8]) && (matrix[0]<
        matrix[4]) && (matrix[0]<-matrix[8])) {
        variant = 2;
    }
    else if ((matrix[4]<
        matrix[8]) && (matrix[0]<-matrix[4]) && (matrix[0]< matrix[8])) {
        variant = 3;
    }
    return variant;
}

static void QuaternionFromRotationMatrix(const double *matrix, double *q) {
    /* taken from "James Diebel. Representing Attitude: Euler
    Angles, Quaternions, and Rotation Vectors. Technical Report, Stanford
    University, Palo Alto, CA."
    */

    // choose the numerically best variant...
    int variant = QuaternionFromRotationMatrix_variant(matrix);
    double denom = 1.0;
    if (variant == 0) {
        denom += matrix[0] + matrix[4] + matrix[8];
    }
    else {
        int tmp = variant * 4;
        denom += matrix[tmp - 4];
        denom -= matrix[tmp % 12];
        denom -= matrix[(tmp + 4) % 12];
    }
    denom = sqrt(denom);
    q[variant] = 0.5*denom;

    denom *= 2.0;
    switch (variant) {
    case 0:
        q[1] = (matrix[5] - matrix[7]) / denom;
        q[2] = (matrix[6] - matrix[2]) / denom;
        q[3] = (matrix[1] - matrix[3]) / denom;
        break;
    case 1:
        q[0] = (matrix[5] - matrix[7]) / denom;
        q[2] = (matrix[1] + matrix[3]) / denom;
        q[3] = (matrix[6] + matrix[2]) / denom;
        break;
    case 2:
        q[0] = (matrix[6] - matrix[2]) / denom;
        q[1] = (matrix[1] + matrix[3]) / denom;
        q[3] = (matrix[5] + matrix[7]) / denom;
        break;
    case 3:
        q[0] = (matrix[1] - matrix[3]) / denom;
        q[1] = (matrix[6] + matrix[2]) / denom;
        q[2] = (matrix[5] + matrix[7]) / denom;
        break;
    }

    if (q[0] < 0.0f) for (int i = 0; i < 4; ++i) q[i] *= -1.0f;
}
#endif

template<typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMBasicEngine<TVoxel, TIndex>::ProcessFrame(ITMUChar4Image *rgbImage,
                                                                              ITMShortImage *rawDepthImage,
                                                                              ITMIMUMeasurement *imuMeasurement) {
  // prepare image and turn it into a depth image
  //! 准备数据：对输入数据预处理后，放到view中
  if (imuMeasurement == NULL)   // 无IMU
    viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
  else                          // 有IMU
    viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);

  if (!mainProcessingActive)    // 在移动端可以关闭主线程（包含tracking？？？）
    return ITMTrackingState::TRACKING_FAILED;

  // tracking
  //! 跟踪
  ORUtils::SE3Pose oldPose(*(trackingState->pose_d));   // trackingState是成员变量
  if (trackingActive) 
    trackingController->Track(trackingState, view);

  ITMTrackingState::TrackingResult trackerResult = ITMTrackingState::TRACKING_GOOD;
  
  //! 跟踪失败后的操作：重定位 or 停止融合
  switch (settings->behaviourOnFailure) {
  case ITMLibSettings::FAILUREMODE_RELOCALISE: // 重定位
    trackerResult = trackingState->trackerResult;
    break;
  case ITMLibSettings::FAILUREMODE_STOP_INTEGRATION: // 停止融合
    if (trackingState->trackerResult != ITMTrackingState::TRACKING_FAILED)
      trackerResult = trackingState->trackerResult;
    else
      trackerResult = ITMTrackingState::TRACKING_POOR;
    break;
  default:
    break;
  }

  //! 如果需要重定位。relocalisation
  int addKeyframeIdx = -1;  // 添加Keyframe的ID？？？
  if (settings->behaviourOnFailure == ITMLibSettings::FAILUREMODE_RELOCALISE) {
    if (trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount > 0) relocalisationCount--;

    int NN;
    float distances;
    view->depth->UpdateHostFromDevice();

    // 查看是否需要添加关键帧。find and add keyframe, if necessary
    bool hasAddedKeyframe =
        relocaliser->ProcessFrame(view->depth, trackingState->pose_d, 0, 1, &NN, &distances,
                                  trackerResult == ITMTrackingState::TRACKING_GOOD && relocalisationCount == 0);

    // 添加失败 && 跟踪失败，正式进入重定位。frame not added and tracking failed -> we need to relocalise
    if (!hasAddedKeyframe && trackerResult == ITMTrackingState::TRACKING_FAILED) {
      relocalisationCount = 10;

      // Reset previous rgb frame since the rgb image is likely different than the one acquired when setting the keyframe
      // 删除输入的彩色图，因为可能与设置关键帧时获得的图像不同
      view->rgb_prev->Clear();

      const FernRelocLib::PoseDatabase::PoseInScene &keyframe = relocaliser->RetrievePose(NN);
      trackingState->pose_d->SetFrom(&keyframe.pose);

      denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live, true);
      trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);
      trackingController->Track(trackingState, view);

      trackerResult = trackingState->trackerResult;
    }
  }
  //! 将当前帧融合进三维模型
  bool didFusion = false;   // 是否融合成功
  // fusion同时满足条件：① 跟踪good or 不是刚刚初始化；② 开启fusion；③ 重定位后连续跟踪成功的帧数足够
  if ((trackerResult == ITMTrackingState::TRACKING_GOOD || !trackingInitialised) && (fusionActive)
      && (relocalisationCount == 0)) {
    denseMapper->ProcessFrame(view, trackingState, scene, renderState_live);
    didFusion = true;
    if (framesProcessed > 50) trackingInitialised = true;

    framesProcessed++;
  }
  //! 更新三维模型在当前帧的投影：使用raycast，只要不是TRACING_FAILED
  if (trackerResult == ITMTrackingState::TRACKING_GOOD || trackerResult == ITMTrackingState::TRACKING_POOR) {
    // 关闭fusion的时候，要更新
    if (!didFusion) denseMapper->UpdateVisibleList(view, trackingState, scene, renderState_live);

    // raycast to renderState_live for tracking and free visualisation
    // 为下一帧的跟踪和可视化做准备。raycast结果存在renderState_live
    trackingController->Prepare(trackingState, scene, view, visualisationEngine, renderState_live);
    // 添加关键帧 // TODO: 不可能>=0
    if (addKeyframeIdx >= 0) {
      ORUtils::MemoryBlock<Vector4u>::MemoryCopyDirection memoryCopyDirection =
          settings->deviceType == ITMLibSettings::DEVICE_CUDA ? ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CUDA
                                                              : ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU;

      kfRaycast->SetFrom(renderState_live->raycastImage, memoryCopyDirection);
    }
  } else *trackingState->pose_d = oldPose;

#ifdef OUTPUT_TRAJECTORY_QUATERNIONS
  const ORUtils::SE3Pose *p = trackingState->pose_d;
  double t[3];
  double R[9];
  double q[4];
  for (int i = 0; i < 3; ++i) t[i] = p->GetInvM().m[3 * 4 + i];
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
      R[r * 3 + c] = p->GetM().m[c * 4 + r];
  QuaternionFromRotationMatrix(R, q);
  fprintf(stderr, "%f %f %f %f %f %f %f\n", t[0], t[1], t[2], q[1], q[2], q[3], q[0]);
#endif

  return trackerResult;
}

template<typename TVoxel, typename TIndex>
Vector2i ITMBasicEngine<TVoxel, TIndex>::GetImageSize(void) const {
  return renderState_live->raycastImage->noDims;
}

template <typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose,
                                              ITMIntrinsics *intrinsics) {
  if (view == NULL)
    return;

  out->Clear();
  //! 根据所需的图片类型不同，渲染不同的图片
  switch (getImageType) {
  // NOTE: 以下都是直接来自输入图片
  case ITMBasicEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:                            // 输入的彩色图
    out->ChangeDims(view->rgb->noDims);
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
    else
      out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    break;
  case ITMBasicEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:                          // 输入的深度图
    out->ChangeDims(view->depth->noDims);
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      view->depth->UpdateHostFromDevice();
    ITMVisualisationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depth);

    break;
  // NOTE: 以下都是固定视角的图片
  case ITMBasicEngine::InfiniTAM_IMAGE_SCENERAYCAST:
  case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:
  case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
  case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
    // use current raycast or forward projection?
    IITMVisualisationEngine::RenderRaycastSelection raycastType;
    if (trackingState->age_pointCloud <= 0) // 直接使用旧的普通raycast结果
      raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
    else                                    // 直接使用旧的增量raycat结果
      raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;

    // 设置渲染图片类型。what sort of image is it?
    IITMVisualisationEngine::RenderImageType imageType;
    switch (getImageType) {
    case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:                // 三维场景的置信度的伪彩色图
      imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
      break;
    case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:                    // 三维场景的单位法向量的伪彩色图
      imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
      break;
    case ITMBasicEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME:                    // 三维场景的彩色图
      imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
      break;
    default:                                                                    // 有序点云的法向量夹角图（灰色）
      imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
    }
    // 渲染图片
    visualisationEngine->RenderImage(scene, trackingState->pose_d, &view->calib.intrinsics_d, renderState_live,
                                     renderState_live->raycastImage, imageType, raycastType);
    // 把渲染的结果转移到out
    ORUtils::Image<Vector4u> *srcImage = NULL;
    if (relocalisationCount != 0)
      srcImage = kfRaycast;
    else
      srcImage = renderState_live->raycastImage;

    out->ChangeDims(srcImage->noDims);  // 修改图片大小
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
    else
      out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

    break;
  }
  // NOTE: 以下都是自由视角的图片
  case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
  case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
  case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
  case ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
    // 默认是法向量夹角图（灰度）
    IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
    if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME)          // 三维场景的彩色图
      type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
    else if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL)     // 单位法向量的伪彩色图
      type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
    else if (getImageType == ITMBasicEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE) // 置信度的伪彩色图
      type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

    if (renderState_freeview == NULL) {
      renderState_freeview =
          ITMRenderStateFactory<TIndex>::CreateRenderState(out->noDims, scene->sceneParams, settings->GetMemoryType());
    }
    // NOTE: 因为现在是自由视角，所以需要在当前视角下重新raycast，不能用跟踪里的raycast结果
    // raycast三部曲：找可见block、确定ray的搜索范围、渲染图片
    visualisationEngine->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
    visualisationEngine->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
    visualisationEngine->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage,
                                     type);

    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
    else
      out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    break;
  }
  case ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN:
    break;
  };
}

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::turnOnTracking() { trackingActive = true; }

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::turnOffTracking() { trackingActive = false; }

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::turnOnIntegration() { fusionActive = true; }

template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::turnOffIntegration() { fusionActive = false; }
/** 主要在移动端使用 */
template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::turnOnMainProcessing() { mainProcessingActive = true; }
/** 主要在移动端使用 */
template<typename TVoxel, typename TIndex>
void ITMBasicEngine<TVoxel, TIndex>::turnOffMainProcessing() { mainProcessingActive = false; }
