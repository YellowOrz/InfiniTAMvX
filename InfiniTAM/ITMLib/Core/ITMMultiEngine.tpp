// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMMultiEngine.h"

#include "../Engines/LowLevel/ITMLowLevelEngineFactory.h"
#include "../Engines/ViewBuilding/ITMViewBuilderFactory.h"
#include "../Engines/Visualisation/ITMVisualisationEngineFactory.h"
#include "../Engines/Visualisation/ITMMultiVisualisationEngineFactory.h"
#include "../Trackers/ITMTrackerFactory.h"

#include "../../MiniSlamGraphLib/QuaternionHelpers.h"

using namespace ITMLib;

//#define DEBUG_MULTISCENE

// 闭环检测时，寻找最近邻居的个数。number of nearest neighbours to find in the loop closure detection
static const int k_loopcloseneighbours = 1;

// 重定位遍历的邻居的距离不能超过这个阈值。maximum distance reported by LCD library to attempt relocalisation
static const float F_maxdistattemptreloc = 0.05f;

// 全局优化是否单独开线程。loop closure global adjustment runs on a separate thread
static const bool separateThreadGlobalAdjustment = true;

template <typename TVoxel, typename TIndex>
ITMMultiEngine<TVoxel, TIndex>::ITMMultiEngine(const ITMLibSettings *settings, const ITMRGBDCalib &calib,
                                               Vector2i imgSize_rgb, Vector2i imgSize_d) {
  // 深度图和彩色图的尺寸
  if ((imgSize_d.x == -1) || (imgSize_d.y == -1)) imgSize_d = imgSize_rgb;
  // 系统设置
  this->settings = settings;
  // 设备类型: CPU、GPU、Metal
  const ITMLibSettings::DeviceType deviceType = settings->deviceType;
  // 底层的图像处理模块（拷贝、彩色转灰色等操作，不是预处理）
  lowLevelEngine = ITMLowLevelEngineFactory::MakeLowLevelEngine(deviceType);
  // 输入图像预处理模块
  viewBuilder = ITMViewBuilderFactory::MakeViewBuilder(calib, deviceType);
  // 渲染（可视化）
  visualisationEngine = ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);
  multiVisualisationEngine = ITMMultiVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(deviceType);
  renderState_multiscene = NULL;
  // mesh
  meshingEngine = NULL;
  if (settings->createMeshingEngine)
    meshingEngine = ITMMultiMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(deviceType);
  // 渲染结果：自由视角
  renderState_freeview = NULL; //will be created by the visualisation engine  // TODO: 没有renderState_live？？？
  // 负责 场景三维模型的融合 && swap in/out
  denseMapper = new ITMDenseMapper<TVoxel, TIndex>(settings);
  // IMU预积分器
  imuCalibrator = new ITMIMUCalibrator_iPad();
  // 跟踪
  tracker = ITMTrackerFactory::Instance().Make(imgSize_rgb, imgSize_d, settings, lowLevelEngine, imuCalibrator,
                                               &settings->sceneParams);
  trackingController = new ITMTrackingController(tracker, settings);
  // 获取用于跟踪的图像大小   // TODO: 是因为金字塔导致用于跟踪的图片大小可能会更小吗？
  trackedImageSize = trackingController->GetTrackedImageSize(imgSize_rgb, imgSize_d);
  // TODO(xzf)
  freeviewLocalMapIdx = 0;
  mapManager =
      new ITMVoxelMapGraphManager<TVoxel, TIndex>(settings, visualisationEngine, denseMapper, trackedImageSize);
  mActiveDataManager = new ITMActiveMapManager(mapManager);
  mActiveDataManager->initiateNewLocalMap(true);

  //TODO	tracker->UpdateInitialPose(allData[0]->trackingState);

  view = NULL; // 当前输入图像。will be allocated by the view builder
  // 重定位： 随机蕨
  relocaliser = new FernRelocLib::Relocaliser<float>(
      imgSize_d, Vector2f(settings->sceneParams.viewFrustum_min, settings->sceneParams.viewFrustum_max), 0.1f, 1000, 4);
  // 全局优化
  mGlobalAdjustmentEngine = new ITMGlobalAdjustmentEngine();
  mScheduleGlobalAdjustment = false;  // 是否进行全局优化
  if (separateThreadGlobalAdjustment) mGlobalAdjustmentEngine->startSeparateThread(); // 给全局优化开单独的线程
}

template<typename TVoxel, typename TIndex>
ITMMultiEngine<TVoxel, TIndex>::~ITMMultiEngine(void) {
  if (renderState_multiscene != NULL) delete renderState_multiscene;

  delete mGlobalAdjustmentEngine;
  delete mActiveDataManager;
  delete mapManager;

  if (renderState_freeview != NULL) delete renderState_freeview;

  delete denseMapper;
  delete trackingController;

  delete tracker;
  delete imuCalibrator;

  delete lowLevelEngine;
  delete viewBuilder;

  if (view != NULL) delete view;

  delete visualisationEngine;

  delete relocaliser;

  delete multiVisualisationEngine;
}

template<typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::changeFreeviewLocalMapIdx(ORUtils::SE3Pose *pose, int newIdx) {
  //if ((newIdx < 0) || ((unsigned)newIdx >= mapManager->numLocalMaps())) return;
  //! 保证id的有效性
  if (newIdx < -1) newIdx = (int) mapManager->numLocalMaps() - 1;
  if ((unsigned) newIdx >= mapManager->numLocalMaps()) newIdx = -1;
  //! 计算 新子图 到 相机 的位姿
  ORUtils::SE3Pose trafo = mapManager->findTransformation(freeviewLocalMapIdx, newIdx); // 旧子图 到 新的位姿，即T_new_old
  pose->SetM(pose->GetM() * trafo.GetInvM()); // T_cam_new = T_cam_old * (T_new_old)^-1
  pose->Coerce();
  freeviewLocalMapIdx = newIdx;
}

template<typename TVoxel, typename TIndex>
ITMTrackingState *ITMMultiEngine<TVoxel, TIndex>::GetTrackingState(void) {
  int idx = mActiveDataManager->findPrimaryLocalMapIdx();
  if (idx < 0) idx = 0;
  return mapManager->getLocalMap(idx)->trackingState;
}

/**
 * 
 - whenever a new local scene is added, add to list of "to be established 3D relations"
 - whenever a relocalisation is detected, add to the same list, preserving any existing information on that 3D relation

 - for all 3D relations to be established :
 - attempt tracking in both scenes
 - if success, add to list of new candidates
 - if less than n_overlap "new candidates" in more than n_reloctrialframes frames, discard
 - if at least n_overlap "new candidates" :
 	- try to compute 3D relation, weighting old information accordingly
	- if outlier ratio below p_relation_outliers and at least n_overlap inliers, success
*/
/** 记录要每个子图的处理内容。每次跟踪，都要处理多个子图。*/
struct TodoListEntry {
  TodoListEntry(int _activeDataID, bool _track, bool _fusion, bool _prepare)
      : dataId(_activeDataID), track(_track), fusion(_fusion), prepare(_prepare), preprepare(false) {}
  TodoListEntry(void) {}
  int dataId;       // 子图的活跃id。=-1表示不处理子图、处理回环检测
  bool track;       // 是否要跟踪
  bool fusion;      // 是否要fusion
  bool prepare;     // 是否要raycast（为下一帧准备）
  bool preprepare;  // 重定位成功后，需要先raycast（为当前帧准备）
};

template <typename TVoxel, typename TIndex>
ITMTrackingState::TrackingResult ITMMultiEngine<TVoxel, TIndex>::ProcessFrame(ITMUChar4Image *rgbImage,
                                                                              ITMShortImage *rawDepthImage,
                                                                              ITMIMUMeasurement *imuMeasurement) {
  ITMTrackingState::TrackingResult primaryLocalMapTrackingResult; // 主子图的跟踪结果

  //! 准备数据：对输入数据预处理后，放到view中。prepare image and turn it into a depth image
  if (imuMeasurement == NULL)   // 无IMU
    viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter);
  else                          // 有IMU
    viewBuilder->UpdateView(&view, rgbImage, rawDepthImage, settings->useBilateralFilter, imuMeasurement);
  
  //! 准备todo list：包含主子图，以及新来、回环、重定位的子图，还有重定位
  std::vector<TodoListEntry> todoList;                            // 每个子图的处理内容。每次跟踪，都要处理多个子图。
  // 把主子图添加到todo list。find primary data, if available 
  int primaryDataIdx = mActiveDataManager->findPrimaryDataIdx();  // 主子图的活跃id
  if (primaryDataIdx >= 0)  // if there is a "primary data index", process it
    todoList.push_back(TodoListEntry(primaryDataIdx, true, true, true));

  // after primary local map, make sure to process all relocalisations, new scenes and loop closures
  // 把新来、回环、重定位的子图添加到todo list
  for (int i = 0; i < mActiveDataManager->numActiveLocalMaps(); ++i) {
    switch (mActiveDataManager->getLocalMapType(i)) {
    case ITMActiveMapManager::NEW_LOCAL_MAP:  // 新来的子图
      todoList.push_back(TodoListEntry(i, true, true, true));
    case ITMActiveMapManager::LOOP_CLOSURE:   // 回环
      todoList.push_back(TodoListEntry(i, true, false, true));
    case ITMActiveMapManager::RELOCALISATION: // 重定位
      todoList.push_back(TodoListEntry(i, true, false, true));
    default:
      break;
    }
  }

  // 调用回环检测 ？？？finally, once all is done, call the loop closure detection engine
  todoList.push_back(TodoListEntry(-1, false, false, false));

  //! 处理todo list
  bool primaryTrackingSuccess = false;            // 主子图是否跟踪成功
  for (size_t i = 0; i < todoList.size(); ++i) {  // NOTE: 必须要用todoList.size()，因为todoList的长度会变
    // - first pass of the todo list is for primary local map and ongoing relocalisation and loopclosure attempts
    // - an element with id -1 marks the end of the first pass, a request to call the loop closure detection engine, and
    // the start of the second pass
    // - second tracking pass will be about newly detected loop closures, relocalisations, etc.
    //! 处理回环检测
    if (todoList[i].dataId == -1) { // NOTE: todoList的最后表示回环检测任务
#ifdef DEBUG_MULTISCENE
      fprintf(stderr, " Reloc(%i)", primaryTrackingSuccess);
#endif
      int NN[k_loopcloseneighbours];          // ?主子图的最近邻居
      float distances[k_loopcloseneighbours]; // ?主子图到最近邻居的距离
      view->depth->UpdateHostFromDevice();    // 数据转移到CPU

      // 获取主子图的全局子图id。primary map index
      int primaryLocalMapIdx = -1;
      if (primaryDataIdx >= 0)
        primaryLocalMapIdx = mActiveDataManager->getLocalMapIndex(primaryDataIdx);

      // 利用主子图重定位。check if relocaliser has fired
      ORUtils::SE3Pose *pose =    // 获取主子图的位姿
          primaryLocalMapIdx >= 0 ? mapManager->getLocalMap(primaryLocalMapIdx)->trackingState->pose_d : NULL;
      bool hasAddedKeyframe = relocaliser->ProcessFrame(view->depth, pose, primaryLocalMapIdx, k_loopcloseneighbours,
                                                        NN, distances, primaryTrackingSuccess);

      // 主子图跟踪失败，则上面重定位不准，把最近邻居都变成活跃子图（如果可以的话）
      // frame not added and tracking failed -> we need to relocalise
      if (!hasAddedKeyframe) {
        for (int j = 0; j < k_loopcloseneighbours; ++j) {
          if (distances[j] > F_maxdistattemptreloc)
            continue;
          const FernRelocLib::PoseDatabase::PoseInScene &keyframe = relocaliser->RetrievePose(NN[j]);
          int newDataIdx =
              mActiveDataManager->initiateNewLink(keyframe.sceneIdx, keyframe.pose, (primaryLocalMapIdx < 0));
          if (newDataIdx >= 0) {  // 添加成功
            TodoListEntry todoItem(newDataIdx, true, false, true);
            todoItem.preprepare = true;
            todoList.push_back(todoItem); // 重定位成功的要跟踪一下，但是不fusion
          }
        }
      }

      continue;
    }
    // 获取当前子图
    ITMLocalMap<TVoxel, TIndex> *currentLocalMap = NULL;                                // 正在处理的子图
    int currentLocalMapIdx = mActiveDataManager->getLocalMapIndex(todoList[i].dataId);  // 正在处理的子图的全局id
    currentLocalMap = mapManager->getLocalMap(currentLocalMapIdx);

    // if a new relocalisation/loopclosure is started, this will do the initial raycasting before tracking can start
    //! 重定位成功后，需要先raycast一下，后续才能track
    if (todoList[i].preprepare) {
      denseMapper->UpdateVisibleList(view, currentLocalMap->trackingState, currentLocalMap->scene,
                                     currentLocalMap->renderState);   // 更新可见列表
      trackingController->Prepare(currentLocalMap->trackingState, currentLocalMap->scene, view, visualisationEngine,
                                  currentLocalMap->renderState);      // raycast
    }
    //! 跟踪
    if (todoList[i].track) {
      int dataId = todoList[i].dataId;  // 正在处理的子图的活跃id

#ifdef DEBUG_MULTISCENE
      int blocksInUse = currentLocalMap->scene->index.getNumAllocatedVoxelBlocks() -
                        currentLocalMap->scene->localVBA.lastFreeBlockId - 1;
      fprintf(stderr, " %i%s (%i)", currentLocalMapIdx, (todoList[i].dataId == primaryDataIdx) ? "*" : "", blocksInUse);
#endif

      // 跟踪单帧。actual tracking
      ORUtils::SE3Pose oldPose(*(currentLocalMap->trackingState->pose_d));  // 子图中旧的相机位姿。T_ls
      trackingController->Track(currentLocalMap->trackingState, view);

      // 除了主子图，其他子图的跟踪结果为Poor直接算成fail。tracking is allowed to be poor only in the primary scenes.
      ITMTrackingState::TrackingResult trackingResult = currentLocalMap->trackingState->trackerResult;  // 跟踪结果
      if (mActiveDataManager->getLocalMapType(dataId) != ITMActiveMapManager::PRIMARY_LOCAL_MAP)
        if (trackingResult == ITMTrackingState::TRACKING_POOR)
          trackingResult = ITMTrackingState::TRACKING_FAILED;

      // actions on tracking result for all scenes // TODO: incorporate behaviour on tracking failure from settings
      // 跟踪不好，就不fusion
      if (trackingResult != ITMTrackingState::TRACKING_GOOD)
        todoList[i].fusion = false;
      // 跟踪失败，就不raycast
      if (trackingResult == ITMTrackingState::TRACKING_FAILED) {  // NOTE: 只有主子图允许track poor
        todoList[i].prepare = false;
        *(currentLocalMap->trackingState->pose_d) = oldPose;
      }

      // 处理主子图的跟踪结果。actions on tracking result for primary local map
      if (mActiveDataManager->getLocalMapType(dataId) == ITMActiveMapManager::PRIMARY_LOCAL_MAP) {
        primaryLocalMapTrackingResult = trackingResult;

        if (trackingResult == ITMTrackingState::TRACKING_GOOD)              // 主子图跟踪good
          primaryTrackingSuccess = true;
        // we need to relocalise in the primary local map
        else if (trackingResult == ITMTrackingState::TRACKING_FAILED) {     // 主子图跟踪失败，准备后续用主子图重定位
          primaryDataIdx = -1;
          todoList.resize(i + 1);                                     // 后面的todo全不要了
          todoList.push_back(TodoListEntry(-1, false, false, false)); // 添加回环的todo
        }                                                                   // 主子图跟踪为poor，跟踪失败，但是要raycast
      }

      // 记录跟踪结果
      mActiveDataManager->recordTrackingResult(dataId, trackingResult, primaryTrackingSuccess);
    }

    // fusion in any subscene as long as tracking is good for the respective subscene
    //! 只有跟踪成功才fusion，否则就只更新可见列表 为后续raycast准备。
    if (todoList[i].fusion)
      denseMapper->ProcessFrame(view, currentLocalMap->trackingState, currentLocalMap->scene,
                                currentLocalMap->renderState);
    else if (todoList[i].prepare)
      denseMapper->UpdateVisibleList(view, currentLocalMap->trackingState, currentLocalMap->scene,
                                     currentLocalMap->renderState);

    //! 只有跟踪成功 or 主子图跟踪poor 才raycast，给 下一帧跟踪 以及 可视化用。
    // raycast to renderState_live for tracking and free visualisation
    if (todoList[i].prepare)
      trackingController->Prepare(currentLocalMap->trackingState, currentLocalMap->scene, view, visualisationEngine,
                                  currentLocalMap->renderState);
  }
  //! 全局优化
  mScheduleGlobalAdjustment |= mActiveDataManager->maintainActiveData();  // 活跃子图发生较大变化，就要进行全局优化
  if (mScheduleGlobalAdjustment) {  
    if (mGlobalAdjustmentEngine->updateMeasurements(*mapManager)) {       // 更新位姿图
      if (separateThreadGlobalAdjustment) // 并行的全局优化
        mGlobalAdjustmentEngine->wakeupSeparateThread();
      else                                // 串行的全局优化
        mGlobalAdjustmentEngine->runGlobalAdjustment();

      mScheduleGlobalAdjustment = false;
    }
  }
  mGlobalAdjustmentEngine->retrieveNewEstimates(*mapManager);

  return primaryLocalMapTrackingResult;
}

template<typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::SaveSceneToMesh(const char *modelFileName) {
  if (meshingEngine == NULL) return;

  ITMMesh *mesh = new ITMMesh(settings->GetMemoryType());

  meshingEngine->MeshScene(mesh, *mapManager);
  mesh->WriteSTL(modelFileName);

  delete mesh;
}

template<typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::SaveToFile() {

}

template<typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::LoadFromFile() {

}

template<typename TVoxel, typename TIndex>
Vector2i ITMMultiEngine<TVoxel, TIndex>::GetImageSize(void) const {
  return trackedImageSize;
}

template <typename TVoxel, typename TIndex>
void ITMMultiEngine<TVoxel, TIndex>::GetImage(ITMUChar4Image *out, GetImageType getImageType, ORUtils::SE3Pose *pose,
                                              ITMIntrinsics *intrinsics) {
  if (view == NULL)
    return;

  out->Clear();
  //! 根据所需的图片类型不同，渲染不同的图片
  printf("[INFO] getImageType = %d\n", getImageType);
  switch (getImageType) {
  // NOTE: 以下都是直接来自输入图片
  case ITMMultiEngine::InfiniTAM_IMAGE_ORIGINAL_RGB:                            // 输入的彩色图
    out->ChangeDims(view->rgb->noDims);
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
    else
      out->SetFrom(view->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    break;
  case ITMMultiEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH:                          // 输入的深度图
    out->ChangeDims(view->depth->noDims);
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      view->depth->UpdateHostFromDevice();
    ITMVisualisationEngine<TVoxel, TIndex>::DepthToUchar4(out, view->depth);
    break;
  // NOTE: 以下都是固定视角的图片
  case ITMMultiEngine::InfiniTAM_IMAGE_SCENERAYCAST:
  case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME: // TODO: add colour rendering
  case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:
  case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE: {
    // 设置渲染用的子图
    int visualisationLocalMapIdx = mActiveDataManager->findBestVisualisationLocalMapIdx();  // 找到用于可视化的活跃子图id
    if (visualisationLocalMapIdx < 0)
      break; // TODO: clear image? what else to do when tracking is lost?
    ITMLocalMap<TVoxel, TIndex> *activeLocalMap = mapManager->getLocalMap(visualisationLocalMapIdx);  // 子图
    // 设置渲染类型。
    IITMVisualisationEngine::RenderRaycastSelection raycastType;
    if (activeLocalMap->trackingState->age_pointCloud <= 0) // 直接使用旧的普通raycast结果
      raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_RAYCAST;
    else                                                    // 直接使用旧的增量raycat结果
      raycastType = IITMVisualisationEngine::RENDER_FROM_OLD_FORWARDPROJ;
    // 设置渲染图片类型。
    IITMVisualisationEngine::RenderImageType imageType;
    switch (getImageType) {
    case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE:                  // 三维场景的置信度的伪彩色图
      imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;
      break;
    case ITMMultiEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL:                      // 三维场景的单位法向量的伪彩色图
      imageType = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL; 
      break;
    default:                                                                      // 有序点云的法向量夹角图（灰色）
      imageType = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS;
      // TODO: 比ITMBasicEngine缺少了InfiniTAM_IMAGE_COLOUR_FROM_VOLUME。为啥缺少呢？？又不是用的ITMMultiVisualisationEngine
    }
    // 渲染图片
    visualisationEngine->RenderImage(activeLocalMap->scene, activeLocalMap->trackingState->pose_d,
                                     &view->calib.intrinsics_d, activeLocalMap->renderState,
                                     activeLocalMap->renderState->raycastImage, imageType, raycastType);
    // 把渲染的结果转移到out
    ORUtils::Image<Vector4u> *srcImage = activeLocalMap->renderState->raycastImage;
    out->ChangeDims(srcImage->noDims);  // 修改图片（内存）大小
    if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
      out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
    else
      out->SetFrom(srcImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    break;
  }
  // NOTE: 以下都是自由视角的图片
  case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED:
  case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME:
  case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL:
  case ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE: {
    // 默认是法向量夹角图（灰度）
    IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE;
    if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME)          // 三维场景的彩色图
      type = IITMVisualisationEngine::RENDER_COLOUR_FROM_VOLUME;
    else if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL)     // 单位法向量的伪彩色图
      type = IITMVisualisationEngine::RENDER_COLOUR_FROM_NORMAL;
    else if (getImageType == ITMMultiEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE) // 置信度的伪彩色图
      type = IITMVisualisationEngine::RENDER_COLOUR_FROM_CONFIDENCE;

    if (freeviewLocalMapIdx >= 0) { // 显示单个子图
      ITMLocalMap<TVoxel, TIndex> *activeData = mapManager->getLocalMap(freeviewLocalMapIdx);   // 要显示的子图
      if (renderState_freeview == NULL) 
        renderState_freeview = visualisationEngine->CreateRenderState(activeData->scene, out->noDims);
      // NOTE: 因为现在是自由视角，所以需要在当前视角下重新raycast，不能用跟踪里的raycast结果
      // raycast三部曲：找可见block、确定ray的搜索范围、渲染图片
      visualisationEngine->FindVisibleBlocks(activeData->scene, pose, intrinsics, renderState_freeview);
      visualisationEngine->CreateExpectedDepths(activeData->scene, pose, intrinsics, renderState_freeview);
      visualisationEngine->RenderImage(activeData->scene, pose, intrinsics, renderState_freeview,
                                       renderState_freeview->raycastImage, type);

      if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
        out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
      else
        out->SetFrom(renderState_freeview->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    } else {                        // 显示所有子图     
      if (renderState_multiscene == NULL)
        renderState_multiscene =
            multiVisualisationEngine->CreateRenderState(mapManager->getLocalMap(0)->scene, out->noDims);
      multiVisualisationEngine->PrepareRenderState(*mapManager, renderState_multiscene);        // 获取所有的子图信息
      multiVisualisationEngine->CreateExpectedDepths(pose, intrinsics, renderState_multiscene);
      multiVisualisationEngine->RenderImage(pose, intrinsics, renderState_multiscene,       // TODO: 下次从这儿开始
                                            renderState_multiscene->raycastImage, type);
      if (settings->deviceType == ITMLibSettings::DEVICE_CUDA)
        out->SetFrom(renderState_multiscene->raycastImage, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
      else
        out->SetFrom(renderState_multiscene->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
    }

    break;
  }
  case ITMMultiEngine::InfiniTAM_IMAGE_UNKNOWN:
    break;
  };
}
