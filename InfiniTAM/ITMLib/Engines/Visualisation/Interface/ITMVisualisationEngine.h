// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
#include "../../../Objects/Scene/ITMScene.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib {
class IITMVisualisationEngine {
public:
  enum RenderImageType {
    RENDER_SHADED_GREYSCALE,              // 渲染三维场景的法向量夹角图（灰度）
    RENDER_SHADED_GREYSCALE_IMAGENORMALS, // 渲染有序点云的法向量夹角图（灰色）
    RENDER_COLOUR_FROM_VOLUME,            // 渲染三维场景的彩色图
    RENDER_COLOUR_FROM_NORMAL,            // 渲染三维场景的单位法向量的伪彩色图
    RENDER_COLOUR_FROM_CONFIDENCE         // 渲染三维场景的置信度的伪彩色图
  };

  enum RenderRaycastSelection { RENDER_FROM_NEW_RAYCAST, RENDER_FROM_OLD_RAYCAST, RENDER_FROM_OLD_FORWARDPROJ };

  virtual ~IITMVisualisationEngine(void) {}

  static void DepthToUchar4(ITMUChar4Image *dst, const ITMFloatImage *src);
  static void NormalToUchar4(ITMUChar4Image *dst, const ITMFloat4Image *src);
  static void WeightToUchar4(ITMUChar4Image *dst, const ITMFloatImage *src);
};

template <class TIndex> struct IndexToRenderState {
  typedef ITMRenderState type;
};
template <> struct IndexToRenderState<ITMVoxelBlockHash> {
  typedef ITMRenderState_VH type;
};

/** \brief
    Interface to engines helping with the visualisation of the results from the rest of the library.

    This is also used internally to get depth estimates for the raycasting done for the trackers. The basic idea there is to project down a scene of 8x8x8 voxel blocks and look at the bounding boxes. The projection provides an idea of the possible depth range for each pixel in an image, which can be used to speed up raycasting operations.
    */
template <class TVoxel, class TIndex> class ITMVisualisationEngine : public IITMVisualisationEngine {
public:
  /** new一个新的ITMRenderState。Creates a render state, containing rendering info for the scene. */
  virtual typename IndexToRenderState<TIndex>::type *CreateRenderState(const ITMScene<TVoxel, TIndex> *scene,
                                                                       const Vector2i &imgSize) const = 0;
  /**
   * @brief 根据当前视角，找到可见的entry（voxel block）
   * Given a scene, pose and intrinsics, compute the visible subset of the scene and store it in an appropriate visualisation state object, created previously using allocateInternalState().
   * @param[in] scene 三维场景信息。
   * @param[in] pose 当前相机位姿。world to local
   * @param[in] intrinsics 相机内参，用于投影图片
   * @param[out] renderState 渲染相关变量。
   */
  virtual void FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                                 const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const = 0;
  /**
   * @brief 统计可见的entry列表中，block id在指定范围内的数量
   * Given a render state, Count the number of visible blocks with minBlockId <= blockID <= maxBlockId .
   * @param[in] scene       三维场景信息。主要用到hash table
   * @param[in] renderState 渲染信息。主要用到GetVisibleEntryIDs
   * @param[in] minBlockId  指定id范围
   * @param[in] maxBlockId  指定id范围
   * @return int 指定范围内的entry数量
   * @note 主要用于Multi Scene中
   */
  virtual int CountVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMRenderState *renderState,
                                 int minBlockId = 0, int maxBlockId = SDF_LOCAL_BLOCK_NUM) const = 0;
  /**
   * @brief 将raycasting中每条ray的最小和最大深度（即搜索范围）设置为常数。用来辅助后面 更快地投影 图像
   * Given scene, pose and intrinsics, create an estimate of the minimum and maximum depths at each pixel of an image.
   * @param[in] scene         三维场景信息。主要用到其中的voxel size和hash table
   * @param[in] pose          当前相机位姿。world to local
   * @param[in] intrinsics    相机内参，用于投影图片
   * @param[out] renderState  渲染相关变量。主要用到其中的renderingRangeImage，来记录raycasting中每条ray的深度范围
   * @note 用于UI界面可视化（自由视角） && 跟踪投影的raycast
   */
  virtual void CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                                    const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const = 0;
  /**
   * @brief 根据渲染类型，从raycast得到的点云中得到图片。This will render an image using raycasting
   * @param[in] scene         三维场景。主要用到其中的voxel size和hash table
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[in] renderState   raycast的结果，主要用到其中的raycastResult
   * @param[out] outputImage  渲染得到的图片
   * @param[in] type          渲染类型
   * @param[in] raycastType   raycast的类型
   */
  virtual void RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                           const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                           ITMUChar4Image *outputImage, RenderImageType type = RENDER_SHADED_GREYSCALE,
                           RenderRaycastSelection raycastType = RENDER_FROM_NEW_RAYCAST) const = 0;
  /**
   * @brief 使用raycast来找到当前视角下相机看到的表面。Finds the scene surface using raycasting. 
   * @param[in] scene         三维场景
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[out] renderState  raycast的结果，主要用到其中的raycastResult
   * @note 除了surfel，其他没地方用到
   */
  virtual void FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                           const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const = 0;
  /**
   * 根据跟踪到的当前相机位姿，使用raycast从三维场景中抽取带RGB的点云（用于下一帧的跟踪？？？）
   * Create a point cloud as required by the ITMLib::Engine::ITMColorTracker classes.
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene               三维场景
   * @param[in] view                当前输入帧。主要用到其中的相机内外参
   * @param[in, out] trackingState  跟踪状态。主要用到其中的相机位姿、点云
   * @param[in, out] renderState    渲染结果。主要用到其中的raycastResult
   * @param[in] skipPoints          抽取点云的时候要不要跳过。=true的话，只保留1/4的点云
   */
  virtual void CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                                ITMTrackingState *trackingState, ITMRenderState *renderState,
                                bool skipPoints) const = 0;
  /**
   * voxel三维场景中，使用raycasting投影点云（只有几何，无纹理）？？？
   * Create an image of reference points and normals as required by the ITMLib::Engine::ITMDepthTracker classes.
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene         三维场景信息
   * @param[in] view          当前输入图像
   * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
   * @param[out] renderState  raycasting的结果
   */
  virtual void CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                             ITMTrackingState *trackingState, ITMRenderState *renderState) const = 0;
  /**
   * 增量式的raycasting
   * Create an image of reference points and normals as required by the ITMLib::Engine::ITMDepthTracker classes. Incrementally previous raycast result.
   * @details 找出旧的raycast结果中在当前视角下找不到的，重新raycast
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene             三维场景信息
   * @param[in] view              当前帧。用到其中的深度图
   * @param[in] trackingState     当前帧的跟踪结果。主要用到当前深度相机的位姿
   * @param[in, out] renderState  渲染相关信息。主要用到其中的raycastResult、renderingRangeImage、forwardProjection、
   *                              fwdProjMissingPoints。前俩是in，后俩是out
   */
  virtual void ForwardRender(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view,
                             ITMTrackingState *trackingState, ITMRenderState *renderState) const = 0;
};
} // namespace ITMLib
