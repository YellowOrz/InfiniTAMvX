// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMVisualisationEngine.h"

struct RenderingBlock;

namespace ITMLib {
template <class TVoxel, class TIndex>
class ITMVisualisationEngine_CUDA : public ITMVisualisationEngine<TVoxel, TIndex> {
private:
  uint *noTotalPoints_device;

public:
  explicit ITMVisualisationEngine_CUDA(void);
  ~ITMVisualisationEngine_CUDA(void);
  /** new一个新的ITMRenderState */
  ITMRenderState *CreateRenderState(const ITMScene<TVoxel, TIndex> *scene, const Vector2i &imgSize) const;
  /**
   * @brief 根据当前视角，找到可见的entry（voxel block）
   * @param[in] scene 三维场景信息。
   * @param[in] pose 当前相机位姿。world to local
   * @param[in] intrinsics 相机内参，用于投影图片
   * @param[out] renderState 渲染相关变量。
   */
  void FindVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  /**
   * @brief 统计可见的entry列表中，block id在指定范围内的数量
   * @param[in] scene       三维场景信息。主要用到hash table
   * @param[in] renderState 渲染信息。主要用到GetVisibleEntryIDs
   * @param[in] minBlockId  指定id范围
   * @param[in] maxBlockId  指定id范围
   * @return int 指定范围内的entry数量
   * @note 主要用于Multi Scene中
   */
  int CountVisibleBlocks(const ITMScene<TVoxel, TIndex> *scene, const ITMRenderState *renderState, int minBlockId,
                         int maxBlockId) const;
  /**
   * @brief 将raycasting中每条ray的最小和最大深度（即搜索范围）设置为常数。用来辅助后面 更快地投影 图像
   * @param[in] scene         三维场景信息。主要用到其中的voxel size和hash table
   * @param[in] pose          当前相机位姿。world to local
   * @param[in] intrinsics    相机内参，用于投影图片
   * @param[out] renderState  渲染相关变量。主要用到其中的renderingRangeImage，来记录raycasting中每条ray的深度范围
   * @note 用于UI界面可视化（自由视角） && 跟踪投影的raycast
   */
  void CreateExpectedDepths(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  /**
   * @brief 根据渲染类型，从raycast得到的点云中得到图片
   * @param[in] scene         三维场景。主要用到其中的voxel size和hash table
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[in] renderState   raycast的结果，主要用到其中的raycastResult
   * @param[out] outputImage  渲染得到的图片
   * @param[in] type          渲染类型
   * @param[in] raycastType   raycast的类型
   */
  void RenderImage(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                   const ITMRenderState *renderState, ITMUChar4Image *outputImage,
                   IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
                   IITMVisualisationEngine::RenderRaycastSelection raycastType =
                       IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
  /**
   * @brief 使用raycast来找到当前视角下相机看到的表面
   * @param[in] scene         三维场景
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[out] renderState  raycast的结果，主要用到其中的raycastResult
   * @note 除了surfel，其他没地方用到
   */
  void FindSurface(const ITMScene<TVoxel, TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
                   const ITMRenderState *renderState) const;
  /**
   * 根据跟踪到的当前相机位姿，使用raycast从三维场景中抽取带RGB的点云（用于下一帧的跟踪？？？）
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene               三维场景
   * @param[in] view                当前输入帧。主要用到其中的相机内外参
   * @param[in, out] trackingState  跟踪状态。主要用到其中的相机位姿、点云
   * @param[in, out] renderState    渲染结果。主要用到其中的raycastResult
   * @param[in] skipPoints          抽取点云的时候要不要跳过。=true的话，只保留1/4的点云
   */
  void CreatePointCloud(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                        ITMRenderState *renderState, bool skipPoints) const;
  /**
   * voxel三维场景中，使用raycasting投影点云（只有几何，无纹理）？？？
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene         三维场景信息
   * @param[in] view          当前输入图像
   * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
   * @param[out] renderState  raycasting的结果
   */
  void CreateICPMaps(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                     ITMRenderState *renderState) const;
  /**
   * 增量式的raycasting
   * @details 找出旧的raycast结果中在当前视角下找不到的，重新raycast
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene             三维场景信息
   * @param[in] view              当前帧。用到其中的深度图
   * @param[in] trackingState     当前帧的跟踪结果。主要用到当前深度相机的位姿
   * @param[in, out] renderState  渲染相关信息。主要用到其中的raycastResult、renderingRangeImage、forwardProjection、
   *                              fwdProjMissingPoints。前俩是in，后俩是out
   */
  void ForwardRender(const ITMScene<TVoxel, TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
                     ITMRenderState *renderState) const;
};

template <class TVoxel>
class ITMVisualisationEngine_CUDA<TVoxel, ITMVoxelBlockHash>
    : public ITMVisualisationEngine<TVoxel, ITMVoxelBlockHash> {
private:
  uint *noTotalPoints_device;                 // ???(在GPU上的临时存放)
  RenderingBlock *renderingBlockList_device;  // render小块(在GPU上的临时存放)
  uint *noTotalBlocks_device;                 // 可见entries数量(在GPU上的临时存放)
  int *noVisibleEntries_device;               // 可见entries列表(在GPU上的临时存放)

public:
  explicit ITMVisualisationEngine_CUDA(void);
  ~ITMVisualisationEngine_CUDA(void);
  /** new一个新的ITMRenderState */
  ITMRenderState_VH *CreateRenderState(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const Vector2i &imgSize) const;
  /**
   * @brief 根据当前视角，找到可见的entry（voxel block）
   * @param[in] scene 三维场景信息。
   * @param[in] pose 当前相机位姿。world to local
   * @param[in] intrinsics 相机内参，用于投影图片
   * @param[out] renderState 渲染相关变量。
   */
  void FindVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                         const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  /**
   * @brief 统计可见的entry列表中，block id在指定范围内的数量
   * @param[in] scene       三维场景信息。主要用到hash table
   * @param[in] renderState 渲染信息。主要用到GetVisibleEntryIDs
   * @param[in] minBlockId  指定id范围
   * @param[in] maxBlockId  指定id范围
   * @return int 指定范围内的entry数量
   * @note 主要用于Multi Scene中
   */
  int CountVisibleBlocks(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMRenderState *renderState,
                         int minBlockId, int maxBlockId) const;
  /**
   * @brief 将raycasting中每条ray的最小和最大深度（即搜索范围）设置为常数。用来辅助后面 更快地投影 图像
   * @param[in] scene         三维场景信息。主要用到其中的voxel size和hash table
   * @param[in] pose          当前相机位姿。world to local
   * @param[in] intrinsics    相机内参，用于投影图片
   * @param[out] renderState  渲染相关变量。主要用到其中的renderingRangeImage，来记录raycasting中每条ray的深度范围
   * @note 用于UI界面可视化（自由视角） && 跟踪投影的raycast
   */
  void CreateExpectedDepths(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                            const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
  /**
   * @brief 根据渲染类型，从raycast得到的点云中得到图片
   * @param[in] scene         三维场景。主要用到其中的voxel size和hash table
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[in] renderState   raycast的结果，主要用到其中的raycastResult
   * @param[out] outputImage  渲染得到的图片
   * @param[in] type          渲染类型
   * @param[in] raycastType   raycast的类型
   */
  void RenderImage(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                   const ITMIntrinsics *intrinsics, const ITMRenderState *renderState, ITMUChar4Image *outputImage,
                   IITMVisualisationEngine::RenderImageType type = IITMVisualisationEngine::RENDER_SHADED_GREYSCALE,
                   IITMVisualisationEngine::RenderRaycastSelection raycastType =
                       IITMVisualisationEngine::RENDER_FROM_NEW_RAYCAST) const;
  /**
   * @brief 使用raycast来找到当前视角下相机看到的表面
   * @param[in] scene         三维场景
   * @param[in] pose          当前视角的相机位姿。world to local
   * @param[in] intrinsics    当前视角的相机参数，用于投影图片
   * @param[out] renderState  raycast的结果，主要用到其中的raycastResult
   * @note 除了surfel，其他没地方用到
   */
  void FindSurface(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose,
                   const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
  /**
   * 根据跟踪到的当前相机位姿，使用raycast从三维场景中抽取带RGB的点云（用于下一帧的跟踪？？？）
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene               三维场景
   * @param[in] view                当前输入帧。主要用到其中的相机内外参
   * @param[in, out] trackingState  跟踪状态。主要用到其中的相机位姿、点云
   * @param[in, out] renderState    渲染结果。主要用到其中的raycastResult
   * @param[in] skipPoints          抽取点云的时候要不要跳过。=true的话，只保留1/4的点云
   */
  void CreatePointCloud(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                        ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
  /**
   * voxel三维场景中，使用raycasting投影点云（只有几何，无纹理）？？？
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene         三维场景信息
   * @param[in] view          当前输入图像
   * @param[in] trackingState 包含跟踪得到的相机位姿、跟踪的分数等
   * @param[out] renderState  raycasting的结果
   */
  void CreateICPMaps(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                     ITMTrackingState *trackingState, ITMRenderState *renderState) const;
  /**
   * 增量式的raycasting
   * @details 找出旧的raycast结果中在当前视角下找不到的，重新raycast
   * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
   * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
   * @param[in] scene             三维场景信息
   * @param[in] view              当前帧。用到其中的深度图
   * @param[in] trackingState     当前帧的跟踪结果。主要用到当前深度相机的位姿
   * @param[in, out] renderState  渲染相关信息。主要用到其中的raycastResult、renderingRangeImage、forwardProjection、
   *                              fwdProjMissingPoints。前俩是in，后俩是out
   */
  void ForwardRender(const ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view,
                     ITMTrackingState *trackingState, ITMRenderState *renderState) const;
};
} // namespace ITMLib
