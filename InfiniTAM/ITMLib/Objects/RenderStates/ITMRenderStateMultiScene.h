// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Engines/MultiScene/ITMMapGraphManager.h"
#include "../Scene/ITMMultiSceneAccess.h"
#include "../../Objects/RenderStates/ITMRenderState.h"

namespace ITMLib {

template<class TVoxel, class TIndex>
class ITMRenderStateMultiScene : public ITMRenderState {
 private:
  MemoryDeviceType memoryType;

 public:
  typedef typename ITMMultiIndex<TIndex>::IndexData MultiIndexData;
  typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
  typedef ITMVoxelMapGraphManager<TVoxel, TIndex> MultiSceneManager;

#ifndef COMPILE_WITHOUT_CUDA
  MultiIndexData *indexData_device;   // 在GPU上的存档
  MultiVoxelData *voxelData_device;
#endif
  MultiIndexData indexData_host;  // 记录多个子图的基础信息。比如全局id、位姿等
  MultiVoxelData voxelData_host;  // 记录多个子图的voxel array指针

  ITMSceneParams sceneParams;     // 三维场景的参数。所有子图都一样
  /** 构造函数 */
  ITMRenderStateMultiScene(const Vector2i &imgSize, float vf_min, float vf_max, MemoryDeviceType _memoryType)
      : ITMRenderState(imgSize, vf_min, vf_max, _memoryType) {
    memoryType = _memoryType;

#ifndef COMPILE_WITHOUT_CUDA
    if (memoryType == MEMORYDEVICE_CUDA) {
      ORcudaSafeCall(cudaMalloc((void **) &indexData_device, sizeof(MultiIndexData)));
      ORcudaSafeCall(cudaMalloc((void **) &voxelData_device, sizeof(MultiVoxelData)));
    }
#endif
  }
  /** 析构函数 */
  ~ITMRenderStateMultiScene(void) {
#ifndef COMPILE_WITHOUT_CUDA
    if (memoryType == MEMORYDEVICE_CUDA) {
      ORcudaSafeCall(cudaFree(indexData_device));
      ORcudaSafeCall(cudaFree(voxelData_device));
    }
#endif
  }
  /**
   * @brief 获取所有子图的相关信息，比如三维场景、位姿等
   * @param[in] sceneManager 管理左右子图
   */
  void PrepareLocalMaps(const MultiSceneManager &sceneManager) {
    sceneParams = *(sceneManager.getLocalMap(0)->scene->sceneParams); // 三维场景的参数，所有子图都一样

    int num = (int) sceneManager.numLocalMaps();          // 子图的总数
    if (num > MAX_NUM_LOCALMAPS) num = MAX_NUM_LOCALMAPS; // 子图的总数不能超过32
    indexData_host.numLocalMaps = num;
    //! 遍历每个子图
    for (int localMapId = 0; localMapId < num; ++localMapId) {
      // 获取每个子图在voxel坐标系下的世界到子图的位姿
      indexData_host.poses_vs[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetM();// 世界到子图位姿，T_sw
      indexData_host.poses_vs[localMapId].m30 /= sceneParams.voxelSize;
      indexData_host.poses_vs[localMapId].m31 /= sceneParams.voxelSize;
      indexData_host.poses_vs[localMapId].m32 /= sceneParams.voxelSize;
      // 获取每个子图的子图到世界的位姿
      indexData_host.posesInv[localMapId] = sceneManager.getEstimatedGlobalPose(localMapId).GetInvM();
      // 获取每个子图的hash table的指针
      indexData_host.index[localMapId] = sceneManager.getLocalMap(localMapId)->scene->index.getIndexData();
      // 获取每个子图的voxel block array的指针
      voxelData_host.voxels[localMapId] = sceneManager.getLocalMap(localMapId)->scene->localVBA.GetVoxelBlocks();
    }
    //! 需要的话，拷贝一份到GPU上
#ifndef COMPILE_WITHOUT_CUDA
    if (memoryType == MEMORYDEVICE_CUDA) {
      ORcudaSafeCall(cudaMemcpy(indexData_device, &(indexData_host), sizeof(MultiIndexData), cudaMemcpyHostToDevice));
      ORcudaSafeCall(cudaMemcpy(voxelData_device, &(voxelData_host), sizeof(MultiVoxelData), cudaMemcpyHostToDevice));
    }
#endif
  }
};

}

