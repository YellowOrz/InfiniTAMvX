// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMSceneReconstructionEngine_CPU.h"

#include "../Shared/ITMSceneReconstructionEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
using namespace ITMLib;

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::ITMSceneReconstructionEngine_CPU(void) {
  int noTotalEntries = ITMVoxelBlockHash::noTotalEntries;
  entriesAllocType = new ORUtils::MemoryBlock<unsigned char>(noTotalEntries, MEMORYDEVICE_CPU);
  blockCoords = new ORUtils::MemoryBlock<Vector4s>(noTotalEntries, MEMORYDEVICE_CPU);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::~ITMSceneReconstructionEngine_CPU(void) {
  delete entriesAllocType;
  delete blockCoords;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::ResetScene(ITMScene<TVoxel,
                                                                                      ITMVoxelBlockHash> *scene) {
  int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
  int blockSize = scene->index.getVoxelBlockSize();

  TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
  for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
  int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
  for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
  scene->localVBA.lastFreeBlockId = numBlocks - 1;

  ITMHashEntry tmpEntry;
  memset(&tmpEntry, 0, sizeof(ITMHashEntry));
  tmpEntry.ptr = -2;
  ITMHashEntry *hashEntry_ptr = scene->index.GetEntries();
  for (int i = 0; i < scene->index.noTotalEntries; ++i) hashEntry_ptr[i] = tmpEntry;
  int *excessList_ptr = scene->index.GetExcessAllocationList();
  for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i) excessList_ptr[i] = i;

  scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}

template<class TVoxel>
/**
 *
 * @tparam [in]TVoxel
 * @param [in]scene
 * @param [in]view
 * @param [in]trackingState
 * @param [in]renderState 存储场景重建和可视化引擎使用的渲染状态。
 */
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(ITMScene<TVoxel,
                                                                                              ITMVoxelBlockHash> *scene,
                                                                                     const ITMView *view,
                                                                                     const ITMTrackingState *trackingState,
                                                                                     const ITMRenderState *renderState) {
  //TODO（h）掌握TSDF voxel hasing
  //！ 变量初始化
  Vector2i rgbImgSize = view->rgb->noDims;  //获得当前帧的彩色图像 大小 以像素为单位
  Vector2i depthImgSize = view->depth->noDims;  //获得当前帧的深度图像 大小 以像素为单位
  float voxelSize = scene->sceneParams->voxelSize;

  Matrix4f M_d, M_rgb;
  Vector4f projParams_d, projParams_rgb;

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *) renderState;  //存储场景重建和可视化引擎使用的渲染状态。

  M_d = trackingState->pose_d->GetM();  //当前深度图像位姿  4*4
  if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;
  // trafo_rgb_to_depth  此转换将点从 RGB 相机坐标系转移到深度相机坐标系。  calib_inv 变换矩阵的逆矩阵 用于配准

  projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;  //深度相机的内在参数中的  校准矩阵 4*1
  projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;  //彩色相机的内在参数中的 校准矩阵 4*1

  float mu = scene->sceneParams->mu;  //场景参数中的一个数值
  int maxW = scene->sceneParams->maxW; //voxel的最大观测次数，用来融合；超过后若还要融合，采用滑窗方式

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);  //获取深度图像的指针
  float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);  //获取深度图像置信度的指针
  Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);  //获取彩色图的指针
  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();  //获得体素块
  ITMHashEntry *hashTable = scene->index.GetEntries();  //获得哈希表指针

  int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();  //可见voxel blocks 的ID 的指针
  int noVisibleEntries = renderState_vh->noVisibleEntries;  //实时的条目数

  bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;  //到达最大观测次数后是否继续融合
  //bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
#pragma omp parallel for
//pragma omp parallel for是OpenMP中的一个指令，表示接下来的for循环将被多线程执行，另外每次循环之间不能有关系
#endif
   //！开始处理
  for (int entryId = 0; entryId < noVisibleEntries; entryId++) {    // noVisibleEntries   实时列表中的条目数。
    Vector3i globalPos;
    const ITMHashEntry &currentHashEntry = hashTable[visibleEntryIds[entryId]];  //哈希表结构

    if (currentHashEntry.ptr < 0) continue;  //ptr 为指向体素数组 小于0为未标识的体素块

    globalPos.x = currentHashEntry.pos.x;   //.pos   8x8x8 卷角的位置，用于标识entry。
    globalPos.y = currentHashEntry.pos.y;
    globalPos.z = currentHashEntry.pos.z;
    globalPos *= SDF_BLOCK_SIZE;    //SDF_BLOCK_SIZE 8
    //放入8*8*8的体素块中

    TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);   //SDF_BLOCK_SIZE3 512
    //构建8*8*8模型
    for (int z = 0; z < SDF_BLOCK_SIZE; z++)
      for (int y = 0; y < SDF_BLOCK_SIZE; y++)
        for (int x = 0; x < SDF_BLOCK_SIZE; x++) {//循环整个block 的每一个小块
          Vector4f pt_model;
          int locId;

          locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;  //给每一个小块一个ID

          if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;
          // stopIntegratingAtMaxW  到达最大观测次数后是否继续融合  maxW  voxel的最大观测次数，用来融合

          //if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) continue;
          //TODO（h）：与投影成图像 TSDF 有关 复习kinectfusion
          pt_model.x = (float) (globalPos.x + x) * voxelSize;
          pt_model.y = (float) (globalPos.y + y) * voxelSize;
          pt_model.z = (float) (globalPos.z + z) * voxelSize;   //globalPos 为卷角坐标*8  voxelSize 单位米
          pt_model.w = 1.0f;  // 为了后续使用齐次坐标？

          ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation, TVoxel::hasConfidenceInformation, TVoxel>::compute(
              localVoxelBlock[locId],
              pt_model,
              M_d,
              projParams_d,
              M_rgb,
              projParams_rgb,
              mu,
              maxW,
              depth,
              confidence,
              depthImgSize,
              rgb,
              rgbImgSize);  //更新体素保存的一些信息    根据hasColorInformation  hasConfidenceInformation 的bool值来选择函数
        }
  }
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(ITMScene<TVoxel,
                                                                                                  ITMVoxelBlockHash> *scene,
                                                                                         const ITMView *view,
                                                                                         const ITMTrackingState *trackingState,
                                                                                         const ITMRenderState *renderState,
                                                                                         bool onlyUpdateVisibleList,
                                                                                         bool resetVisibleList) {
  Vector2i depthImgSize = view->depth->noDims;   //图像的大小 像素为单位
  float voxelSize = scene->sceneParams->voxelSize;  //单位米

  Matrix4f M_d, invM_d;
  Vector4f projParams_d, invProjParams_d;

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *) renderState;  //存储场景重建和可视化引擎使用的渲染状态。
  if (resetVisibleList) renderState_vh->noVisibleEntries = 0;  //实时列表的条目数

  M_d = trackingState->pose_d->GetM();  //  深度相机的当前位姿 用4*4的矩阵
  M_d.inv(invM_d);  //逆矩阵 invM_d 为M_d的逆矩阵

  projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all; //深度相机的内在参数   校准参数的4*1矩阵
  invProjParams_d = projParams_d;
  invProjParams_d.x = 1.0f / invProjParams_d.x;  //x y z w
  invProjParams_d.y = 1.0f / invProjParams_d.y;

  float mu = scene->sceneParams->mu;
    //获取一系列的指针
  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);  //获得浮点值深度图像的指针
  int *voxelAllocationList = scene->localVBA.GetAllocationList();
  int *excessAllocationList = scene->index.GetExcessAllocationList();  //获得溢出的列表的ID？
  ITMHashEntry *hashTable = scene->index.GetEntries(); //获得哈希表里的数据的指针
  ITMHashSwapState *swapStates = scene->globalCache != NULL ? scene->globalCache->GetSwapStates(false) : 0;
  int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();  //获得可见entrys的ID
  uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType();  //
  uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
  Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU);  //获得块坐标？
  int noTotalEntries = scene->index.noTotalEntries;

  bool useSwapping = scene->globalCache != NULL;

  float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);

  int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;
  int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

  int noVisibleEntries = 0;

  memset(entriesAllocType, 0, noTotalEntries);
  //将 entriesAllocType 中 长度为noTotalEntries 内存块填充数字0 内存清零

  for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
    entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed 在前一帧可见且未流式传输


  //build hashVisibility 创建
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
    int y = locId / depthImgSize.x;
    int x = locId - y * depthImgSize.x;
    buildHashAllocAndVisibleTypePP(entriesAllocType,
                                   entriesVisibleType,
                                   x,
                                   y,
                                   blockCoords,
                                   depth,
                                   invM_d,
                                   invProjParams_d,
                                   mu,
                                   depthImgSize,
                                   oneOverVoxelSize,
                                   hashTable,
                                   scene->sceneParams->viewFrustum_min,
                                   scene->sceneParams->viewFrustum_max);
  }
   //TODO 构建哈希分配以及可见类型pp？
  if (onlyUpdateVisibleList) useSwapping = false;
  if (!onlyUpdateVisibleList) {
    //allocate 分配
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
      int vbaIdx, exlIdx;
      unsigned char hashChangeType = entriesAllocType[targetIdx];

      switch (hashChangeType) {
        case 1: //needs allocation, fits in the ordered list 需要分配，适合有序列表
          vbaIdx = lastFreeVoxelBlockId;
          lastFreeVoxelBlockId--;

          if (vbaIdx >= 0) //there is room in the voxel block array 体素块阵列中有空间
          {
            Vector4s pt_block_all = blockCoords[targetIdx];

            ITMHashEntry hashEntry;
            hashEntry.pos.x = pt_block_all.x;
            hashEntry.pos.y = pt_block_all.y;
            hashEntry.pos.z = pt_block_all.z;
            hashEntry.ptr = voxelAllocationList[vbaIdx];
            hashEntry.offset = 0;

            hashTable[targetIdx] = hashEntry;
          } else {
            // Mark entry as not visible since we couldn't allocate it but buildHashAllocAndVisibleTypePP changed its state.
            entriesVisibleType[targetIdx] = 0;

            // Restore previous value to avoid leaks.
            lastFreeVoxelBlockId++;
          }

          break;
        case 2: //needs allocation in the excess list
          vbaIdx = lastFreeVoxelBlockId;
          lastFreeVoxelBlockId--;
          exlIdx = lastFreeExcessListId;
          lastFreeExcessListId--;

          if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
          {
            Vector4s pt_block_all = blockCoords[targetIdx];

            ITMHashEntry hashEntry;
            hashEntry.pos.x = pt_block_all.x;
            hashEntry.pos.y = pt_block_all.y;
            hashEntry.pos.z = pt_block_all.z;
            hashEntry.ptr = voxelAllocationList[vbaIdx];
            hashEntry.offset = 0;

            int exlOffset = excessAllocationList[exlIdx];

            hashTable[targetIdx].offset = exlOffset + 1; //connect to child

            hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

            entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; //make child visible and in memory
          } else {
            // No need to mark the entry as not visible since buildHashAllocAndVisibleTypePP did not mark it.
            // Restore previous value to avoid leaks.
            lastFreeVoxelBlockId++;
            lastFreeExcessListId++;
          }

          break;
      }
    }
  }

  //build visible list
  for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
    unsigned char hashVisibleType = entriesVisibleType[targetIdx];
    const ITMHashEntry &hashEntry = hashTable[targetIdx];

    if (hashVisibleType == 3) {
      bool isVisibleEnlarged, isVisible;

      if (useSwapping) {
        checkBlockVisibility<true>(isVisible,
                                   isVisibleEnlarged,
                                   hashEntry.pos,
                                   M_d,
                                   projParams_d,
                                   voxelSize,
                                   depthImgSize);
        if (!isVisibleEnlarged) hashVisibleType = 0;
      } else {
        checkBlockVisibility<false>(isVisible,
                                    isVisibleEnlarged,
                                    hashEntry.pos,
                                    M_d,
                                    projParams_d,
                                    voxelSize,
                                    depthImgSize);
        if (!isVisible) { hashVisibleType = 0; }
      }
      entriesVisibleType[targetIdx] = hashVisibleType;
    }

    if (useSwapping) {
      if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
    }

    if (hashVisibleType > 0) {
      visibleEntryIDs[noVisibleEntries] = targetIdx;
      noVisibleEntries++;
    }

#if 0
    // "active list", currently disabled
    if (hashVisibleType == 1)
    {
        activeEntryIDs[noActiveEntries] = targetIdx;
        noActiveEntries++;
    }
#endif
  }

  //reallocate deleted ones from previous swap operation
  if (useSwapping) {
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
      int vbaIdx;
      ITMHashEntry hashEntry = hashTable[targetIdx];

      if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) {
        vbaIdx = lastFreeVoxelBlockId;
        lastFreeVoxelBlockId--;
        if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
        else lastFreeVoxelBlockId++; // Avoid leaks
      }
    }
  }

  renderState_vh->noVisibleEntries = noVisibleEntries;

  scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
  scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CPU(void) {}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CPU(void) {}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::ResetScene(ITMScene<TVoxel,
                                                                                       ITMPlainVoxelArray> *scene) {
  int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
  int blockSize = scene->index.getVoxelBlockSize();

  TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
  for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
  int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
  for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
  scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(ITMScene<TVoxel,
                                                                                                   ITMPlainVoxelArray> *scene,
                                                                                          const ITMView *view,
                                                                                          const ITMTrackingState *trackingState,
                                                                                          const ITMRenderState *renderState,
                                                                                          bool onlyUpdateVisibleList,
                                                                                          bool resetVisibleList) {}

template<class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(ITMScene<TVoxel,
                                                                                               ITMPlainVoxelArray> *scene,
                                                                                      const ITMView *view,
                                                                                      const ITMTrackingState *trackingState,
                                                                                      const ITMRenderState *renderState) {
  Vector2i rgbImgSize = view->rgb->noDims;
  Vector2i depthImgSize = view->depth->noDims;
  float voxelSize = scene->sceneParams->voxelSize;

  Matrix4f M_d, M_rgb;
  Vector4f projParams_d, projParams_rgb;

  M_d = trackingState->pose_d->GetM();
  if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

  projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
  projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

  float mu = scene->sceneParams->mu;
  int maxW = scene->sceneParams->maxW;

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
  Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
  TVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();

  const ITMPlainVoxelArray::IndexData *arrayInfo = scene->index.getIndexData();

  bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;
  //bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int locId = 0;
       locId < scene->index.getVolumeSize().x * scene->index.getVolumeSize().y * scene->index.getVolumeSize().z;
       ++locId) {
    int z = locId / (scene->index.getVolumeSize().x * scene->index.getVolumeSize().y);
    int tmp = locId - z * scene->index.getVolumeSize().x * scene->index.getVolumeSize().y;
    int y = tmp / scene->index.getVolumeSize().x;
    int x = tmp - y * scene->index.getVolumeSize().x;
    Vector4f pt_model;

    if (stopIntegratingAtMaxW) if (voxelArray[locId].w_depth == maxW) continue;
    //if (approximateIntegration) if (voxelArray[locId].w_depth != 0) continue;

    pt_model.x = (float) (x + arrayInfo->offset.x) * voxelSize;
    pt_model.y = (float) (y + arrayInfo->offset.y) * voxelSize;
    pt_model.z = (float) (z + arrayInfo->offset.z) * voxelSize;
    pt_model.w = 1.0f;

    ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation,
                            TVoxel::hasConfidenceInformation,
                            TVoxel>::compute(voxelArray[locId],
                                             pt_model,
                                             M_d,
                                             projParams_d,
                                             M_rgb,
                                             projParams_rgb,
                                             mu,
                                             maxW,
                                             depth,
                                             depthImgSize,
                                             rgb,
                                             rgbImgSize);
  }
}
