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

template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::ResetScene(
    ITMScene<TVoxel, ITMVoxelBlockHash> *scene) {
  int numBlocks = scene->index.getNumAllocatedVoxelBlocks();    // block数量，=SDF_LOCAL_BLOCK_NUM
  int blockSize = scene->index.getVoxelBlockSize();             // block中voxel的数量，=SDF_BLOCK_SIZE3
  //! 清空每个voxel信息
  TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
  for (int i = 0; i < numBlocks * blockSize; ++i) // numBlocks * blockSize = voxel总数
    voxelBlocks_ptr[i] = TVoxel();
  //! 重置voxel block array中的空闲信息
  int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
  for (int i = 0; i < numBlocks; ++i)
    vbaAllocationList_ptr[i] = i;
  //! 重置localVBA中可用空间的数量
  scene->localVBA.lastFreeBlockId = numBlocks - 1;  // -1是因为lastFreeBlockId本质上是localVBA中的空余下标
  //! 重置hash table
  ITMHashEntry tmpEntry;
  memset(&tmpEntry, 0, sizeof(ITMHashEntry));
  tmpEntry.ptr = -2;
  ITMHashEntry *hashEntry_ptr = scene->index.GetEntries();
  for (int i = 0; i < scene->index.noTotalEntries; ++i)
    hashEntry_ptr[i] = tmpEntry;
  //! 重置unordered list的空闲信息   
  int *excessList_ptr = scene->index.GetExcessAllocationList();
  for (int i = 0; i < SDF_EXCESS_LIST_SIZE; ++i)
    excessList_ptr[i] = i;

  scene->index.SetLastFreeExcessListId(SDF_EXCESS_LIST_SIZE - 1);
}

template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(
    ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
    const ITMRenderState *renderState) {
  //! 准备
  Vector2i rgbImgSize = view->rgb->noDims;          // 当前帧RGB图的分辨率
  Vector2i depthImgSize = view->depth->noDims;      // 当前帧深度图的分辨率
  float voxelSize = scene->sceneParams->voxelSize;

  Matrix4f M_d = trackingState->pose_d->GetM();             // 当前帧中深度图的位姿（word to local）
  Matrix4f M_rgb;
  if (TVoxel::hasColorInformation)
    M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d; // 当前帧中RGB图的位姿（word to local）

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *) renderState;  // TODO: 父类转子类指针，最好用dynamic_cast

  Vector4f projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;      // 深度图相机内参
  Vector4f projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;  // RGB图相机内参

  float mu = scene->sceneParams->mu;    // TSDF的截断值对应的距离，单位米
  int maxW = scene->sceneParams->maxW;  // voxel的最大观测次数，用于融合

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);                // 深度图
  float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU); // 深度图的置信度（根据距离计算）
  Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);                 // 彩色图
  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();                  // device上的voxel block array
  ITMHashEntry *hashTable = scene->index.GetEntries();                  // hash table

  int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();          // 可见entry id列表
  int noVisibleEntries = renderState_vh->noVisibleEntries;              // 可见entry数量

  bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;  // 到达最大观测次数后是否继续融合
  //bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
#pragma omp parallel for // 告诉openmp中展开循环
#endif
  //! 遍历可见entry列表
  for (int entryId = 0; entryId < noVisibleEntries; entryId++) {
    // 获取entry
    const ITMHashEntry &currentHashEntry = hashTable[visibleEntryIds[entryId]];
    if (currentHashEntry.ptr < 0) continue;       // ptr为-，说明没有分配block

    // 计算entry对应的block靠近原点的顶点的voxel坐标   // TODO: 有bug吧？如果存在hash冲突，不就不能找到真正的block了吗？？？
    Vector3i globalPos;
    globalPos.x = currentHashEntry.pos.x;   // block坐标
    globalPos.y = currentHashEntry.pos.y;
    globalPos.z = currentHashEntry.pos.z;
    globalPos *= SDF_BLOCK_SIZE;            // voxel坐标

    // 遍历当前block中的每一个voxel，更新（融合）数据
    TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);  // block中第1个voxel的指针
    for (int z = 0; z < SDF_BLOCK_SIZE; z++)
      for (int y = 0; y < SDF_BLOCK_SIZE; y++)
        for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
          // 当前voxel在当前block中的一维坐标
          int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE; 
          // 到达最大观测次数后就不融合了
          if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;
          //if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) continue;  // TODO:???

          // 当前voxel在全局下的真实坐标（单位米）
          Vector4f pt_model;
          pt_model.x = (float) (globalPos.x + x) * voxelSize;
          pt_model.y = (float) (globalPos.y + y) * voxelSize;
          pt_model.z = (float) (globalPos.z + z) * voxelSize;
          pt_model.w = 1.0f;        // 为了后续使用齐次坐标???

          // 更新当前voxel
          ComputeUpdatedVoxelInfo<TVoxel::hasColorInformation, TVoxel::hasConfidenceInformation, TVoxel>::compute(
              localVoxelBlock[locId], pt_model, M_d, projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence,
              depthImgSize, rgb, rgbImgSize); 
        }
  }
}

template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::AllocateSceneFromDepth(
    ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
    const ITMRenderState *renderState, bool onlyUpdateVisibleList, bool resetVisibleList) {
  //! 准备
  Vector2i depthImgSize = view->depth->noDims;   // 深度图分辨率
  float voxelSize = scene->sceneParams->voxelSize; 

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *) renderState;  // TODO: 父类转子类指针，最好用dynamic_cast
  if (resetVisibleList) renderState_vh->noVisibleEntries = 0;             // 需要的话，visitle list置为零
   
  Matrix4f M_d = trackingState->pose_d->GetM();  // 深度图的位姿 和 它的逆
  Matrix4f invM_d;
  M_d.inv(invM_d); 

  Vector4f projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;  // 深度图的相机内参
  Vector4f invProjParams_d = projParams_d;                                      // 相机内参的反投影，方便后面计算
  invProjParams_d.x = 1.0f / invProjParams_d.x;                                 // = 1/fx
  invProjParams_d.y = 1.0f / invProjParams_d.y;                                 // = 1/fy

  float mu = scene->sceneParams->mu;                                  // TSDF的截断值对应的距离

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);              // 深度图
  int *voxelAllocationList = scene->localVBA.GetAllocationList();     // VBA的空闲信息
  int *excessAllocationList = scene->index.GetExcessAllocationList(); // excess list的空闲信息
  ITMHashEntry *hashTable = scene->index.GetEntries();                // hash table
  ITMHashSwapState *swapStates =                                      // 数据传输状态（host和device之间）
      scene->globalCache != NULL ? scene->globalCache->GetSwapStates(false) : 0;
  int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();         // 当前视角下可见entrys的ID(数组)
  uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType(); // 可见entry的类型
  int noTotalEntries = scene->index.noTotalEntries;                     // entry总数。包含ordered + unordered
  uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);  // 数据存放位置。order entry或excess list
  memset(entriesAllocType, 0, noTotalEntries);  
  Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU); // 每个entry对应的block坐标

  bool useSwapping = scene->globalCache != NULL;                        // 是否支持host-device之间的数据交换

  float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE); // block实际边长的倒数，方便后面计算。应该叫BlockSize更合适
  int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;           // VBA中剩余空位数
  int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();    // excess list中剩余空位数
  //! 将上一帧可见但还没拷贝出去的entry的可见类型都值为3 ???
  for (int i = 0; i < renderState_vh->noVisibleEntries; i++)
    entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed
  //! 查看每个像素对应三维点附近所有block的alloction情况和可见类型。build hashVisibility
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (int locId = 0; locId < depthImgSize.x * depthImgSize.y; locId++) {
    int y = locId / depthImgSize.x;
    int x = locId - y * depthImgSize.x;
    buildHashAllocAndVisibleTypePP(entriesAllocType, entriesVisibleType, x, y, blockCoords, depth, invM_d,
                                   invProjParams_d, mu, depthImgSize, oneOverVoxelSize, hashTable,
                                   scene->sceneParams->viewFrustum_min, scene->sceneParams->viewFrustum_max);
  }
  if (onlyUpdateVisibleList)  // ???为啥只更新可见列表的话，关闭swap？？？
    useSwapping = false;
  //! 对于所有的entry，对于上面找到的需要分配的block进行分配。推荐参考文件“说明材料/localVBA.pptx”
  if (!onlyUpdateVisibleList) {  
    // allocate 分配
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {  // 遍历每一个entry
      int vbaIdx, exlIdx;
      unsigned char hashChangeType = entriesAllocType[targetIdx];

      switch (hashChangeType) {
      case 1:         //! 需要分配到ordered list。needs allocation, fits in the ordered list
        vbaIdx = lastFreeVoxelBlockId;
        lastFreeVoxelBlockId--;           // --是因为VBA是从后往前存的

        if (vbaIdx >= 0) {  // 剩余空间充足。there is room in the voxel block array
          Vector4s pt_block_all = blockCoords[targetIdx]; // 取出buildHashAllocAndVisibleTypePP中记录的block坐标

          ITMHashEntry hashEntry;
          hashEntry.pos.x = pt_block_all.x;
          hashEntry.pos.y = pt_block_all.y;
          hashEntry.pos.z = pt_block_all.z;
          hashEntry.ptr = voxelAllocationList[vbaIdx];  // NOTE：voxelAllocationList[vbaIdx]就是VBA中的空闲位置
          hashEntry.offset = 0;   // TODO: 设置成-1更合理。这样之后遇到冲突了，offset就不用+1或者-1了

          hashTable[targetIdx] = hashEntry;             // hash table中记录entry信息
        } else {            // 剩余空间不足
          // Mark entry as not visible since we couldn't allocate it but buildHashAllocAndVisibleTypePP changed its
          // state.
          entriesVisibleType[targetIdx] = 0;  // 无法allocate，需要设置visible=false

          // 剩余空间不足了，要把刚刚分配的再还回去。Restore previous value to avoid leaks.
          lastFreeVoxelBlockId++;
        }
        break;
      case 2:         //! 需要分配到excess list（即unordered entry）。needs allocation in the excess list
        vbaIdx = lastFreeVoxelBlockId;
        lastFreeVoxelBlockId--;
        exlIdx = lastFreeExcessListId;
        lastFreeExcessListId--;

        if (vbaIdx >= 0 && exlIdx >= 0) { // VBA和excess list中都有空位。
                                          // there is room in the voxel block array and excess list
          Vector4s pt_block_all = blockCoords[targetIdx]; // 取出block坐标

          ITMHashEntry hashEntry;
          hashEntry.pos.x = pt_block_all.x;
          hashEntry.pos.y = pt_block_all.y;
          hashEntry.pos.z = pt_block_all.z;
          hashEntry.ptr = voxelAllocationList[vbaIdx];  // NOTE：所谓分配内存，就是将entry的ptr取值为VBA的空闲位置的下标
                                                        // NOTE: voxelAllocationList[vbaIdx]就是VBA中的空闲位置
          hashEntry.offset = 0; // TODO: 设置成-1更合理。这样之后遇到冲突了，offset就不用+1或者-1了
          int exlOffset = excessAllocationList[exlIdx];   // 跟voxelAllocationList同理，找到excess list的空闲位置

          hashTable[targetIdx].offset = exlOffset + 1;  // +1是因为上面offset = 0，后面使用的时候会-1，判断的时候会跟1比较
                                                        // connect to child
          hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry;  // SDF_BUCKET_NUM之后的是excess list
                                                              // add child to the excess list
          entriesVisibleType[SDF_BUCKET_NUM + exlOffset] = 1; // ∵buildHashAllocAndVisibleTypePP没对unorder entry标记可见
          //TODO：这里是不是有bug？lastFreeExcessListId应该指向一个空的位置，但是exlOffset + 1，导致每次添加会有一个空位？？？
        } else {                          // VBA和excess list有一个没空位了
          // No need to mark the entry as not visible since buildHashAllocAndVisibleTypePP did not mark it.
          // Restore previous value to avoid leaks.
          lastFreeVoxelBlockId++;
          lastFreeExcessListId++;
        }
        break;
      }
    }
  }

  //! 找到所有entry中可见的 build visible list
  int noVisibleEntries = 0;
  for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
    unsigned char hashVisibleType = entriesVisibleType[targetIdx];
    const ITMHashEntry &hashEntry = hashTable[targetIdx];
    // 之前可见 但 现在不可见的再检查一下可见性
    if (hashVisibleType == 3) {       // =3说明之前可见，但是现在不可见
      bool isVisibleEnlarged, isVisible;
      if (useSwapping) {  // 有swap的会用更大（扩大1/8）的image size查看可见性。 ∵要保证device和host的数据无缝融合？？？
        checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
                                   depthImgSize);
        if (!isVisibleEnlarged) hashVisibleType = 0;  // 搜索后确定不可见
      } else {            // 没有swap用正常的image size查看可见性
        checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
                                    depthImgSize);
        if (!isVisible) hashVisibleType = 0;          // 搜索后确定不可见
      }
      entriesVisibleType[targetIdx] = hashVisibleType;
    }
    // 开启swap，可见 但 不只在显存上，标记一下需要合并
    if (useSwapping) {
      if (hashVisibleType > 0 && swapStates[targetIdx].state != 2)
        swapStates[targetIdx].state = 1;  // =1说明数据同时在内存和显存上，尚未合并
    }
    // 最终记录可见的
    if (hashVisibleType > 0) {
      visibleEntryIDs[noVisibleEntries] = targetIdx; 
      noVisibleEntries++;
    }

#if 0   // TODO: active list是啥？？？论文中没有
    // "active list", currently disabled
    if (hashVisibleType == 1) {
        activeEntryIDs[noActiveEntries] = targetIdx;
        noActiveEntries++;
    }
#endif
  }

  //! 开启swap，将所有entry中可见但是没有分配空间的分配一下 
  // TODO：之前buildHashAllocAndVisibleTypePP不是都分配过了吗？？？还有什么情况会没有分配过？？？
  // reallocate deleted ones from previous swap operation
  if (useSwapping) {
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {  // TODO: 遍历visibleEntryIDs更好吧？？？
      int vbaIdx;
      ITMHashEntry hashEntry = hashTable[targetIdx];

      if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) { // 可见但是没有分配空间
        vbaIdx = lastFreeVoxelBlockId;
        lastFreeVoxelBlockId--;
        if (vbaIdx >= 0)    // 还有剩余空间
          hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx]; // NOTE：分配内存，就是将entry的ptr取值为VBA的空闲位置的下标
        else                // 没有剩余空间了，还回去
          lastFreeVoxelBlockId++; // Avoid leaks
      }
    }
  }

  renderState_vh->noVisibleEntries = noVisibleEntries;

  scene->localVBA.lastFreeBlockId = lastFreeVoxelBlockId;
  scene->index.SetLastFreeExcessListId(lastFreeExcessListId); // TODO：为啥不把lastFreeExcessListId弄成public
}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::ITMSceneReconstructionEngine_CPU(void) {}

template<class TVoxel>
ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::~ITMSceneReconstructionEngine_CPU(void) {}

template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::ResetScene(
    ITMScene<TVoxel, ITMPlainVoxelArray> *scene) {
  int numBlocks = scene->index.getNumAllocatedVoxelBlocks();
  int blockSize = scene->index.getVoxelBlockSize();

  TVoxel *voxelBlocks_ptr = scene->localVBA.GetVoxelBlocks();
  for (int i = 0; i < numBlocks * blockSize; ++i) voxelBlocks_ptr[i] = TVoxel();
  int *vbaAllocationList_ptr = scene->localVBA.GetAllocationList();
  for (int i = 0; i < numBlocks; ++i) vbaAllocationList_ptr[i] = i;
  scene->localVBA.lastFreeBlockId = numBlocks - 1;
}

template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::AllocateSceneFromDepth(
    ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
    const ITMRenderState *renderState, bool onlyUpdateVisibleList, bool resetVisibleList) {}

template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMPlainVoxelArray>::IntegrateIntoScene(
    ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
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
