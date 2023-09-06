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

/**
 * 根据可见列表，将当前输入的单帧融入场景中
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in,out] scene 三维场景
 * @param[in] view 当前输入图像
 * @param[in] trackingState 存储一些关于当前跟踪状态的内部变量，最重要的是相机姿势
 * @param[in] renderState 渲染相关数据。主要用到其中的可见entry列表
 */
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
  int maxW = scene->sceneParams->maxW;  // voxel的最大观测次数，用来融合；超过后若还要融合，采用滑窗方式

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);                // 深度图
  float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU); // 深度图的置信度（根据距离计算）
  Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);                 // 彩色图
  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();                  // voxel block array
  ITMHashEntry *hashTable = scene->index.GetEntries();                  // hash table

  int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();          // 可见entry列表
  int noVisibleEntries = renderState_vh->noVisibleEntries;              // 可见entry数量

  bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;  // 到达最大观测次数后是否继续融合
  //bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
#pragma omp parallel for // 告诉openmp中展开循环
#endif
   //！开始处理

/*
   1 所有的block存储在连续的内存之中 即称VBA
   2 voxel block 由8*8*8个voxel组成。voxel 存储 sdf，color 和 weight 信息
   3 hash table 连续的数组  其记录的映射关系（对应）关系   其中ptr储存该block在array 中的位置
   4 哈希函数 输入一个block的ID坐标（pos） 返回一个独一无二的 index（即ptr GTR(）

    哈希冲突即为 不同的pos计算出相同的ptr
    解决方法为将映射相同ptr的block的全部存储下来，offset初始化为-1 如有冲突offset为两个相同ptr的entry的距离 查找时，遍历直至offset<=-1
*/
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
              depthImgSize, rgb,
              rgbImgSize); // 更新体素保存的一些信息    根据hasColorInformation  hasConfidenceInformation
                           // 的bool值来选择函数
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

  Vector4f projParams_d, invProjParams_d;

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *) renderState;  // TODO: 父类转子类指针，最好用dynamic_cast
  if (resetVisibleList) renderState_vh->noVisibleEntries = 0;             // 需要的话，visitle list置为零

  Matrix4f M_d, invM_d;   // 深度图的位姿 和 它的逆
  M_d = trackingState->pose_d->GetM();  
  M_d.inv(invM_d); 

  projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all; // 深度图的相机内参
  invProjParams_d = projParams_d;                                     // 相机内参的反投影，方便后面计算
  invProjParams_d.x = 1.0f / invProjParams_d.x;                       // =1/fx
  invProjParams_d.y = 1.0f / invProjParams_d.y;                       // =1/fy

  float mu = scene->sceneParams->mu;                                  // TSDF的截断值对应的距离

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);              // 深度图
  int *voxelAllocationList = scene->localVBA.GetAllocationList();     // voxel block array
  int *excessAllocationList = scene->index.GetExcessAllocationList(); // ???获得溢出的列表的ID？
  ITMHashEntry *hashTable = scene->index.GetEntries();                // hash table
  ITMHashSwapState *swapStates =                                      // 数据传输状态（内存和显存之间）
      scene->globalCache != NULL ? scene->globalCache->GetSwapStates(false) : 0;
  int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();         // 当前视角下可见entrys的ID数组
  uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType(); // 可见entrys的类型？？？
  for (int i = 0; i < renderState_vh->noVisibleEntries; i++)  // 上一帧可见的？？？
    entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed
  uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);  // 数据存放位置。order entry或excess list
  int noTotalEntries = scene->index.noTotalEntries;
  memset(entriesAllocType, 0, noTotalEntries);
  Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU); // 每个entry对应的block坐标

  bool useSwapping = scene->globalCache != NULL;                        // 是否支持内存-显存之间的数据交换

  float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);         // block实际边长的倒数，方便后面计算 
                                                                        //TODO：不应该叫voxelsize

  int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;           // VBA中最前面一个空位的id（∵从后往前存）
  int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();    // excess list中最前面一个空位的id（∵从后往前存）

  //! 查看每个像素对应三维点附近所有block的alloction和可见情况。build hashVisibility
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
  //! 对于所有的entry，对于上面找到的需要分配的block进行分配
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
          Vector4s pt_block_all = blockCoords[targetIdx]; // 取出block坐标

          ITMHashEntry hashEntry;
          hashEntry.pos.x = pt_block_all.x;
          hashEntry.pos.y = pt_block_all.y;
          hashEntry.pos.z = pt_block_all.z;
          hashEntry.ptr = voxelAllocationList[vbaIdx];  //! 分配的核心：将entry的ptr指向VBA中可用的空余空间的ID
          hashEntry.offset = 0;

          hashTable[targetIdx] = hashEntry;             // hash table中记录entry信息
        } else {            // 剩余空间不足，无法allocate，需要设置visible=false
          // Mark entry as not visible since we couldn't allocate it but buildHashAllocAndVisibleTypePP changed its
          // state.
          entriesVisibleType[targetIdx] = 0;

          // Restore previous value to avoid leaks.
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
          hashEntry.ptr = voxelAllocationList[vbaIdx]; //! 分配的核心：将entry的ptr指向VBA中可用的空余空间的ID
          hashEntry.offset = 0;                        
          // excess list中首先找到可以用的entry的id
          int exlOffset = excessAllocationList[exlIdx];   //???

          hashTable[targetIdx].offset = exlOffset + 1;  // 记录到excess list都是确定有哈希冲突的，所以offset是往后挪一位
                                                        // connect to child
          hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry;  // add child to the excess list
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
    // hashVisibleType == 3的再检查一下可见性？？
    if (hashVisibleType == 3) {   // =3说明没有被重置（即resetVisibleList=false）
      bool isVisibleEnlarged, isVisible;
      if (useSwapping) {  // 有swap的会用更大的imagesize查看可见性。 why？？？
        checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
                                   depthImgSize);
        if (!isVisibleEnlarged)
          hashVisibleType = 0;
      } else {            // 没有swap用正常的imagesize查看可见性
        checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize,
                                    depthImgSize);
        if (!isVisible) {
          hashVisibleType = 0;
        }
      }
      entriesVisibleType[targetIdx] = hashVisibleType;
    }
    // 开启swap，可见 但 不止在显存上，标记一下需要合并
    if (useSwapping) {
      if (hashVisibleType > 0 && swapStates[targetIdx].state != 2)
        swapStates[targetIdx].state = 1;  // =1说明数据同时在内存和显存上，尚未合并
    }
    // 最终记录可见的
    if (hashVisibleType > 0) {
      visibleEntryIDs[noVisibleEntries] = targetIdx;
      noVisibleEntries++;
    }

#if 0
    // "active list", currently disabled
    if (hashVisibleType == 1) {
        activeEntryIDs[noActiveEntries] = targetIdx;
        noActiveEntries++;
    }
#endif
  }

  //! 开启swap，将所有entry中可见但是没有分配空间的分配一下 TODO：之前buildHashAllocAndVisibleTypePP不是都分配过了吗？？？？
  // reallocate deleted ones from previous swap operation
  if (useSwapping) {
    for (int targetIdx = 0; targetIdx < noTotalEntries; targetIdx++) {
      int vbaIdx;
      ITMHashEntry hashEntry = hashTable[targetIdx];

      if (entriesVisibleType[targetIdx] > 0 && hashEntry.ptr == -1) { // 可见但是没有分配空间???
        vbaIdx = lastFreeVoxelBlockId;
        lastFreeVoxelBlockId--;
        if (vbaIdx >= 0)    // 还有剩余空间
          hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
        else
          lastFreeVoxelBlockId++; // Avoid leaks
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
