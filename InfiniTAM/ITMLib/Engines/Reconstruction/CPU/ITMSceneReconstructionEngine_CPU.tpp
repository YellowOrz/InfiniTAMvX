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

/**
 * ITM场景重建引擎
 * @tparam TVoxel 将3D世界模型表示为小体素块的散列
 * @param [in]scene 这是体素块哈希实现的中心类。它包含CPU上所需的所有数据和GPU上数据结构的指针。
 * @param [in]view 表示单个“视图”，即RGB和深度图像以及所有固有和相对校准信息
 * @param [in]trackingState 存储一些关于当前跟踪状态的内部变量，最重要的是相机姿势
 * @param [in]renderState 存储场景重建和可视化引擎使用的渲染状态。
 */
template <class TVoxel>
void ITMSceneReconstructionEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateIntoScene(
    ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
    const ITMRenderState *renderState) { // ITM渲染状态Vector2i rgbImgSize = view->rgb->noDims;//RGB图像大小
  // ！ 变量初始化
  Vector2i rgbImgSize = view->rgb->noDims;  // 本帧彩色图
  Vector2i depthImgSize = view->depth->noDims;//深度图大小 //获得当前帧的彩色图像 大小 以像素为单位 // 本帧深度图
  float voxelSize = scene->sceneParams->voxelSize;//体素大小 //获得当前帧的深度图像 大小 以像素为单位 // 单个voxel大小 单位通常是米

  Matrix4f M_d, M_rgb;
  Vector4f projParams_d, projParams_rgb;

  ITMRenderState_VH *renderState_vh = (ITMRenderState_VH *) renderState;//ITM渲染状态//存储场景重建和可视化引擎使用的渲染状态。// 场景构造和可视化引擎所使用的状态

  M_d = trackingState->pose_d->GetM();//跟踪状态位姿//当前深度图像位姿  4*4 // 深度相机当前位姿矩阵
  // trafo_rgb_to_depth  此转换将点从 RGB 相机坐标系转移到深度相机坐标系。  calib_inv 变换矩阵的逆矩阵 用于配准 // 配准 深度相机位姿与彩色相机位姿存在一定差异
  if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;//

  projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;  //深度相机的内在参数中的  校准矩阵 4*1 // 校准的深度相机内参
  projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;  //彩色相机的内在参数中的 校准矩阵 4*1 // 校准的彩色相机内参

  float mu = scene->sceneParams->mu;    // TSDF的截断值对应的距离
  int maxW = scene->sceneParams->maxW;  // voxel的最大观测次数，用来融合；超过后若还要融合，采用滑窗方式  // voxel的最大观测次数

  float *depth = view->depth->GetData(MEMORYDEVICE_CPU);  //获取深度图像的指针
  float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);//获取深度图像置信度的指针   置信度为 被摄像头观测的次数，出现在其视野里的次数
  Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);  //获取彩色图的指针
  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();  //获得体素块 // 获取voxel block指针
  ITMHashEntry *hashTable = scene->index.GetEntries();  //获得哈希表指针 //ITM哈希表 // 获取hash列表指针

  int *visibleEntryIds = renderState_vh->GetVisibleEntryIDs();  //可见voxel blocks 的ID 的指针 //可见条目id  // 获取当前可见的voxel block列表的ID指针
  int noVisibleEntries = renderState_vh->noVisibleEntries;  //实时的条目数 //不可见条目 // 当前voxel block的数量

  bool stopIntegratingAtMaxW = scene->sceneParams->stopIntegratingAtMaxW;  //到达最大观测次数后是否继续融合
  //bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
#pragma omp parallel for //表示接下来的循环将被多线程执行
//pragma omp parallel for是OpenMP中的一个指令，表示接下来的for循环将被多线程执行，另外每次循环之间不能有关系
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
  for (int entryId = 0; entryId < noVisibleEntries; entryId++) {    //遍历hash table //遍历不可见条目
    Vector3i globalPos;
    const ITMHashEntry &currentHashEntry = hashTable[visibleEntryIds[entryId]];//建立ITM哈希表  //获取hash列表的单个条目

    if (currentHashEntry.ptr < 0) continue;  //如果ptr为- 则该block没有储存数据  // 判断体素块是否为空

    //世界坐标系中目标三维体素位置 //在voxel block 的坐标 大小8x8x8
    globalPos.x = currentHashEntry.pos.x;   //该block的ID坐标（应该是世界坐标系下的坐标）
    globalPos.y = currentHashEntry.pos.y;
    globalPos.z = currentHashEntry.pos.z;
    globalPos *= SDF_BLOCK_SIZE;    //


    TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);   //SDF_BLOCK_SIZE3 512
    //构建8*8*8模型
    for (int z = 0; z < SDF_BLOCK_SIZE; z++)
      for (int y = 0; y < SDF_BLOCK_SIZE; y++)
        for (int x = 0; x < SDF_BLOCK_SIZE; x++) {//循环整个block 的每一个voxel
          Vector4f pt_model;
          int locId;

          locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;  //给每个voxel一个ID //TODO(wangyuren)自己规定的计算方式？

          if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;
          // stopIntegratingAtMaxW  到达最大观测次数后是否继续融合  maxW  voxel的最大观测次数，用来融合

          //if (approximateIntegration) if (localVoxelBlock[locId].w_depth != 0) continue;
          //TODO（h）：与投影成图像 TSDF 有关 复习kinectfusion
          //3d点在局部SDF_BLOCK_SIZE中坐标 //更新到整个模型的坐标
          pt_model.x = (float) (globalPos.x + x) * voxelSize;
          pt_model.y = (float) (globalPos.y + y) * voxelSize;
          pt_model.z = (float) (globalPos.z + z) * voxelSize;   //globalPos 为卷角坐标*8  voxelSize 单位米
          pt_model.w = 1.0f;  // 为了后续使用齐次坐标？

          //对体素块进行更新 //各项体素数据更新
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
  int *voxelAllocationList = scene->localVBA.GetAllocationList();     // ???
  int *excessAllocationList = scene->index.GetExcessAllocationList(); // ???获得溢出的列表的ID？
  ITMHashEntry *hashTable = scene->index.GetEntries();                // hash table
  ITMHashSwapState *swapStates =                                      // 数据传输状态（内存和显存之间）
      scene->globalCache != NULL ? scene->globalCache->GetSwapStates(false) : 0;
  int *visibleEntryIDs = renderState_vh->GetVisibleEntryIDs();         // 可见entrys的ID
  uchar *entriesVisibleType = renderState_vh->GetEntriesVisibleType(); // 可见entrys的类型？？？
  for (int i = 0; i < renderState_vh->noVisibleEntries; i++)  // 上一帧可见的？？？
    entriesVisibleType[visibleEntryIDs[i]] = 3; // visible at previous frame and unstreamed
  uchar *entriesAllocType = this->entriesAllocType->GetData(MEMORYDEVICE_CPU);
  int noTotalEntries = scene->index.noTotalEntries;
  memset(entriesAllocType, 0, noTotalEntries);
  Vector4s *blockCoords = this->blockCoords->GetData(MEMORYDEVICE_CPU); // 获得块坐标？

  bool useSwapping = scene->globalCache != NULL;                        // 是否支持内存-显存之间的数据交换

  float oneOverVoxelSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);         // block实际边长的倒数，方便后面计算 
                                                                        //TODO：不应该叫voxelsize

  int lastFreeVoxelBlockId = scene->localVBA.lastFreeBlockId;           // 正在用的的block中还未被占用的？？？
  int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

  int noVisibleEntries = 0;

  //! build hashVisibility
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
          lastFreeVoxelBlockId--;//voxel block array剩余空位

          if (vbaIdx >= 0) //there is room in the voxel block array 体素块阵列中有空间
          {
            Vector4s pt_block_all = blockCoords[targetIdx];//目标块三维世界坐标

            ITMHashEntry hashEntry;
            hashEntry.pos.x = pt_block_all.x;
            hashEntry.pos.y = pt_block_all.y;
            hashEntry.pos.z = pt_block_all.z;
            hashEntry.ptr = voxelAllocationList[vbaIdx];//体素块数组地址
            hashEntry.offset = 0;

            //目标块三维世界坐标和体素块数组地址填充哈希表
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
            Vector4s pt_block_all = blockCoords[targetIdx];//目标块三维世界坐标位置

            ITMHashEntry hashEntry;
            hashEntry.pos.x = pt_block_all.x;
            hashEntry.pos.y = pt_block_all.y;
            hashEntry.pos.z = pt_block_all.z;
            hashEntry.ptr = voxelAllocationList[vbaIdx];//体素块数组地址
            hashEntry.offset = 0;//偏移量，用于定位每个特定体素块的体素数据

            //将链接列表的枚举将移动到超额分配列表
            int exlOffset = excessAllocationList[exlIdx];

            //更改链接列表中最后找到的条目指向新填充的条目
            hashTable[targetIdx].offset = exlOffset + 1; //connect to child

            //目标块三维世界坐标和体素块数组地址填充哈希表
            hashTable[SDF_BUCKET_NUM + exlOffset] = hashEntry; //add child to the excess list

            //条目标记类型为可见
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
