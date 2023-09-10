// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ITMSwappingEngine_CPU.h"

#include "../Shared/ITMSwappingEngine_Shared.h"
#include "../../../Objects/RenderStates/ITMRenderState_VH.h"
using namespace ITMLib;

template<class TVoxel>
ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::ITMSwappingEngine_CPU(void) {
}

template<class TVoxel>
ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::~ITMSwappingEngine_CPU(void) {
}

template<class TVoxel>
int ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::LoadFromGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene) {
  //! 准备
  ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;             // swapping所需变量
  ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);     // 数据传输状态
  int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);    // 需要从global memory中读取的entry id ？？？
  TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);  // 读取出来的voxel
  bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);    // 记录是否读取成功
  int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);   // TODO: 跟neededEntryIDs_local重复了
  int noTotalEntries = globalCache->noTotalEntries;                     // 整个场景中的entry总数
  //! 找出所有要合并的entry id
  int noNeededEntries = 0;  
  for (int entryId = 0; entryId < noTotalEntries; entryId++) {
    if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;   // 一次性传输大小有限制，默认为0x1000 // TODO: 超过的咋办？？？
    if (swapStates[entryId].state == 1) {   // =1表示数据同时在内存和显存上，尚未合并
      neededEntryIDs_local[noNeededEntries] = entryId;
      noNeededEntries++;
    }
  }

  //! 将所有要合并的entry对应的block数据读取出来。would copy neededEntryIDs_local into neededEntryIDs_global here
  if (noNeededEntries > 0) {   // 用两个for循环是为了能一次性初始化完整
    // 初始化
    memset(syncedVoxelBlocks_global, 0, noNeededEntries * SDF_BLOCK_SIZE3 * sizeof(TVoxel));
    memset(hasSyncedData_global, 0, noNeededEntries * sizeof(bool));
    // 拷贝每一个block
    for (int i = 0; i < noNeededEntries; i++) {
      int entryId = neededEntryIDs_global[i];

      if (globalCache->HasStoredData(entryId)) {    // 已经存在Host上的才读取出来。没有读个屁！
        hasSyncedData_global[i] = true;
        memcpy(syncedVoxelBlocks_global + i * SDF_BLOCK_SIZE3, globalCache->GetStoredVoxelBlock(entryId),
               SDF_BLOCK_SIZE3 * sizeof(TVoxel));
      }
    }
  }

  // would copy syncedVoxelBlocks_global and hasSyncedData_global and syncedVoxelBlocks_local and hasSyncedData_local here
  // TODO: 上面这句注释是要说啥？？？
  return noNeededEntries;
}

template <class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateGlobalIntoLocal(  // TODO：下次从这儿开始
    ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState) {
  //! 从host端读取swapstate=1的block数据
  int noNeededEntries = this->LoadFromGlobalMemory(scene);
  //! 取出融合所需数据
  ITMHashEntry *hashTable = scene->index.GetEntries();        // hash table
  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();        // device端voxel block array
  int maxW = scene->sceneParams->maxW;                        // voxel的最大观测次数

  ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;   // swapping所需变量
  ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);   // 每个entry的传输状态。=1在CPU和GPU，需要合并
  /** @note 以下三个数组的长度都为一次性传输block的最大数量 */
  TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false); // transfer buffer
  bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);     // transfer buffer中每个block是否传输完成
  int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);    // transfer buffer中每个block的entry id
  //! 将host的每一个block融入device
  for (int i = 0; i < noNeededEntries; i++) {
    int entryDestId = neededEntryIDs_local[i];

    if (hasSyncedData_local[i]) {   // 读取成功的才融合
      TVoxel *srcVB = syncedVoxelBlocks_local + i * SDF_BLOCK_SIZE3;            // host的block
      TVoxel *dstVB = localVBA + hashTable[entryDestId].ptr * SDF_BLOCK_SIZE3;  // device的block
      // 融合每一个voxel
      for (int vIdx = 0; vIdx < SDF_BLOCK_SIZE3; vIdx++) {
        CombineVoxelInformation<TVoxel::hasColorInformation, TVoxel>::compute(srcVB[vIdx], dstVB[vIdx], maxW);
      }
    }

    swapStates[entryDestId].state = 2;    // 融合完成后记得改状态。=2表示存在device端
  }
}

template<class TVoxel>  // TODO：下次从这儿开始
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::SaveToGlobalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
                                                                          ITMRenderState *renderState) {
  ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

  ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

  ITMHashEntry *hashTable = scene->index.GetEntries();
  uchar *entriesVisibleType = ((ITMRenderState_VH *) renderState)->GetEntriesVisibleType();

  TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
  bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
  int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

  TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);
  bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);
  int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);

  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
  int *voxelAllocationList = scene->localVBA.GetAllocationList();

  int noTotalEntries = globalCache->noTotalEntries;

  int noNeededEntries = 0;
  int noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;

  for (int entryDestId = 0; entryDestId < noTotalEntries; entryDestId++) {
    if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;

    int localPtr = hashTable[entryDestId].ptr;
    ITMHashSwapState &swapState = swapStates[entryDestId];

    if (swapState.state == 2 && localPtr >= 0 && entriesVisibleType[entryDestId] == 0) {
      TVoxel *localVBALocation = localVBA + localPtr * SDF_BLOCK_SIZE3;

      neededEntryIDs_local[noNeededEntries] = entryDestId;

      hasSyncedData_local[noNeededEntries] = true;
      memcpy(syncedVoxelBlocks_local + noNeededEntries * SDF_BLOCK_SIZE3,
             localVBALocation,
             SDF_BLOCK_SIZE3 * sizeof(TVoxel));

      swapStates[entryDestId].state = 0;

      int vbaIdx = noAllocatedVoxelEntries;
      if (vbaIdx < SDF_BUCKET_NUM - 1) {
        noAllocatedVoxelEntries++;
        voxelAllocationList[vbaIdx + 1] = localPtr;
        hashTable[entryDestId].ptr = -1;

        for (int i = 0; i < SDF_BLOCK_SIZE3; i++) localVBALocation[i] = TVoxel();
      }

      noNeededEntries++;
    }
  }

  scene->localVBA.lastFreeBlockId = noAllocatedVoxelEntries;

  // would copy neededEntryIDs_local, hasSyncedData_local and syncedVoxelBlocks_local into *_global here

  if (noNeededEntries > 0) {
    for (int entryId = 0; entryId < noNeededEntries; entryId++) {
      if (hasSyncedData_global[entryId])
        globalCache->SetStoredData(neededEntryIDs_global[entryId],
                                   syncedVoxelBlocks_global + entryId * SDF_BLOCK_SIZE3);
    }
  }
}

template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::CleanLocalMemory(ITMScene<TVoxel, ITMVoxelBlockHash> *scene,
                                                                        ITMRenderState *renderState) {
  ITMHashEntry *hashTable = scene->index.GetEntries();
  uchar *entriesVisibleType = ((ITMRenderState_VH *) renderState)->GetEntriesVisibleType();

  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
  int *voxelAllocationList = scene->localVBA.GetAllocationList();

  int noTotalEntries = scene->index.noTotalEntries;

  int noNeededEntries = 0;
  int noAllocatedVoxelEntries = scene->localVBA.lastFreeBlockId;

  for (int entryDestId = 0; entryDestId < noTotalEntries; entryDestId++) {
    if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;

    int localPtr = hashTable[entryDestId].ptr;

    if (localPtr >= 0 && entriesVisibleType[entryDestId] == 0) {
      TVoxel *localVBALocation = localVBA + localPtr * SDF_BLOCK_SIZE3;

      int vbaIdx = noAllocatedVoxelEntries;
      if (vbaIdx < SDF_BUCKET_NUM - 1) {
        noAllocatedVoxelEntries++;
        voxelAllocationList[vbaIdx + 1] = localPtr;
        hashTable[entryDestId].ptr = -1;

        for (int i = 0; i < SDF_BLOCK_SIZE3; i++) localVBALocation[i] = TVoxel();
      }

      noNeededEntries++;
    }
  }

  scene->localVBA.lastFreeBlockId = noAllocatedVoxelEntries;
}