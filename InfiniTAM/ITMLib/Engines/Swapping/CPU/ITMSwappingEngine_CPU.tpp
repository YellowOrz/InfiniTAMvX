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
  ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;             // 初始化场景全局缓存的指针

  ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);     // 初始化交换状态指针为否

  int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);    // 初始化需要存入的本地ID的状态指针为否

  TVoxel *syncedVoxelBlocks_global = globalCache->GetSyncedVoxelBlocks(false);  //
  bool *hasSyncedData_global = globalCache->GetHasSyncedData(false);    // 初始化已经同步的
  int *neededEntryIDs_global = globalCache->GetNeededEntryIDs(false);   // 初始化需要存入的全局ID的状态指针为否

  int noTotalEntries = globalCache->noTotalEntries;                     // 总条目
  // 统计信息还没有合并的需要存入的条目数量并记录其ID
  // state = 0 最近的数据在主机上，数据目前不在活动内存中
  // state = 1 主机上的数据和活动内存中的数据，信息还没有合并
  // state = 2 最近的数据在活动内存中，应该在某个时候将这些数据保存回主机
  int noNeededEntries = 0;  // 初始化需要存入的条目数为0
  for (int entryId = 0; entryId < noTotalEntries; entryId++) {
    if (noNeededEntries >= SDF_TRANSFER_BLOCK_NUM) break;   // SDF_TRANSFER_BLOCK_NUM 在一次交换操作中传输的最大块数，大小为0x1000
    if (swapStates[entryId].state == 1) {
      neededEntryIDs_local[noNeededEntries] = entryId;
      noNeededEntries++;
    }
  }

  // would copy neededEntryIDs_local into neededEntryIDs_global here

  if (noNeededEntries > 0) {
    memset(syncedVoxelBlocks_global, 0, noNeededEntries * SDF_BLOCK_SIZE3 * sizeof(TVoxel));
    memset(hasSyncedData_global, 0, noNeededEntries * sizeof(bool));
    for (int i = 0; i < noNeededEntries; i++) {
      int entryId = neededEntryIDs_global[i];

      if (globalCache->HasStoredData(entryId)) {
        hasSyncedData_global[i] = true;
        memcpy(syncedVoxelBlocks_global + i * SDF_BLOCK_SIZE3,
               globalCache->GetStoredVoxelBlock(entryId),
               SDF_BLOCK_SIZE3 * sizeof(TVoxel));
      }
    }
  }

  // would copy syncedVoxelBlocks_global and hasSyncedData_global and syncedVoxelBlocks_local and hasSyncedData_local here

  return noNeededEntries;
}

template<class TVoxel>
void ITMSwappingEngine_CPU<TVoxel, ITMVoxelBlockHash>::IntegrateGlobalIntoLocal(ITMScene<TVoxel,
                                                                                         ITMVoxelBlockHash> *scene,
                                                                                ITMRenderState *renderState) {
  ITMGlobalCache<TVoxel> *globalCache = scene->globalCache;

  ITMHashEntry *hashTable = scene->index.GetEntries();

  ITMHashSwapState *swapStates = globalCache->GetSwapStates(false);

  TVoxel *syncedVoxelBlocks_local = globalCache->GetSyncedVoxelBlocks(false);
  bool *hasSyncedData_local = globalCache->GetHasSyncedData(false);
  int *neededEntryIDs_local = globalCache->GetNeededEntryIDs(false);

  TVoxel *localVBA = scene->localVBA.GetVoxelBlocks();

  int noNeededEntries = this->LoadFromGlobalMemory(scene);

  int maxW = scene->sceneParams->maxW;

  for (int i = 0; i < noNeededEntries; i++) {
    int entryDestId = neededEntryIDs_local[i];

    if (hasSyncedData_local[i]) {
      TVoxel *srcVB = syncedVoxelBlocks_local + i * SDF_BLOCK_SIZE3;
      TVoxel *dstVB = localVBA + hashTable[entryDestId].ptr * SDF_BLOCK_SIZE3;

      for (int vIdx = 0; vIdx < SDF_BLOCK_SIZE3; vIdx++) {
        CombineVoxelInformation<TVoxel::hasColorInformation, TVoxel>::compute(srcVB[vIdx], dstVB[vIdx], maxW);
      }
    }

    swapStates[entryDestId].state = 2;
  }
}

template<class TVoxel>
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