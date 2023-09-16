// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "ITMVoxelBlockHash.h"
#include "../../../ORUtils/CUDADefines.h"

namespace ITMLib {
struct ITMHashSwapState {   // 为啥不弄成一个枚举类型？？？
  ///  0 - 最近的数据在内存上，不在显存中
  ///  1 - 数据同时在内存和显存上，尚未合并
  ///  2 - 大多数最近的数据都在显存中，应该在某个时候传到内存中
  ///  0 - most recent data is on host, data not currently in active
  ///      memory
  ///  1 - data both on host and in active memory, information has not
  ///      yet been combined
  ///  2 - most recent data is in active memory, should save this data
  ///      back to host at some point
  uchar state;
};

template<class TVoxel>
class ITMGlobalCache {
 private:
  /** @note 下面三个数组的长度都为noTotalEntries */
  bool *hasStoredData;                                    // 每个entry（block）是否已经存在Host上
  TVoxel *storedVoxelBlocks;                              // 论文Fig.5中的Host Voxel Memory？？？
  ITMHashSwapState *swapStates_host, *swapStates_device;  // 每个entry的传输状态。具体意义见ITMHashSwapState::state
  /** @note 以下三个数组的长度都为一次性传输block的最大数量 */
  bool *hasSyncedData_host, *hasSyncedData_device;            // transfer buffer中每个block是否传输完成
  TVoxel *syncedVoxelBlocks_host, *syncedVoxelBlocks_device;  // 论文Fig.5中的transfer buffer？？？
  int *neededEntryIDs_host, *neededEntryIDs_device;           // transfer buffer中每个block的entry id
 public:
  /**
   * @brief 拷贝 block数据 到 host端的指定位置
   * @param[in] address host端的指定位置。就是第几个block
   * @param[in] data block数据
   */
  inline void SetStoredData(int address, TVoxel *data) {
    hasStoredData[address] = true;  // 记录
    memcpy(storedVoxelBlocks + address * SDF_BLOCK_SIZE3, data, sizeof(TVoxel) * SDF_BLOCK_SIZE3);  // 拷贝
  }
  inline bool HasStoredData(int address) const { return hasStoredData[address]; }
  inline TVoxel *GetStoredVoxelBlock(int address) { return storedVoxelBlocks + address * SDF_BLOCK_SIZE3; }

  bool *GetHasSyncedData(bool useGPU) const { return useGPU ? hasSyncedData_device : hasSyncedData_host; }
  TVoxel *GetSyncedVoxelBlocks(bool useGPU) const { return useGPU ? syncedVoxelBlocks_device : syncedVoxelBlocks_host; }

  ITMHashSwapState *GetSwapStates(bool useGPU) { return useGPU ? swapStates_device : swapStates_host; }
  int *GetNeededEntryIDs(bool useGPU) { return useGPU ? neededEntryIDs_device : neededEntryIDs_host; }

  int noTotalEntries;     // scene中entry总数 = bucket(ordered list)数量 + excess entry数量   // TODO: 为啥放到public？

  ITMGlobalCache() : noTotalEntries(SDF_BUCKET_NUM + SDF_EXCESS_LIST_SIZE) {
    hasStoredData = (bool *) malloc(noTotalEntries * sizeof(bool));
    storedVoxelBlocks = (TVoxel *) malloc(noTotalEntries * sizeof(TVoxel) * SDF_BLOCK_SIZE3);
    memset(hasStoredData, 0, noTotalEntries);

    swapStates_host = (ITMHashSwapState *) malloc(noTotalEntries * sizeof(ITMHashSwapState));
    memset(swapStates_host, 0, sizeof(ITMHashSwapState) * noTotalEntries);

#ifndef COMPILE_WITHOUT_CUDA
    ORcudaSafeCall(cudaMallocHost((void **) &syncedVoxelBlocks_host,
                                  SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
    ORcudaSafeCall(cudaMallocHost((void **) &hasSyncedData_host, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));
    ORcudaSafeCall(cudaMallocHost((void **) &neededEntryIDs_host, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));

    ORcudaSafeCall(cudaMalloc((void **) &swapStates_device, noTotalEntries * sizeof(ITMHashSwapState)));
    ORcudaSafeCall(cudaMemset(swapStates_device, 0, noTotalEntries * sizeof(ITMHashSwapState)));

    ORcudaSafeCall(cudaMalloc((void **) &syncedVoxelBlocks_device,
                              SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3));
    ORcudaSafeCall(cudaMalloc((void **) &hasSyncedData_device, SDF_TRANSFER_BLOCK_NUM * sizeof(bool)));

    ORcudaSafeCall(cudaMalloc((void **) &neededEntryIDs_device, SDF_TRANSFER_BLOCK_NUM * sizeof(int)));
#else
    syncedVoxelBlocks_host = (TVoxel *) malloc(SDF_TRANSFER_BLOCK_NUM * sizeof(TVoxel) * SDF_BLOCK_SIZE3);
    hasSyncedData_host = (bool *) malloc(SDF_TRANSFER_BLOCK_NUM * sizeof(bool));
    neededEntryIDs_host = (int *) malloc(SDF_TRANSFER_BLOCK_NUM * sizeof(int));
#endif
  }

  void SaveToFile(char *fileName) const {
    TVoxel *storedData = storedVoxelBlocks;

    FILE *f = fopen(fileName, "wb");

    fwrite(hasStoredData, sizeof(bool), noTotalEntries, f);
    for (int i = 0; i < noTotalEntries; i++) {
      fwrite(storedData, sizeof(TVoxel) * SDF_BLOCK_SIZE3, 1, f);
      storedData += SDF_BLOCK_SIZE3;
    }

    fclose(f);
  }

  void ReadFromFile(char *fileName) {
    TVoxel *storedData = storedVoxelBlocks;
    FILE *f = fopen(fileName, "rb");

    size_t tmp = fread(hasStoredData, sizeof(bool), noTotalEntries, f);
    if (tmp == (size_t) noTotalEntries) {
      for (int i = 0; i < noTotalEntries; i++) {
        fread(storedData, sizeof(TVoxel) * SDF_BLOCK_SIZE3, 1, f);
        storedData += SDF_BLOCK_SIZE3;
      }
    }

    fclose(f);
  }

  ~ITMGlobalCache(void) {
    free(hasStoredData);
    free(storedVoxelBlocks);

    free(swapStates_host);

#ifndef COMPILE_WITHOUT_CUDA
    ORcudaSafeCall(cudaFreeHost(hasSyncedData_host));
    ORcudaSafeCall(cudaFreeHost(syncedVoxelBlocks_host));
    ORcudaSafeCall(cudaFreeHost(neededEntryIDs_host));

    ORcudaSafeCall(cudaFree(swapStates_device));
    ORcudaSafeCall(cudaFree(syncedVoxelBlocks_device));
    ORcudaSafeCall(cudaFree(hasSyncedData_device));
    ORcudaSafeCall(cudaFree(neededEntryIDs_device));
#else
    free(hasSyncedData_host);
    free(syncedVoxelBlocks_host);
    free(neededEntryIDs_host);
#endif
  }
};
}
