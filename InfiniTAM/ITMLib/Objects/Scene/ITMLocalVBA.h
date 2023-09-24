// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"

namespace ITMLib {
/** 
 * @brief 存储在device上的voxel block array
 * @note 使用方法可以参考文档“说明材料/localVBA.pptx”
 * Stores the actual voxel content that is referred to by a ITMLib::ITMHashTable.
*/
template<class TVoxel>
class ITMLocalVBA {
 private:
  ORUtils::MemoryBlock<TVoxel> *voxelBlocks;  // voxel block array。数据是从后往前存的
  // 记录VBA中所有被allocate的下标??? 长度为SDF_LOCAL_BLOCK_NUM。场景重置的时候allocationList[i] = i
  ORUtils::MemoryBlock<int> *allocationList;

  MemoryDeviceType memoryType;                // 存储类型：CPU or GPU

 public:
  inline TVoxel *GetVoxelBlocks(void) { return voxelBlocks->GetData(memoryType); }
  inline const TVoxel *GetVoxelBlocks(void) const { return voxelBlocks->GetData(memoryType); }
  int *GetAllocationList(void) { return allocationList->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
  const void* GetVoxelBlocks_MB() const { return voxelBlocks->GetMetalBuffer(); }
  const void* GetAllocationList_MB(void) const { return allocationList->GetMetalBuffer(); }
#endif
  int lastFreeBlockId;  // VBA中剩余空位数, =SDF_LOCAL_BLOCK_NUM-1。因为VBA中数据从后往前存，因此该ID是逐渐变小的
  int allocatedSize;    // 所有block中voxel的数量

  /** 将所有数据（VBA+allocationList+lastFreeBlockId）保存到硬盘上 */
  void SaveToDirectory(const std::string &outputDirectory) const {
    std::string VBFileName = outputDirectory + "voxel.dat";
    std::string ALFileName = outputDirectory + "alloc.dat";
    std::string AllocSizeFileName = outputDirectory + "vba.txt";

    ORUtils::MemoryBlockPersister::SaveMemoryBlock(VBFileName, *voxelBlocks, memoryType);
    ORUtils::MemoryBlockPersister::SaveMemoryBlock(ALFileName, *allocationList, memoryType);

    std::ofstream ofs(AllocSizeFileName.c_str());
    if (!ofs) throw std::runtime_error("Could not open " + AllocSizeFileName + " for writing");

    ofs << lastFreeBlockId << ' ' << allocatedSize;
  }

  /** 从硬盘读取数据（VBA+allocationList+lastFreeBlockId） */
  void LoadFromDirectory(const std::string &inputDirectory) {
    std::string VBFileName = inputDirectory + "voxel.dat";
    std::string ALFileName = inputDirectory + "alloc.dat";
    std::string AllocSizeFileName = inputDirectory + "vba.txt";

    ORUtils::MemoryBlockPersister::LoadMemoryBlock(VBFileName, *voxelBlocks, memoryType);
    ORUtils::MemoryBlockPersister::LoadMemoryBlock(ALFileName, *allocationList, memoryType);

    std::ifstream ifs(AllocSizeFileName.c_str());
    if (!ifs) throw std::runtime_error("Could not open " + AllocSizeFileName + " for reading");

    ifs >> lastFreeBlockId >> allocatedSize;
  }

  /**
   * @brief 构造函数
   * @param[in] memoryType  内存类型，CPU or GPU
   * @param[in] noBlocks    block数量，=SDF_LOCAL_BLOCK_NUM
   * @param[in] blockSize   block中voxel的数量，=SDF_BLOCK_SIZE3
   */
  ITMLocalVBA(MemoryDeviceType memoryType, int noBlocks, int blockSize) {
    this->memoryType = memoryType;

    allocatedSize = noBlocks * blockSize; // 所有block中voxel的数量

    voxelBlocks = new ORUtils::MemoryBlock<TVoxel>(allocatedSize, memoryType);
    allocationList = new ORUtils::MemoryBlock<int>(noBlocks, memoryType);
  }

  ~ITMLocalVBA(void) {
    delete voxelBlocks;
    delete allocationList;
  }

  // Suppress the default copy constructor and assignment operator
  ITMLocalVBA(const ITMLocalVBA &);
  ITMLocalVBA &operator=(const ITMLocalVBA &);
};
}
