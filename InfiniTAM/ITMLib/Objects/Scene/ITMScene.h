// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"
#include "../../Utils/ITMSceneParams.h"

namespace ITMLib {
/** 用voxel hashing表示的三维模型
 * Represents the 3D world model as a hash of small voxel blocks
*/
template<class TVoxel, class TIndex>
class ITMScene {
 public:
  /** 场景的参数。比如voxel size。
   * Scene parameters like voxel size etc. */
  const ITMSceneParams *sceneParams;

  /** hash table。
   * Hash table to reference the 8x8x8 blocks */
  TIndex index;

  /** 正在用的的block（默认是8*8*8大小的voxel）。
   * @note 存储的最小元素是voxel，即将单个block中的voxel拉成一维的存入其中。
   *       若用了CUDA，存在显存；若只用了CPU，存在内存
   * Current local content of the 8x8x8 voxel blocks -- stored host or device */
  ITMLocalVBA<TVoxel> localVBA;

  /** 暂时不用的block
   * @note 不管是CPU还是GPU，都是存在内存。只有开启swap后才会启用。
   * Global content of the 8x8x8 voxel blocks -- stored on host only */
  ITMGlobalCache<TVoxel> *globalCache;

  /** 
   * @brief 将三维模型的数据（block+hash table）存到硬盘 //TODO: 不存globalCache吗？
   * @param[in] outputDirectory 保存路径
   */
  void SaveToDirectory(const std::string &outputDirectory) const {
    localVBA.SaveToDirectory(outputDirectory);
    index.SaveToDirectory(outputDirectory);
  }

  /**
   * @brief 从硬盘读取 三维模型的数据（block+hash table）
   * @param[in] outputDirectory 文件所在路径
   */
  void LoadFromDirectory(const std::string &outputDirectory) {
    localVBA.LoadFromDirectory(outputDirectory);
    index.LoadFromDirectory(outputDirectory);
  }

  ITMScene(const ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType)
      : sceneParams(_sceneParams), index(_memoryType),
        localVBA(_memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize()) {
    if (_useSwapping)
      globalCache = new ITMGlobalCache<TVoxel>();
    else
      globalCache = NULL;
  }

  ~ITMScene(void) {
    if (globalCache != NULL) delete globalCache;
  }

  // 删除 拷贝构造。Suppress the default copy constructor and assignment operator
  ITMScene(const ITMScene &);
  // 删除 赋值运算符
  ITMScene &operator=(const ITMScene &);
};
}
