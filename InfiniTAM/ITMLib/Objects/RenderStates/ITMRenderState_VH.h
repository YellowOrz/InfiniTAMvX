// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

#include "../../../ORUtils/MemoryBlock.h"
#include "../Scene/ITMVoxelBlockHash.h"
#include "ITMRenderState.h"

namespace ITMLib {
/** 使用voxel hasing的raycasting结果。用于SceneReconstruction和visualisation engines。
    Stores the render state used by the SceneReconstruction
    and visualisation engines, as used by voxel hashing.
*/
class ITMRenderState_VH : public ITMRenderState {
private:
  MemoryDeviceType memoryType;

  /** 可见entries列表（即正被tracker处理的）的id
   * A list of "visible entries", that are currently
  being processed by the tracker.
  */
  ORUtils::MemoryBlock<int> *visibleEntryIDs;

  /** 可见entries列表（即正被integration和tracker处理的）的类型
   * A list of "visible entries", that are
  currently being processed by integration
  and tracker.
  */
  ORUtils::MemoryBlock<uchar> *entriesVisibleType;

public:
  /** Number of entries in the live list. */
  int noVisibleEntries;

  ITMRenderState_VH(int noTotalEntries, const Vector2i &imgSize, float vf_min, float vf_max,
                    MemoryDeviceType memoryType = MEMORYDEVICE_CPU)
      : ITMRenderState(imgSize, vf_min, vf_max, memoryType) {
    this->memoryType = memoryType;

    visibleEntryIDs = new ORUtils::MemoryBlock<int>(SDF_LOCAL_BLOCK_NUM, memoryType);
    entriesVisibleType = new ORUtils::MemoryBlock<uchar>(noTotalEntries, memoryType);

    noVisibleEntries = 0;
  }
  ~ITMRenderState_VH() {
    delete visibleEntryIDs;
    delete entriesVisibleType;
  }
  /** 获取可见entries列表（即正被tracker处理的）的id
   * Get the list of "visible entries", that are currently
  processed by the tracker.
  */
  const int *GetVisibleEntryIDs(void) const { return visibleEntryIDs->GetData(memoryType); }
  int *GetVisibleEntryIDs(void) { return visibleEntryIDs->GetData(memoryType); }

  /** 获取visible entry列表。在integration和tracker用到。
   * Get the list of "visible entries", that are
  currently processed by integration and tracker.
  */
  uchar *GetEntriesVisibleType(void) { return entriesVisibleType->GetData(memoryType); }

#ifdef COMPILE_WITH_METAL
  const void *GetVisibleEntryIDs_MB(void) { return visibleEntryIDs->GetMetalBuffer(); }
  const void *GetEntriesVisibleType_MB(void) { return entriesVisibleType->GetMetalBuffer(); }
#endif
};
} // namespace ITMLib
