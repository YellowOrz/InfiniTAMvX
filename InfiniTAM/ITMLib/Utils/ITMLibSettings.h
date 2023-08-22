// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMSceneParams.h"
#include "ITMSurfelSceneParams.h"
#include "../../ORUtils/MemoryDeviceType.h"

namespace ITMLib {
class ITMLibSettings {
 public:
  /// The device used to run the DeviceAgnostic code
  /** 使用设备 */
  typedef enum {
    DEVICE_CPU,
    DEVICE_CUDA,  /**< 英伟达GPU */
    DEVICE_METAL  /**< 苹果（opengl+opencl）*/
  } DeviceType;
  /** 重建失败后的操作 */
  typedef enum {
    FAILUREMODE_RELOCALISE,       /**< 重定位 */
    FAILUREMODE_IGNORE,           /**< 忽略，继续重建 */
    FAILUREMODE_STOP_INTEGRATION  /**< 停止重建 */
  } FailureMode;
  /** 内存交换策略 */
  typedef enum {
    SWAPPINGMODE_DISABLED,  /**< 关闭 */
    SWAPPINGMODE_ENABLED,   /**< 开启 */
    SWAPPINGMODE_DELETE     /**< 删除不可见的（闭环检测模式下不支持）*/
  } SwappingMode;
  /** SLAM系统模式 */
  typedef enum {
    LIBMODE_BASIC,          /**< 使用voxel的基础模式 */ // TODO(xzf)：不确定
    LIBMODE_BASIC_SURFELS,  /**< 使用surfel的基础模式 */
    LIBMODE_LOOPCLOSURE     /**< 回环检测模式 */
  } LibMode;

  /// Select the type of device to use
  DeviceType deviceType;    // 计算设备：CPU or GPU(cuda)

  bool useApproximateRaycast;   // 是否使用增量式的raycasting（这是一种近似）

  bool useBilateralFilter;      // 是否使用双边滤波

  /// For ITMColorTracker: skip every other point in energy function evaluation.
  bool skipPoints;    // raycasting渲染RGB的点云时，是否要跳过一些点   // TODO(xzf):用来减少体积 ？？？

  bool createMeshingEngine;  // 是否创建Mesh模块（包含marching cube和mesh提取），会占用大量内存

  FailureMode behaviourOnFailure;   // 重建失败的操作
  SwappingMode swappingMode;  // 内存交换策略
  LibMode libMode;    // SLAM系统模式

  const char *trackerConfig;  // 跟踪配置

  /// Further, scene specific parameters such as voxel size
  ITMSceneParams sceneParams;
  ITMSurfelSceneParams surfelSceneParams;

  ITMLibSettings(void);
  virtual ~ITMLibSettings(void) {}

  // Suppress the default copy constructor and assignment operator
  ITMLibSettings(const ITMLibSettings &);
  ITMLibSettings &operator=(const ITMLibSettings &);

  MemoryDeviceType GetMemoryType() const;
};
}
