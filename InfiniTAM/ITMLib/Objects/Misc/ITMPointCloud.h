// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib {
class ITMPointCloud {
 public:
  uint noTotalPoints;

  ORUtils::Image<Vector4f> *locations, *colours;  // 点云中的坐标和颜色（不一定是RGB，可以是法向量）。为啥是4f？？？

  explicit ITMPointCloud(Vector2i imgSize, MemoryDeviceType memoryType) {
    this->noTotalPoints = 0;

    locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
    colours = new ORUtils::Image<Vector4f>(imgSize, memoryType);
  }

  void UpdateHostFromDevice() {
    this->locations->UpdateHostFromDevice();
    this->colours->UpdateHostFromDevice();
  }

  void UpdateDeviceFromHost() {
    this->locations->UpdateDeviceFromHost();
    this->colours->UpdateDeviceFromHost();
  }

  ~ITMPointCloud() {
    delete locations;
    delete colours;
  }

  // Suppress the default copy constructor and assignment operator
  ITMPointCloud(const ITMPointCloud &);
  ITMPointCloud &operator=(const ITMPointCloud &);
};
}
