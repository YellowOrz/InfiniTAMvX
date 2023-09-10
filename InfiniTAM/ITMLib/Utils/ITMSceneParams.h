// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

namespace ITMLib {
/** \brief
    Stores parameters of a scene like voxel size
*/
class ITMSceneParams {
 public:
  /// Size of a voxel, usually given in meters.
  float voxelSize;  // voxel size，单位米

  /** \brief
      Fallback parameters: consider only parts of the
      scene from @p viewFrustum_min in front of the camera
      to a distance of @p viewFrustum_max. Usually the
      actual depth range should be determined
      automatically by a ITMLib::Engine::ITMVisualisationEngine.
  */
  float viewFrustum_min, viewFrustum_max;   // 视锥中，深度最近和最远距离，单位米

  /** \brief
   * TSDF的截断值对应的距离。单位为米。TSDF值的变化间隔=mu÷voxelSize。
   * ITMLibSettings中默认设定为0.2。
   * Encodes the width of the band of the truncated
      signed distance transform that is actually stored
      in the volume. This is again usually specified in
      meters. The resulting width in voxels is @ref mu
      divided by @ref voxelSize.
  */
  float mu;

  /** \brief
      Up to @ref maxW observations per voxel are averaged.
      Beyond that a sliding average is computed.
  */
  int maxW; // voxel的最大观测次数，用来限制voxel的权重（因为权重=观测次数） // TODO：并没有用到滑窗呀

  /** Stop integration once maxW has been reached. */
  bool stopIntegratingAtMaxW;   // 到达最大观测次数后是否继续融合

  ITMSceneParams(void) {}

  ITMSceneParams(float mu, int maxW, float voxelSize,
                 float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW) {
    this->mu = mu;
    this->maxW = maxW;
    this->voxelSize = voxelSize;
    this->viewFrustum_min = viewFrustum_min;
    this->viewFrustum_max = viewFrustum_max;
    this->stopIntegratingAtMaxW = stopIntegratingAtMaxW;
  }

  explicit ITMSceneParams(const ITMSceneParams *sceneParams) { this->SetFrom(sceneParams); }

  void SetFrom(const ITMSceneParams *sceneParams) {
    this->voxelSize = sceneParams->voxelSize;
    this->viewFrustum_min = sceneParams->viewFrustum_min;
    this->viewFrustum_max = sceneParams->viewFrustum_max;
    this->mu = sceneParams->mu;
    this->maxW = sceneParams->maxW;
    this->stopIntegratingAtMaxW = sceneParams->stopIntegratingAtMaxW;
  }
};
}
