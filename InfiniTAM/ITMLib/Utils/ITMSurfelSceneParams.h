// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

namespace ITMLib {
/**
 * \brief An instance of this struct can be used to specify parameters for a surfel scene.
 */
struct ITMSurfelSceneParams {
  //#################### PUBLIC VARIABLES ####################

  /** The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur. */
  float deltaRadius;  // 完全融合时，新surfel半径 大于 旧surfel 的最大值

  /** The sigma value for the Gaussian used when calculating the sample confidence. */
  float gaussianConfidenceSigma;  // 计算置信度时 使用的高斯分布的σ

  /** The maximum angle allowed between the normals of a pair of surfels if they are to be merged. */
  float maxMergeAngle;  // 融合surfel，法向量夹角的最大差

  /** The maximum distance allowed between a pair of surfels if they are to be merged. */
  float maxMergeDist;  // 融合surfel，距离的最大差

  /** The maximum radius a surfel is allowed to have. */
  float maxSurfelRadius;  // surfel的最大半径

  /** The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged. */
  float minRadiusOverlapFactor;  // 融合时，重叠的最小半径  TODO(xzf): 不确定

  /** The confidence value a surfel must have in order for it to be considered "stable". */
  float stableSurfelConfidence;   // 稳定surfel的置信度

  /** The factor by which to supersample (in each axis) the index image used for finding surfel correspondences. */
  int supersamplingFactor;  // 查找surfel的对应时，（每个轴上的）上采样系数

  /** The maximum depth a surfel must have in order for it to be used for tracking. */
  float trackingSurfelMaxDepth;   // 用于跟踪的surfel的最大深度

  /** The minimum confidence value a surfel must have in order for it to be used for tracking. */
  float trackingSurfelMinConfidence;  // 用于跟踪的surfel的最小置信度

  /** The number of time steps a surfel is allowed to be unstable without being updated before being removed. */
  int unstableSurfelPeriod;   // surfel被删除前，不稳定的最小时间（帧数）

  /** The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative. */
  int unstableSurfelZOffset;    // 渲染时，若没有稳定的surfel，对不稳定的surfel的z轴偏移

  /** Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper. */
  bool useGaussianSampleConfidence;  // 是否使用基于 高斯加权 的置信度（来自Keller的论文）

  /** Whether or not to use surfel merging. */
  bool useSurfelMerging;  // 是否对surfel进行融合

  //#################### CONSTRUCTORS ####################

  /**
   * \brief Constructs a set of surfel scene parameters.
   *
   * \param deltaRadius_                  The maximum fraction by which a new surfel can have a larger radius than the surfel into which it is being fused if full fusion is to occur.
   * \param gaussianConfidenceSigma_      The sigma value for the Gaussian used when calculating the sample confidence.
   * \param maxMergeAngle_                The maximum angle allowed between the normals of a pair of surfels if they are to be merged.
   * \param maxMergeDist_                 The maximum distance allowed between a pair of surfels if they are to be merged.
   * \param maxSurfelRadius_              The maximum radius a surfel is allowed to have.
   * \param minRadiusOverlapFactor_       The minimum factor by which the radii of a pair of surfels must overlap if they are to be merged.
   * \param stableSurfelConfidence_       The confidence value a surfel must have in order for it to be considered "stable".
   * \param supersamplingFactor_          The factor by which to supersample (in each axis) the index image used for finding surfel correspondences.
   * \param trackingSurfelMaxDepth_       The maximum depth a surfel must have in order for it to be used for tracking.
   * \param trackingSurfelMinConfidence_  The minimum confidence value a surfel must have in order for it to be used for tracking.
   * \param unstableSurfelPeriod_         The number of time steps a surfel is allowed to be unstable without being updated before being removed.
   * \param unstableSurfelZOffset_        The z offset to apply to unstable surfels when trying to ensure that they are only rendered if there is no stable alternative.
   * \param useGaussianSampleConfidence_  Whether or not to use a Gaussian-weighted sample confidence as described in the Keller paper.
   * \param useSurfelMerging_             Whether or not to use surfel merging.
   */
  explicit ITMSurfelSceneParams(float deltaRadius_,
                                float gaussianConfidenceSigma_,
                                float maxMergeAngle_,
                                float maxMergeDist_,
                                float maxSurfelRadius_,
                                float minRadiusOverlapFactor_,
                                float stableSurfelConfidence_,
                                int supersamplingFactor_,
                                float trackingSurfelMaxDepth_,
                                float trackingSurfelMinConfidence_,
                                int unstableSurfelPeriod_,
                                int unstableSurfelZOffset_,
                                bool useGaussianSampleConfidence_,
                                bool useSurfelMerging_)
      : deltaRadius(deltaRadius_),
        gaussianConfidenceSigma(gaussianConfidenceSigma_),
        maxMergeAngle(maxMergeAngle_),
        maxMergeDist(maxMergeDist_),
        maxSurfelRadius(maxSurfelRadius_),
        minRadiusOverlapFactor(minRadiusOverlapFactor_),
        stableSurfelConfidence(stableSurfelConfidence_),
        supersamplingFactor(supersamplingFactor_),
        trackingSurfelMaxDepth(trackingSurfelMaxDepth_),
        trackingSurfelMinConfidence(trackingSurfelMinConfidence_),
        unstableSurfelPeriod(unstableSurfelPeriod_),
        unstableSurfelZOffset(unstableSurfelZOffset_),
        useGaussianSampleConfidence(useGaussianSampleConfidence_),
        useSurfelMerging(useSurfelMerging_) {}
};
}
