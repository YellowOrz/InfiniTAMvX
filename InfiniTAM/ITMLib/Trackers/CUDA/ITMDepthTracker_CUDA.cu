// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "../../../ORUtils/CUDADefines.h"
#include "../../Utils/ITMCUDAUtils.h"
#include "../Shared/ITMDepthTracker_Shared.h"
#include "ITMDepthTracker_CUDA.h"

using namespace ITMLib;

struct ITMDepthTracker_CUDA::AccuCell {
  int numPoints;
  float f;
  float g[6];
  float h[6 + 5 + 4 + 3 + 2 + 1];
};

struct ITMDepthTracker_KernelParameters {
  ITMDepthTracker_CUDA::AccuCell *accu;
  float *depth;
  Matrix4f approxInvPose;
  Vector4f *pointsMap;
  Vector4f *normalsMap;
  Vector4f sceneIntrinsics;
  Vector2i sceneImageSize;
  Matrix4f scenePose;
  Vector4f viewIntrinsics;
  Vector2i viewImageSize;
  float distThresh;
};

template <bool shortIteration, bool rotationOnly>
__global__ void depthTrackerOneLevel_g_rt_device(ITMDepthTracker_KernelParameters para);

// host methods

ITMDepthTracker_CUDA::ITMDepthTracker_CUDA(Vector2i imgSize, TrackerIterationType *trackingRegime,
                                           int noHierarchyLevels, float terminationThreshold,
                                           float failureDetectorThreshold, const ITMLowLevelEngine *lowLevelEngine)
    : ITMDepthTracker(imgSize, trackingRegime, noHierarchyLevels, terminationThreshold, failureDetectorThreshold,
                      lowLevelEngine, MEMORYDEVICE_CUDA) {
  ORcudaSafeCall(cudaMallocHost((void **)&accu_host, sizeof(AccuCell)));
  ORcudaSafeCall(cudaMalloc((void **)&accu_device, sizeof(AccuCell)));
}

ITMDepthTracker_CUDA::~ITMDepthTracker_CUDA(void) {
  ORcudaSafeCall(cudaFreeHost(accu_host));
  ORcudaSafeCall(cudaFree(accu_device));
}

int ITMDepthTracker_CUDA::ComputeGandH(float &f, float *nabla, float *hessian, Matrix4f approxInvPose) {
  //! 从scene中获取所需信息
  Vector4f *pointsMap = sceneHierarchyLevel->pointsMap->GetData(MEMORYDEVICE_CUDA);   // 三维坐标点。第4个数字是w？？？
  Vector4f *normalsMap = sceneHierarchyLevel->normalsMap->GetData(MEMORYDEVICE_CUDA); // 法向量。第4个数字是w？？？
  Vector4f sceneIntrinsics = sceneHierarchyLevel->intrinsics;                         // 场景相机内参
  Vector2i sceneImageSize = sceneHierarchyLevel->pointsMap->noDims;                   // 场景图像大小
  //! 从当前帧获取所需信息
  float *depth = viewHierarchyLevel->data->GetData(MEMORYDEVICE_CUDA); // 获取当前帧的深度图（以一维存储）
  Vector4f viewIntrinsics = viewHierarchyLevel->intrinsics;            // 当前帧相机内参
  Vector2i viewImageSize = viewHierarchyLevel->data->noDims;           // 当前帧图像大小

  if (iterationType == TRACKER_ITERATION_NONE)
    return 0;
  //! 初始化
  bool shortIteration =
      (iterationType == TRACKER_ITERATION_ROTATION) || (iterationType == TRACKER_ITERATION_TRANSLATION);

  int noPara = shortIteration ? 3 : 6;  // CPU版本中的noParaSQ放在下面的核函数
  
  //! 计算每个像素的 误差、Hessian矩阵 和 Nabla算子
  dim3 blockSize(16, 16);
  dim3 gridSize((int)ceil((float)viewImageSize.x / (float)blockSize.x),
                (int)ceil((float)viewImageSize.y / (float)blockSize.y));
  ORcudaSafeCall(cudaMemset(accu_device, 0, sizeof(AccuCell)));

  struct ITMDepthTracker_KernelParameters args;
  args.accu = accu_device;
  args.depth = depth;
  args.approxInvPose = approxInvPose;
  args.pointsMap = pointsMap;
  args.normalsMap = normalsMap;
  args.sceneIntrinsics = sceneIntrinsics;
  args.sceneImageSize = sceneImageSize;
  args.scenePose = scenePose;
  args.viewIntrinsics = viewIntrinsics;
  args.viewImageSize = viewImageSize;
  args.distThresh = distThresh[levelId];

  switch (iterationType) {
  case TRACKER_ITERATION_ROTATION:    // 只跟踪 旋转
    depthTrackerOneLevel_g_rt_device<true, true><<<gridSize, blockSize>>>(args);
    ORcudaKernelCheck;
    break;
  case TRACKER_ITERATION_TRANSLATION: // 只跟踪 平移
    depthTrackerOneLevel_g_rt_device<true, false><<<gridSize, blockSize>>>(args);
    ORcudaKernelCheck;
    break;
  case TRACKER_ITERATION_BOTH:        // 跟踪 旋转+平移
    depthTrackerOneLevel_g_rt_device<false, false><<<gridSize, blockSize>>>(args);
    ORcudaKernelCheck;
    break;
  default:
    break;
  }

  ORcudaSafeCall(cudaMemcpy(accu_host, accu_device, sizeof(AccuCell), cudaMemcpyDeviceToHost));

  //! 获得最终的Hessian矩阵、Nabla算子 和 误差
  for (int r = 0, counter = 0; r < noPara; r++)   // H的下三角
    for (int c = 0; c <= r; c++, counter++)
      hessian[r + c * 6] = accu_host->h[counter]; // 改成hessian[r+c*6] = hessian[c+r*6] = sumHessian[counter]？？？
  for (int r = 0; r < noPara; ++r)                // H的上三角
    for (int c = r + 1; c < noPara; c++)
      hessian[r + c * 6] = hessian[c + r * 6];

  memcpy(nabla, accu_host->g, noPara * sizeof(float));
  f = (accu_host->numPoints > 100) ? accu_host->f / accu_host->numPoints : 1e5f;  //最终的误差函数

  return accu_host->numPoints;
}

// device functions
/**
 * 计算单个像素的Hessian矩阵、Nabla算子 和 误差
 * @tparam shortIteration     只求解 旋转 or 平移
 * @tparam rotationOnly       
 * @param[out] accu           计算结果，包含Hessian矩阵、Nabla算子、误差
 * @param[in] depth
 * @param[in] approxInvPose   初始位姿，=上一帧的位姿？？？
 * @param[in] pointsMap       场景投影出来的三维点？？？
 * @param[in] normalsMap      投影点对应的法向量
 * @param[in] sceneIntrinsics 场景的
 * @param[in] sceneImageSize
 * @param[in] scenePose
 * @param[in] viewIntrinsics  当前帧的
 * @param[in] viewImageSize
 * @param[in] distThresh      距离阈值，用来剔除误差大的点
 */
template <bool shortIteration, bool rotationOnly>
__device__ void depthTrackerOneLevel_g_rt_device_main(ITMDepthTracker_CUDA::AccuCell *accu, float *depth,
                                                      Matrix4f approxInvPose, Vector4f *pointsMap, Vector4f *normalsMap,
                                                      Vector4f sceneIntrinsics, Vector2i sceneImageSize,
                                                      Matrix4f scenePose, Vector4f viewIntrinsics,
                                                      Vector2i viewImageSize, float distThresh) {
  int x = threadIdx.x + blockIdx.x * blockDim.x, 
      y = threadIdx.y + blockIdx.y * blockDim.y;
  int locId_local = threadIdx.x + threadIdx.y * blockDim.x;

  __shared__ float dim_shared1[256];
  __shared__ float dim_shared2[256];
  __shared__ float dim_shared3[256];
  __shared__ bool should_prefix;
  should_prefix = false;
  __syncthreads();

  //! 转最小二乘问题Ax=b
  const int noPara = shortIteration ? 3 : 6;
  const int noParaSQ = shortIteration ? 3 + 2 + 1 : 6 + 5 + 4 + 3 + 2 + 1;
  float A[noPara];
  float b;
  bool isValidPoint = false;

  if (x < viewImageSize.x && y < viewImageSize.y) {
    isValidPoint = computePerPointGH_Depth_Ab<shortIteration, rotationOnly>(
        A, b, x, y, depth[x + y * viewImageSize.x], viewImageSize, viewIntrinsics, sceneImageSize, sceneIntrinsics,
        approxInvPose, scenePose, pointsMap, normalsMap, distThresh);
    if (isValidPoint)
      should_prefix = true;
  }
  //! 无效的像素退出
  if (!isValidPoint) {
    for (int i = 0; i < noPara; i++)
      A[i] = 0.0f;
    b = 0.0f;
  }
  __syncthreads();
  if (!should_prefix) return;

  //! 有效像素数量求和的规约
  { // reduction for noValidPoints
    dim_shared1[locId_local] = isValidPoint;
    __syncthreads();

    if (locId_local < 128)    // 128是因为block size = (16, 16)。 TODO：可以用for循环替换？
      dim_shared1[locId_local] += dim_shared1[locId_local + 128];
    __syncthreads();
    if (locId_local < 64)
      dim_shared1[locId_local] += dim_shared1[locId_local + 64];
    __syncthreads();

    if (locId_local < 32)
      warpReduce(dim_shared1, locId_local);

    if (locId_local == 0)
      atomicAdd(&(accu->numPoints), (int)dim_shared1[locId_local]);
  }

  //! 误差求和的规约
  { // reduction for energy function value
    dim_shared1[locId_local] = b * b;
    __syncthreads();

    if (locId_local < 128)
      dim_shared1[locId_local] += dim_shared1[locId_local + 128];
    __syncthreads();
    if (locId_local < 64)
      dim_shared1[locId_local] += dim_shared1[locId_local + 64];
    __syncthreads();

    if (locId_local < 32)
      warpReduce(dim_shared1, locId_local);

    if (locId_local == 0)
      atomicAdd(&(accu->f), dim_shared1[locId_local]);
  }

  __syncthreads();

  // reduction for nabla
  //! nabla算子（高斯牛顿中的g）求和的规约
  for (unsigned char paraId = 0; paraId < noPara; paraId += 3) {
    dim_shared1[locId_local] = b * A[paraId + 0];
    dim_shared2[locId_local] = b * A[paraId + 1];
    dim_shared3[locId_local] = b * A[paraId + 2];
    __syncthreads();

    if (locId_local < 128) {
      dim_shared1[locId_local] += dim_shared1[locId_local + 128];
      dim_shared2[locId_local] += dim_shared2[locId_local + 128];
      dim_shared3[locId_local] += dim_shared3[locId_local + 128];
    }
    __syncthreads();
    if (locId_local < 64) {
      dim_shared1[locId_local] += dim_shared1[locId_local + 64];
      dim_shared2[locId_local] += dim_shared2[locId_local + 64];
      dim_shared3[locId_local] += dim_shared3[locId_local + 64];
    }
    __syncthreads();

    if (locId_local < 32) {
      warpReduce(dim_shared1, locId_local);
      warpReduce(dim_shared2, locId_local);
      warpReduce(dim_shared3, locId_local);
    }
    __syncthreads();

    if (locId_local == 0) {
      atomicAdd(&(accu->g[paraId + 0]), dim_shared1[0]);
      atomicAdd(&(accu->g[paraId + 1]), dim_shared2[0]);
      atomicAdd(&(accu->g[paraId + 2]), dim_shared3[0]);
    }
  }

  __syncthreads();

  float localHessian[noParaSQ];
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
  for (unsigned char r = 0, counter = 0; r < noPara; r++) {
#if (defined(__CUDACC__) && defined(__CUDA_ARCH__)) || (defined(__METALC__))
#pragma unroll
#endif
    for (int c = 0; c <= r; c++, counter++)
      localHessian[counter] = A[r] * A[c];
  }

  // reduction for hessian
  //! Hessian矩阵求和的规约  // TODO：可以优化？？？
  for (unsigned char paraId = 0; paraId < noParaSQ; paraId += 3) {
    dim_shared1[locId_local] = localHessian[paraId + 0];
    dim_shared2[locId_local] = localHessian[paraId + 1];
    dim_shared3[locId_local] = localHessian[paraId + 2];
    __syncthreads();

    if (locId_local < 128) {
      dim_shared1[locId_local] += dim_shared1[locId_local + 128];
      dim_shared2[locId_local] += dim_shared2[locId_local + 128];
      dim_shared3[locId_local] += dim_shared3[locId_local + 128];
    }
    __syncthreads();
    if (locId_local < 64) {
      dim_shared1[locId_local] += dim_shared1[locId_local + 64];
      dim_shared2[locId_local] += dim_shared2[locId_local + 64];
      dim_shared3[locId_local] += dim_shared3[locId_local + 64];
    }
    __syncthreads();

    if (locId_local < 32) {
      warpReduce(dim_shared1, locId_local);
      warpReduce(dim_shared2, locId_local);
      warpReduce(dim_shared3, locId_local);
    }
    __syncthreads();

    if (locId_local == 0) {
      atomicAdd(&(accu->h[paraId + 0]), dim_shared1[0]);
      atomicAdd(&(accu->h[paraId + 1]), dim_shared2[0]);
      atomicAdd(&(accu->h[paraId + 2]), dim_shared3[0]);
    }
  }
}

/*只是做了个等价调用，就不注释了*/
template <bool shortIteration, bool rotationOnly>
__global__ void depthTrackerOneLevel_g_rt_device(ITMDepthTracker_KernelParameters para) {
  depthTrackerOneLevel_g_rt_device_main<shortIteration, rotationOnly>(
      para.accu, para.depth, para.approxInvPose, para.pointsMap, para.normalsMap, para.sceneIntrinsics,
      para.sceneImageSize, para.scenePose, para.viewIntrinsics, para.viewImageSize, para.distThresh);
}
