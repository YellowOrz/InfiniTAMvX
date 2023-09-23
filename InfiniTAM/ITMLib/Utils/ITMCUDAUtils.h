// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

template<class T>
inline __device__ void warpReduce(volatile T *sdata, int tid) {
  sdata[tid] += sdata[tid + 32];
  sdata[tid] += sdata[tid + 16];
  sdata[tid] += sdata[tid + 8];
  sdata[tid] += sdata[tid + 4];
  sdata[tid] += sdata[tid + 2];
  sdata[tid] += sdata[tid + 1];
}

inline __device__ void warpReduce3(volatile float *sdata, int tid) {
  sdata[3 * tid + 0] += sdata[3 * (tid + 32) + 0];
  sdata[3 * tid + 1] += sdata[3 * (tid + 32) + 1];
  sdata[3 * tid + 2] += sdata[3 * (tid + 32) + 2];
  sdata[3 * tid + 0] += sdata[3 * (tid + 16) + 0];
  sdata[3 * tid + 1] += sdata[3 * (tid + 16) + 1];
  sdata[3 * tid + 2] += sdata[3 * (tid + 16) + 2];
  sdata[3 * tid + 0] += sdata[3 * (tid + 8) + 0];
  sdata[3 * tid + 1] += sdata[3 * (tid + 8) + 1];
  sdata[3 * tid + 2] += sdata[3 * (tid + 8) + 2];
  sdata[3 * tid + 0] += sdata[3 * (tid + 4) + 0];
  sdata[3 * tid + 1] += sdata[3 * (tid + 4) + 1];
  sdata[3 * tid + 2] += sdata[3 * (tid + 4) + 2];
  sdata[3 * tid + 0] += sdata[3 * (tid + 2) + 0];
  sdata[3 * tid + 1] += sdata[3 * (tid + 2) + 1];
  sdata[3 * tid + 2] += sdata[3 * (tid + 2) + 2];
  sdata[3 * tid + 0] += sdata[3 * (tid + 1) + 0];
  sdata[3 * tid + 1] += sdata[3 * (tid + 1) + 1];
  sdata[3 * tid + 2] += sdata[3 * (tid + 1) + 2];
}
/**
 * 通过计算数组的前缀和 来找到数组中想要的每个元素 在最终存放的数组中的位置
 * @details 前缀和的计算方法：分治。
 *          步骤一：归约求和。跟普通的归约求和相反（数据往前加），将数据往后加。刚开始分组小（2个1组），计算每组之和存在最后；
 *                 分组长度*2，重复计算
 *          步骤二：更新组内中间数。从最大的分组开始，每组中间 加上前一组的最后一个数，即为它在整个数组中的前缀和。
 *                 分组长度/2，重复计算。等长度为2的组 也计算完，整个数组的前缀和就完成了
 *          但由于CUDA是要分grid和block。所以下面的代码先在block内部（使用共享内存）计算步骤一、二（称为block内的offset），然后计算
 *                block之间的和，
 *          推荐参考“说明材料/CUDA计算前缀和.pptx”和“说明材料/CUDA计算前缀和.cpp”
 *          这里不是单纯的前缀和计算，在前缀和的后面还加入了条件判断，从而专门用于计算所需元素的存放位置
 * @tparam T 数组中数字的类型
 * @param[in] element 数组中的某个数字。值为0（表示不要） or 1（表示想要）。    // TODO: 应该不会是其他取值了吧
 * @param[in,out] sum 数组中最大的前缀和，即想要的元素个数
 * @param[in] localSize cuda的block大小。必须为256，∵在共享内存中申请了长256的数组
 * @param[in] localId block中的线程id
 * @return 当前元素 在最终存放的数组中的位置。-1表示这个元素不需要
 */
template<typename T>
__device__ int computePrefixSum_device(uint element, T *sum, int localSize, int localId) {
  // TODO: should be localSize...
  __shared__ uint prefixBuffer[16 * 16];  // block
  __shared__ uint groupOffset;    

  prefixBuffer[localId] = element;
  __syncthreads();

  int s1, s2;
  //! block内部 归约求和。s1是步长（以2的倍数增长），s2用来选择每组最后一个数（用来存放求和的值）。
  // NOTE：以下两个for循环可以运行“说明材料/CUDA计算前缀和.cpp”，通过输出进行理解
  for (s1 = 1, s2 = 1; s1 < localSize; s1 <<= 1) {    // NOTE: 可以参考“说明材料/CUDA计算前缀和.pptx”中第一页
    s2 |= s1;   // s2从左往右一位一位变成1。因为s2表示区间（长度为2^n）中的最后一个数，而且从0开始，所以s2=2^n-1，其二进制是连续多个1
    // NOTE: A & B = B 说明 B 中所有为1的位在 A 中也必须为1。这里是找到localId的二进制中从低位有连续几个1，这个for循环就遍历几遍
    // 例如，localId=71，二进制为1000111，最低位有3个1，则该for循环会遍历3次
    if ((localId & s2) == s2)   // 位置是s2
      prefixBuffer[localId] += prefixBuffer[localId - s1];
    __syncthreads();
  }
  /* 上面for循环改成这样更容易理解
  for (s1 = 1; s1 < localSize; s1 *= 2) {
    if((localId + 1) % (s1*2) == 0)   // +1是因为localId从0开始，s1*2是因为 分组长度 是 步长的2倍
      prefixBuffer[localId] += prefixBuffer[localId - s1];
    __syncthreads();
  }
  */
  //! block内部 更新组内最中间的数。步长从64开始。得到block内部的前缀和
  for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1) {   // NOTE: 可以参考“说明材料/CUDA计算前缀和.pptx”中第二页
    if (localId != localSize - 1 && (localId & s2) == s2) 
      prefixBuffer[localId + s1] += prefixBuffer[localId];  // 第一个循环只有prefixBuffer[192] += prefixBuffer[128]
    __syncthreads();
  }
  /* 上面for循环改成这样更容易理解
  for (s1 = localSize / 2; s1 >= 2; s1 /= 2) {
    if(localId != localSize - 1 && (localId + 1)%s1 == 0)   // +1是因为localId从0开始
      prefixBuffer[localId+s1/2] += prefixBuffer[localId];  // s1/2表示当前区间的中间，localId是上一个区间的最后
    __syncthreads();
  }
  */
  //! block之间 求和。得到整个数组（全局）的前缀和
  if (localId == 0 && prefixBuffer[localSize - 1] > 0) groupOffset = atomicAdd(sum, prefixBuffer[localSize - 1]);
  __syncthreads();
  //! 计算当前元素在整个数组（全局）的前缀和
  int offset;// = groupOffset + prefixBuffer[localId] - 1;
  if (localId == 0) {
    if (prefixBuffer[localId] == 0) offset = -1;                          // 前缀和为0，表示当前元素值为0，不想要
    else offset = groupOffset;
  } else {
    if (prefixBuffer[localId] == prefixBuffer[localId - 1]) offset = -1;  // 跟前一个前缀和相同，表示当前元素值为0，不想要
    else offset = groupOffset + prefixBuffer[localId - 1];                // 当前元素在整个数组（全局）的前缀和，想要
  }

  return offset;
}

__device__ static inline void atomicMin(float *address, float val) {
  int *address_as_i = (int *) address;
  int old = *address_as_i, assumed;
  do {
    assumed = old;
    old = ::atomicCAS(address_as_i, assumed,
                      __float_as_int(::fminf(val, __int_as_float(assumed))));
  } while (assumed != old);
}

__device__ static inline void atomicMax(float *address, float val) {
  int *address_as_i = (int *) address;
  int old = *address_as_i, assumed;
  do {
    assumed = old;
    old = ::atomicCAS(address_as_i, assumed,
                      __float_as_int(::fmaxf(val, __int_as_float(assumed))));
  } while (assumed != old);
}

template<typename T>
__global__ void memsetKernel_device(T *devPtr, const T val, size_t nwords) {
  size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
  if (offset >= nwords) return;
  devPtr[offset] = val;
}

template<typename T>
__global__ void memsetKernelLarge_device(T *devPtr, const T val, size_t nwords) {
  size_t offset = threadIdx.x + blockDim.x * (blockIdx.x + blockIdx.y * gridDim.x);
  if (offset >= nwords) return;
  devPtr[offset] = val;
}

template<typename T>
inline void memsetKernel(T *devPtr, const T val, size_t nwords) {
  dim3 blockSize(256);
  dim3 gridSize((int) ceil((float) nwords / (float) blockSize.x));
  if (gridSize.x <= 65535) {
    memsetKernel_device<T> <<<gridSize, blockSize>>>(devPtr, val, nwords);
    ORcudaKernelCheck;
  } else {
    gridSize.x = (int) ceil(sqrt((float) gridSize.x));
    gridSize.y = (int) ceil((float) nwords / (float) (blockSize.x * gridSize.x));
    memsetKernelLarge_device<T> <<<gridSize, blockSize>>>(devPtr, val, nwords);
    ORcudaKernelCheck;
  }
}

template<typename T>
__global__ void fillArrayKernel_device(T *devPtr, size_t nwords) {
  size_t offset = threadIdx.x + blockDim.x * blockIdx.x;
  if (offset >= nwords) return;
  devPtr[offset] = offset;
}

template<typename T>
inline void fillArrayKernel(T *devPtr, size_t nwords) {
  dim3 blockSize(256);
  dim3 gridSize((int) ceil((float) nwords / (float) blockSize.x));
  fillArrayKernel_device<T> <<<gridSize, blockSize>>>(devPtr, nwords);
  ORcudaKernelCheck;
}

