// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMVoxelBlockHash.h"

/**
 * 从三维block坐标 计算 hash id（hash table中的索引）
 * @tparam T block坐标的类型（感觉这么说不太准确？？？）
 * @param[in] blockPos 三维block坐标
 * @return hash id = hash value % bucket number
 */
template<typename T>
_CPU_AND_GPU_CODE_ inline int hashIndex(const THREADPTR(T) &blockPos) {
  return (((uint) blockPos.x * 73856093u) ^ ((uint) blockPos.y * 19349669u) ^ ((uint) blockPos.z * 83492791u))
      & (uint) SDF_HASH_MASK;
}

/**
 * 从三维voxel坐标 转 三维block坐标
 * @param[in] point 三维voxel坐标(int)
 * @param[out] blockPos 三维block坐标(int)
 * @return voxel在block内部坐标的一维坐标（或者叫下标）
 */
_CPU_AND_GPU_CODE_ inline int pointToVoxelBlockPos(const THREADPTR(Vector3i) &point, THREADPTR(Vector3i) &blockPos) {
  blockPos.x = ((point.x < 0) ? point.x - SDF_BLOCK_SIZE + 1 : point.x) / SDF_BLOCK_SIZE;
  blockPos.y = ((point.y < 0) ? point.y - SDF_BLOCK_SIZE + 1 : point.y) / SDF_BLOCK_SIZE;
  blockPos.z = ((point.z < 0) ? point.z - SDF_BLOCK_SIZE + 1 : point.z) / SDF_BLOCK_SIZE;

  // Vector3i locPos = point - blockPos * SDF_BLOCK_SIZE;
  // return locPos.x + locPos.y * SDF_BLOCK_SIZE + locPos.z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
  // voxel在block内部的坐标 = (px - px/bs*bs, py - py/bs*bs, pz - pz/bs*bs)，px/bs就是上面算出来的bx
  // 一维形式 = px' + py' * bs + pz' * bs * bs  // 为啥要写成下面？？？
  return point.x + (point.y - blockPos.x) * SDF_BLOCK_SIZE + (point.z - blockPos.y) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE -
         blockPos.z * SDF_BLOCK_SIZE3;
}
/**
 * 寻找某个voxel 在 voxel block array中的位置 && 更新cache
 * @param[in] voxelIndex  hash table
 * @param[in] point       要寻找的voxel坐标。注意是整数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @param[in, out] cache  上次找到的block坐标。
 * @return voxel 在 voxel block array中的位置
 * @note 有cache是因为 每次找block都要遍历bucket（∵哈希冲突），而快找到所需voxel时会反复找同一个block，索性记录一下
 */
_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) * voxelIndex,
                                        const THREADPTR(Vector3i) & point, THREADPTR(int) & vmIndex,
                                        THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache) {
  //! 从三维voxel坐标 转 三维block坐标
  Vector3i blockPos;
  short linearIdx = pointToVoxelBlockPos(point, blockPos);
  //! 先看看是不是刚刚找过的，是的话血赚
  if IS_EQUAL3(blockPos, cache.blockPos) {
    vmIndex = true;
    return cache.blockPtr + linearIdx;
  }
  //! 遍历哈希表进行检索
  int hashIdx = hashIndex(blockPos);

  while (true) {
    ITMHashEntry hashEntry = voxelIndex[hashIdx];

    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
      vmIndex = true;
      cache.blockPos = blockPos;
      cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
      return cache.blockPtr + linearIdx;
    }

    if (hashEntry.offset < 1) break;  // <1说明没有哈希冲突
    hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;  // -1是因为offset在记录的时候+1
  }

  vmIndex = false;
  return -1;
}
/**
 * 寻找某个voxel 在 voxel block array中的位置
 * @param[in] voxelIndex  hash table
 * @param[in] point       要寻找的voxel坐标。注意是整数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @return voxel 在 voxel block array中的位置
 */
_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) * voxelIndex,
                                        Vector3i point, THREADPTR(int) & vmIndex) {
  ITMLib::ITMVoxelBlockHash::IndexCache cache;
  return findVoxel(voxelIndex, point, vmIndex, cache);
}
/**
 * 寻找某个voxel 在 voxel block array中的位置
 * @param[in] voxelIndex  hash table
 * @param[in] point       要寻找的voxel坐标。注意是整数
 * @param[out] findVoxel  能否找到所需voxel
 * @return voxel 在 voxel block array中的位置
 */
_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) * voxelIndex,
                                        Vector3i point, THREADPTR(bool) & foundPoint) {
  int vmIndex;
  ITMLib::ITMVoxelBlockHash::IndexCache cache;
  int result = findVoxel(voxelIndex, point, vmIndex, cache);
  foundPoint = vmIndex != 0;
  return result;
}

/**
 * 根据 voxel坐标 找到VBA中对应voxel && 更新cache
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要寻找的voxel坐标。注意是整数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @param[in, out] cache  上次找到的block坐标。
 * @return 找到的voxel数据。如果找不到则为空的voxel
 * @note 有cache是因为 每次找block都要遍历bucket（∵哈希冲突），而快找到所需voxel时会反复找同一个block，索性记录一下
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) * voxelData,
                                           const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) * voxelIndex,
                                           const THREADPTR(Vector3i) & point, THREADPTR(int) & vmIndex,
                                           THREADPTR(ITMLib::ITMVoxelBlockHash::IndexCache) & cache) {
  //! 从voxel坐标转block坐标
  Vector3i blockPos;
  int linearIdx = pointToVoxelBlockPos(point, blockPos);  // linearIdx为voxel在block内部的一维坐标
  //! 刚好是上次访问的block（cache），直接读取就好
  if IS_EQUAL3(blockPos, cache.blockPos) {
    vmIndex = true;
    return voxelData[cache.blockPtr + linearIdx];
  }
  //! 否则遍历bucket，找到真正的block
  int hashIdx = hashIndex(blockPos);
  while (true) {
    ITMHashEntry hashEntry = voxelIndex[hashIdx];

    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
      cache.blockPos = blockPos;
      cache.blockPtr = hashEntry.ptr * SDF_BLOCK_SIZE3;
      vmIndex = hashIdx + 1; // 添加 1 以支持 isFound 的旧版 true/false 操作。因为hashIdx从0开始？？？
                             // add 1 to support legacy true / false operations for isFound
      return voxelData[cache.blockPtr + linearIdx];
    }

    if (hashEntry.offset < 1) break;  // <1说明没有哈希冲突
    hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;  // -1是因为offset在记录的时候+1
  }
  //! 找不到，说明这个block里面还没数据，则返回空
  vmIndex = false;
  return TVoxel();
}
/**
 * 根据 voxel坐标 找到VBA中对应voxel
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要寻找的voxel坐标。注意是整数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @return 找到的voxel数据。如果找不到则为空的voxel
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) * voxelData,
                                           const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) * voxelIndex,
                                           Vector3i point, THREADPTR(int) & vmIndex) {
  ITMLib::ITMVoxelBlockHash::IndexCache cache;
  return readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
}
/**
 * 根据 voxel坐标 找到VBA中对应voxel
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要寻找的voxel坐标。注意是整数
 * @param[out] foundPoint 能否找到所需voxel
 * @return 找到的voxel数据。如果找不到则为空的voxel
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) * voxelData,
                                           const CONSTPTR(ITMLib::ITMVoxelBlockHash::IndexData) * voxelIndex,
                                           Vector3i point, THREADPTR(bool) & foundPoint) {
  int vmIndex;
  ITMLib::ITMVoxelBlockHash::IndexCache cache;
  TVoxel result = readVoxel(voxelData, voxelIndex, point, vmIndex, cache);
  foundPoint = vmIndex != 0;
  return result;
}

/**
 * 读取某个voxel坐标（取整）的TSDF值（没有插值）
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要读取TSDF值的voxel坐标。注意是小数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @return                voxel坐标的TSDF值
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTPTR(TVoxel) * voxelData,
                                                                 const CONSTPTR(TIndex) * voxelIndex, Vector3f point,
                                                                 THREADPTR(int) & vmIndex) {
  TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)),
                         vmIndex);
  return TVoxel::valueToFloat(res.sdf);
}

/**
 * 读取某个voxel坐标（取整）的TSDF值（没有插值） && 更新cache
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam TCache 存放block信息的临时数据结构。用于voxel hashing，而ITMPlainVoxelArray中也有但为空
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要读取TSDF值的voxel坐标。注意是小数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @param[in, out] cache  之前找到的block的三维坐标和hash id
 * @return                voxel坐标的TSDF值
 * @note 有cache是因为每次找block都要遍历bucket（∵哈希冲突），而快找到所需voxel时会反复找同一个block，索性记录一下
 */
template <class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const CONSTPTR(TVoxel) * voxelData,
                                                                 const CONSTPTR(TIndex) * voxelIndex, Vector3f point,
                                                                 THREADPTR(int) & vmIndex, THREADPTR(TCache) & cache) {
  TVoxel res = readVoxel(voxelData, voxelIndex, Vector3i((int)ROUND(point.x), (int)ROUND(point.y), (int)ROUND(point.z)),
                         vmIndex, cache);
  return TVoxel::valueToFloat(res.sdf);
}

/**
 * 通过邻域插值的方式 获取某个voxel坐标（带小数）的TSDF值 && 更新cache
 * @note 以当前voxel坐标（小数）相邻的8个voxel进行三线性插值
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam TCache 存放block信息的临时数据结构。用于voxel hashing，而ITMPlainVoxelArray中也有但为空
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要读取TSDF值的voxel坐标。注意是小数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @param[in] cache       之前找到的block的三维坐标和hash id
 * @return                voxel坐标的TSDF值
 * @note 有cache是因为每次找block都要遍历bucket（∵哈希冲突），而快找到所需voxel时会反复找同一个block，索性记录一下。
 *       然后因为这个是在找到等值面附近才调用，所以readVoxel里面不会更新cache
 */
template <class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const CONSTPTR(TVoxel) * voxelData,
                                                               const CONSTPTR(TIndex) * voxelIndex, Vector3f point,
                                                               THREADPTR(int) & vmIndex, THREADPTR(TCache) & cache) {
  float res1, res2, v1, v2;
  Vector3f coeff;
  Vector3i pos;
  TO_INT_FLOOR3(pos, coeff, point);
  //! 三线性插值
  // NOTE: 推荐参考资料https://zhuanlan.zhihu.com/p/77496615，这里是用双线性插值拼凑出来的，没有参考资料里面的优雅！
  v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache).sdf;
  v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache).sdf;
  res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

  v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache).sdf;
  v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache).sdf;
  res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

  v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache).sdf;
  v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache).sdf;
  res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

  v1 = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache).sdf;
  v2 = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache).sdf;
  res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

  vmIndex = true;
  return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}

/**
 * 通过邻域插值的方式 获取某个voxel坐标（带小数）的TSDF值和置信度 && 更新cache
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam TCache 存放block信息的临时数据结构。用于voxel hashing，而ITMPlainVoxelArray中也有但为空
 * @param[out] confidence voxel坐标的置信度（即观测次数）。同样通过插值得到
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要读取TSDF值的voxel坐标。注意是小数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @param[in] cache       之前找到的block的三维坐标和hash id
 * @return                voxel坐标的TSDF值
 * @note 有cache是因为每次找block都要遍历bucket（∵哈希冲突），而快找到所需voxel时会反复找同一个block，索性记录一下。
 *       然后因为这个是在找到等值面附近才调用，所以readVoxel里面不会更新cache
 */
template <class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float
readWithConfidenceFromSDF_float_interpolated(THREADPTR(float) & confidence, const CONSTPTR(TVoxel) * voxelData,
                                             const CONSTPTR(TIndex) * voxelIndex, Vector3f point,
                                             THREADPTR(int) & vmIndex, THREADPTR(TCache) & cache) {
  float res1, res2, v1, v2;
  float res1_c, res2_c, v1_c, v2_c;
  TVoxel voxel;

  Vector3f coeff;
  Vector3i pos;
  TO_INT_FLOOR3(pos, coeff, point);
  //! 三线性插值
  // NOTE: 推荐参考资料https://zhuanlan.zhihu.com/p/77496615，这里是用双线性插值拼凑出来的，没有参考资料里面的优雅！
  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
  v1 = voxel.sdf;
  v1_c = voxel.w_depth;
  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
  v2 = voxel.sdf;
  v2_c = voxel.w_depth;
  res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;
  res1_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
  v1 = voxel.sdf;
  v1_c = voxel.w_depth;
  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
  v2 = voxel.sdf;
  v2_c = voxel.w_depth;
  res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
  res1_c = (1.0f - coeff.y) * res1_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
  v1 = voxel.sdf;
  v1_c = voxel.w_depth;
  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
  v2 = voxel.sdf;
  v2_c = voxel.w_depth;
  res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;
  res2_c = (1.0f - coeff.x) * v1_c + coeff.x * v2_c;

  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
  v1 = voxel.sdf;
  v1_c = voxel.w_depth;
  voxel = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
  v2 = voxel.sdf;
  v2_c = voxel.w_depth;
  res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);
  res2_c = (1.0f - coeff.y) * res2_c + coeff.y * ((1.0f - coeff.x) * v1_c + coeff.x * v2_c);

  vmIndex = true;

  confidence = (1.0f - coeff.z) * res1_c + coeff.z * res2_c;

  return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}
/**
 * 通过邻域插值的方式 获取某个voxel坐标（带小数）的TSDF值和观测次数 && 更新cache
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam TCache 存放block信息的临时数据结构。用于voxel hashing，而ITMPlainVoxelArray中也有但为空
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       要读取TSDF值的voxel坐标。注意是小数
 * @param[out] vmIndex    voxel坐标所属block在hash table中的index，如果找不到则为0
 * @param[in] cache       之前找到的block的三维坐标和hash id
 * @param[out] maxW       voxel坐标的观测次数。取相邻voxel中最大的观测次数
 * @return                voxel坐标的TSDF值
 * @note 有cache是因为每次找block都要遍历bucket（∵哈希冲突），而快找到所需voxel时会反复找同一个block，索性记录一下。
 *       然后因为这个是在找到等值面附近才调用，所以readVoxel里面不会更新cache
 * @see 跟readWithConfidenceFromSDF_float_interpolated很像，只不过它的置信度（=观测次数）是插值来的，本函数是取max
 */
template <class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline float
readFromSDF_float_interpolated(const CONSTPTR(TVoxel) * voxelData, const CONSTPTR(TIndex) * voxelIndex, Vector3f point,
                               THREADPTR(int) & vmIndex, THREADPTR(TCache) & cache, int &maxW) {
  float res1, res2, v1, v2;
  Vector3f coeff;
  Vector3i pos;
  TO_INT_FLOOR3(pos, coeff, point);
  //! 取邻域最近8个voxel中置信度最大的
  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
    v1 = v.sdf;
    maxW = v.w_depth;
  }
  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
    v2 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  res1 = (1.0f - coeff.x) * v1 + coeff.x * v2;

  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
    v1 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
    v2 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  res1 = (1.0f - coeff.y) * res1 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
    v1 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
    v2 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  res2 = (1.0f - coeff.x) * v1 + coeff.x * v2;

  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
    v1 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  {
    const TVoxel &v = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
    v2 = v.sdf;
    if (v.w_depth > maxW) maxW = v.w_depth;
  }
  res2 = (1.0f - coeff.y) * res2 + coeff.y * ((1.0f - coeff.x) * v1 + coeff.x * v2);

  vmIndex = true;
  return TVoxel::valueToFloat((1.0f - coeff.z) * res1 + coeff.z * res2);
}
/**
 * 通过邻域插值的方式 获取某个voxel坐标（带小数）的RGB && 更新cache
 * @note 以当前voxel坐标（小数）相邻的8个voxel进行三线性插值
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam TCache 存放block信息的临时数据结构。用于voxel hashing，而ITMPlainVoxelArray中也有但为空
 * @param[in] voxelData voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point 要读取TSDF值的voxel坐标
 * @param[in] cache 之前找到的block的三维坐标和hash id
 * @return voxel坐标的RGB
 */
template <class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f
readFromSDF_color4u_interpolated(const CONSTPTR(TVoxel) * voxelData, const CONSTPTR(TIndex) * voxelIndex,
                                 const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache) {
  TVoxel resn;
  Vector3f ret(0.0f);
  Vector4f ret4;
  int vmIndex;  // voxel坐标所属block在hash table中的index，如果找不到则为0。理论上应该return，但是这里没用到
  Vector3f coeff;
  Vector3i pos;
  TO_INT_FLOOR3(pos, coeff, point); // 将小数的整数位和小数位分开
  //! 三线性插值
  // NOTE: 推荐参考资料https://zhuanlan.zhihu.com/p/77496615
  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
  ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
  ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
  ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
  ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
  ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
  ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
  ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
  ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

  ret4.x = ret.x;
  ret4.y = ret.y;
  ret4.z = ret.z;
  ret4.w = 255.0f;

  return ret4 / 255.0f;
}
/**
 * 通过邻域插值的方式 获取某个voxel坐标（带小数）的 RGB和观测次数 && 更新cache
 * @note 以当前voxel坐标（小数）相邻的8个voxel进行三线性插值
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam TCache 存放block信息的临时数据结构。用于voxel hashing，而ITMPlainVoxelArray中也有但为空
 * @param[in] voxelData voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point 要读取TSDF值的voxel坐标
 * @param[in] cache 之前找到的block的三维坐标和hash id
 * @param[out] maxW 观测次数。取相邻voxel中最大的观测次数
 * @return voxel坐标的RGB
 */
template <class TVoxel, class TIndex, class TCache>
_CPU_AND_GPU_CODE_ inline Vector4f
readFromSDF_color4u_interpolated(const CONSTPTR(TVoxel) * voxelData, const CONSTPTR(TIndex) * voxelIndex,
                                 const THREADPTR(Vector3f) & point, THREADPTR(TCache) & cache, int &maxW) {
  TVoxel resn;
  Vector3f ret(0.0f);
  Vector4f ret4;
  int vmIndex;
  Vector3f coeff;
  Vector3i pos;
  TO_INT_FLOOR3(pos, coeff, point);
  //! 三线性插值
  // NOTE: 推荐参考资料https://zhuanlan.zhihu.com/p/77496615
  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex, cache);
  maxW = resn.w_depth;
  ret += (1.0f - coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (coeff.x) * (1.0f - coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (1.0f - coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (coeff.x) * (coeff.y) * (1.0f - coeff.z) * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (1.0f - coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (coeff.x) * (1.0f - coeff.y) * coeff.z * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (1.0f - coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

  resn = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex, cache);
  if (resn.w_depth > maxW) maxW = resn.w_depth;
  ret += (coeff.x) * (coeff.y) * coeff.z * resn.clr.toFloat();

  ret4.x = ret.x;
  ret4.y = ret.y;
  ret4.z = ret.z;
  ret4.w = 255.0f;

  return ret4 / 255.0f;
}

/**
 * 直接从TSDF中计算 单个voxel坐标（小数）的法向量
 * @details 使用三线性插值得到当前坐标在各个方向的梯度，作为法向量
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] voxelData   voxel block array
 * @param[in] voxelIndex  hash table
 * @param[in] point       voxel坐标。注意是小数
 * @return                voxel坐标的法向量
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline Vector3f computeSingleNormalFromSDF(const CONSTPTR(TVoxel) * voxelData,
                                                              const CONSTPTR(TIndex) * voxelIndex,
                                                              const THREADPTR(Vector3f) & point) {
  int vmIndex;

  Vector3f ret;
  Vector3f coeff;
  Vector3i pos;
  TO_INT_FLOOR3(pos, coeff, point);
  Vector3f ncoeff(1.0f - coeff.x, 1.0f - coeff.y, 1.0f - coeff.z);

  // all 8 values are going to be reused several times
  //! 取出邻域8个voxel的TSDF值
  Vector4f front, back;
  front.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 0), vmIndex).sdf;   // xOy平面上
  front.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 0), vmIndex).sdf;
  front.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 0), vmIndex).sdf;
  front.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 0), vmIndex).sdf;
  back.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 1), vmIndex).sdf;    // xOy平面向上平移1
  back.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 1), vmIndex).sdf;
  back.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 1), vmIndex).sdf;
  back.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 1), vmIndex).sdf;

  Vector4f tmp;
  float p1, p2, v1;
  
  // NOTE：下面的双线性插值公式见 https://en.wikipedia.org/wiki/Bilinear_interpolation 中的“On the unit square”
  //! 用平行yOz面、相邻着当前cube计算X方向的梯度 // gradient x  
  // 用紧挨着yOz面上的cube的TSDF三线性插值  // TODO: 有必要吗？？？直接用当前cube的对立两个面计算法向量就好了，反正只是用来看看
  p1 = front.x * ncoeff.y * ncoeff.z + front.z * coeff.y * ncoeff.z + back.x * ncoeff.y * coeff.z +
       back.z * coeff.y * coeff.z;                                    // 双线性插值。参考《从TSDF中计算法向量.pptx》中的第一页
  tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 0), vmIndex).sdf;  // yOz面向x轴负方向 平移
  tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 0), vmIndex).sdf;
  tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 0, 1), vmIndex).sdf;
  tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(-1, 1, 1), vmIndex).sdf;
  p2 = tmp.x * ncoeff.y * ncoeff.z + tmp.y * coeff.y * ncoeff.z + 
       tmp.z * ncoeff.y * coeff.z + tmp.w * coeff.y * coeff.z;          // 平移后再 双线性插值
  v1 = p1 * coeff.x + p2 * ncoeff.x;                                    // 用插值到的两个点再插值 // TODO: 为啥这里能用cx???
  // 用平行yOz的那个面上的四个点
  p1 = front.y * ncoeff.y * ncoeff.z + front.w * coeff.y * ncoeff.z + back.y * ncoeff.y * coeff.z +
       back.w * coeff.y * coeff.z;                                      // 同理
  tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 0), vmIndex).sdf;
  tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 0), vmIndex).sdf;
  tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 0, 1), vmIndex).sdf;
  tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(2, 1, 1), vmIndex).sdf;
  p2 =
      tmp.x * ncoeff.y * ncoeff.z + tmp.y * coeff.y * ncoeff.z + tmp.z * ncoeff.y * coeff.z + tmp.w * coeff.y * coeff.z;

  ret.x = TVoxel::valueToFloat(p1 * ncoeff.x + p2 * coeff.x - v1);  // 两个cube插值完后相减，就是梯度 // TODO: 为啥不是v1-v2?
  // NOTE: 梯度x的计算示意图可以见《从TSDF中计算法向量.ggb》。其中红色cube就是point相邻的点（即上面的front、back），蓝色cube就是插值得到v1的，绿色cube就是插值得到v2的

  //! 用平行xOz面、相邻着当前cube计算Y方向的梯度 // gradient y
  p1 = front.x * ncoeff.x * ncoeff.z + front.y * coeff.x * ncoeff.z + back.x * ncoeff.x * coeff.z +
       back.y * coeff.x * coeff.z;
  tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 0), vmIndex).sdf;
  tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 0), vmIndex).sdf;
  tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, -1, 1), vmIndex).sdf;
  tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, -1, 1), vmIndex).sdf;
  p2 =
      tmp.x * ncoeff.x * ncoeff.z + tmp.y * coeff.x * ncoeff.z + tmp.z * ncoeff.x * coeff.z + tmp.w * coeff.x * coeff.z;
  v1 = p1 * coeff.y + p2 * ncoeff.y;

  p1 = front.z * ncoeff.x * ncoeff.z + front.w * coeff.x * ncoeff.z + back.z * ncoeff.x * coeff.z +
       back.w * coeff.x * coeff.z;
  tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 0), vmIndex).sdf;
  tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 0), vmIndex).sdf;
  tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 2, 1), vmIndex).sdf;
  tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 2, 1), vmIndex).sdf;
  p2 =
      tmp.x * ncoeff.x * ncoeff.z + tmp.y * coeff.x * ncoeff.z + tmp.z * ncoeff.x * coeff.z + tmp.w * coeff.x * coeff.z;

  ret.y = TVoxel::valueToFloat(p1 * ncoeff.y + p2 * coeff.y - v1);

  //! 用平行xOy面、相邻着当前cube计算Z方向的梯度 // gradient z
  p1 = front.x * ncoeff.x * ncoeff.y + front.y * coeff.x * ncoeff.y + front.z * ncoeff.x * coeff.y +
       front.w * coeff.x * coeff.y;
  tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, -1), vmIndex).sdf;
  tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, -1), vmIndex).sdf;
  tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, -1), vmIndex).sdf;
  tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, -1), vmIndex).sdf;
  p2 =
      tmp.x * ncoeff.x * ncoeff.y + tmp.y * coeff.x * ncoeff.y + tmp.z * ncoeff.x * coeff.y + tmp.w * coeff.x * coeff.y;
  v1 = p1 * coeff.z + p2 * ncoeff.z;

  p1 = back.x * ncoeff.x * ncoeff.y + back.y * coeff.x * ncoeff.y + back.z * ncoeff.x * coeff.y +
       back.w * coeff.x * coeff.y;
  tmp.x = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 0, 2), vmIndex).sdf;
  tmp.y = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 0, 2), vmIndex).sdf;
  tmp.z = readVoxel(voxelData, voxelIndex, pos + Vector3i(0, 1, 2), vmIndex).sdf;
  tmp.w = readVoxel(voxelData, voxelIndex, pos + Vector3i(1, 1, 2), vmIndex).sdf;
  p2 =
      tmp.x * ncoeff.x * ncoeff.y + tmp.y * coeff.x * ncoeff.y + tmp.z * ncoeff.x * coeff.y + tmp.w * coeff.x * coeff.y;

  ret.z = TVoxel::valueToFloat(p1 * ncoeff.z + p2 * coeff.z - v1);

  return ret;   // 梯度算完就是法向量
}

template <bool hasColor, class TVoxel, class TIndex> struct VoxelColorReader;
/** 不带颜色的强行读取颜色都是0 */
template <class TVoxel, class TIndex> struct VoxelColorReader<false, TVoxel, TIndex> {// TODO: 为啥突然搞个结构体？？？
  _CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTPTR(TVoxel) * voxelData,
                                                 const CONSTPTR(typename TIndex::IndexData) * voxelIndex,
                                                 const THREADPTR(Vector3f) & point) {
    return Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
  }
};
/** 插值地读取voxel坐标（带小数）的RGB */
template <class TVoxel, class TIndex> struct VoxelColorReader<true, TVoxel, TIndex> {
  _CPU_AND_GPU_CODE_ static Vector4f interpolate(const CONSTPTR(TVoxel) * voxelData,
                                                 const CONSTPTR(typename TIndex::IndexData) * voxelIndex,
                                                 const THREADPTR(Vector3f) & point) {
    typename TIndex::IndexCache cache;
    return readFromSDF_color4u_interpolated(voxelData, voxelIndex, point, cache);
  }
};

#ifndef __METALC__

#include "ITMPlainVoxelArray.h"

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) * voxelIndex,
                                        const THREADPTR(Vector3i) & point_orig, THREADPTR(int) & vmIndex) {
  Vector3i point = point_orig - voxelIndex->offset;

  if ((point.x < 0) || (point.x >= voxelIndex->size.x) ||
      (point.y < 0) || (point.y >= voxelIndex->size.y) ||
      (point.z < 0) || (point.z >= voxelIndex->size.z)) {
    vmIndex = false;
    return -1;
  }

  int linearIdx = point.x + point.y * voxelIndex->size.x + point.z * voxelIndex->size.x * voxelIndex->size.y;

  vmIndex = true;
  return linearIdx;
}

_CPU_AND_GPU_CODE_ inline int findVoxel(const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) * voxelIndex,
                                        const THREADPTR(Vector3i) & point_orig, THREADPTR(int) & vmIndex,
                                        THREADPTR(ITMLib::ITMPlainVoxelArray::IndexCache) & cache) {
  return findVoxel(voxelIndex, point_orig, vmIndex);
}

template <class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) * voxelData,
                                           const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) * voxelIndex,
                                           const THREADPTR(Vector3i) & point_orig, THREADPTR(int) & vmIndex) {
  int voxelAddress = findVoxel(voxelIndex, point_orig, vmIndex);
  return vmIndex ? voxelData[voxelAddress] : TVoxel();
}

template <class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel readVoxel(const CONSTPTR(TVoxel) * voxelData,
                                           const CONSTPTR(ITMLib::ITMPlainVoxelArray::IndexData) * voxelIndex,
                                           const THREADPTR(Vector3i) & point_orig, THREADPTR(int) & vmIndex,
                                           THREADPTR(ITMLib::ITMPlainVoxelArray::IndexCache) & cache) {
  return readVoxel(voxelData, voxelIndex, point_orig, vmIndex);
}

/**
* \brief The specialisations of this struct template can be used to write/read colours to/from surfels.
*
* \tparam hasColour  Whether or not the surfel type can store colour information.
*/
template<bool hasColour>
struct SurfelColourManipulator;

/**
* \brief This template specialisation can be used to write/read dummy colours to/from surfels.
*
* It is intended for use with surfel types that cannot store colour information.
*/
template<>
struct SurfelColourManipulator<false> {
  /**
  * \brief Simulates the reading of a colour from the specified surfel.
  *
  * \param surfel  The surfel.
  * \return        A dummy colour (black).
  */
  template<typename TSurfel>
  _CPU_AND_GPU_CODE_
  static Vector3u read(const TSurfel &surfel) {
    return Vector3u((uchar) 0);
  }

  /**
  * \brief Simulates the writing of a colour to the specified surfel.
  *
  * In practice, this is just a no-op, since the surfel can't store a colour.
  *
  * \param surfel  The surfel.
  * \param colour  The colour.
  */
  template<typename TSurfel>
  _CPU_AND_GPU_CODE_
  static void write(TSurfel &surfel, const Vector3u &colour) {
    // No-op
  }
};

/**
* \brief This template specialisation can be used to write/read actual colours to/from surfels.
*
* It is intended for use with surfel types that can store colour information.
*/
template<>
struct SurfelColourManipulator<true> {
  /**
  * \brief Gets the colour of the specified surfel.
  *
  * \param surfel  The surfel.
  * \return        The surfel's colour.
  */
  template<typename TSurfel>
  _CPU_AND_GPU_CODE_
  static Vector3u read(const TSurfel &surfel) {
    return surfel.colour;
  }

  /**
  * \brief Sets the colour of the specified surfel.
  *
  * \param surfel  The surfel.
  * \param colour  The surfel's new colour.
  */
  template<typename TSurfel>
  _CPU_AND_GPU_CODE_
  static void write(TSurfel &surfel, const Vector3u &colour) {
    surfel.colour = colour;
  }
};

#endif
