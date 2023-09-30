// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"

/**
 * 更新单个voxel的depth
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in, out] voxel 要更新的voxel
 * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
 * @param[in] M_d  当前帧中深度图的位姿（word to local）
 * @param[in] projParams_d 深度图的相机内参，fx fy cx cy
 * @param[in] mu TSDF的截断值对应的距离，单位米
 * @param[in] maxW voxel的最大观测次数
 * @param[in] depth 深度图
 * @param[in] imgSize 当前帧深度图的分辨率
 * @return depth图中对应的深度值 与 voxel深度 的差。>0 说明voxel在depth图前面，<0 在后面（即voxel被depth图遮挡）
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline float
computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                             const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d, float mu,
                             int maxW, const CONSTPTR(float) * depth, const CONSTPTR(Vector2i) & imgSize) {
  //! 将voxel投影到成像平面 project point into image
  Vector4f pt_camera = M_d * pt_model;
  if (pt_camera.z <= 0) return -1;

  Vector2f pt_image;  // 成像平面上的二维坐标
  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;   // 相机投影公式
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || 
      (pt_image.y < 1) || (pt_image.y > imgSize.y - 2))       // 超出图像边缘的不要
    return -1;

  //! 根据深度差 判断是否要更新voxel。check whether voxel needs updating  
  float depth_measure = depth[(int) (pt_image.x + 0.5f) +   // 获取深度图对应的深度值。先转一维坐标。+0.5是为了四舍五入
                              (int) (pt_image.y + 0.5f) * imgSize.x];   // get measured depth from image
  if (depth_measure <= 0.0f) return -1;     // 无效退出

  float eta = depth_measure - pt_camera.z;
  if (eta < -mu) return eta;              // <，说明voxel被depth遮挡  // TODO: 为啥不是> 或者取绝对值？？？？

  //! 更新 compute updated SDF value and reliability 
  float oldF = TVoxel::valueToFloat(voxel.sdf);   // 取出voxel的sdf和权重（即观测次数）
  int oldW = voxel.w_depth;
  float newF = MIN(1.0f, eta / mu);               // 计算深度图中depth值的sdf（上限为1），权重置为1
  int newW = 1;
  // 加权取平均
  newF = oldW * oldF + newW * newF;               
  newW = oldW + newW;
  newF /= newW;
  newW = MIN(newW, maxW);
  // 记录到voxel里。write back
  voxel.sdf = TVoxel::floatToValue(newF);         
  voxel.w_depth = newW;

  return eta;
}

/**
 * 更新单个voxel的depth和置信度
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in, out] voxel 要更新的voxel
 * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）  // TODO：是voxel的中心坐标吗？
 * @param[in] M_d  当前帧中深度图的位姿（word to local）
 * @param[in] projParams_d 深度图的相机内参，fx fy cx cy
 * @param[in] mu TSDF的截断值对应的距离，单位米
 * @param[in] maxW voxel的最大观测次数
 * @param[in] depth 深度图
 * @param[in] confidence 深度图的置信度（根据距离计算）
 * @param[in] imgSize 当前帧深度图的分辨率
 * @return depth图中对应的深度值 与 voxel深度 的差。>0 说明voxel在depth图前面，<0 在后面（即voxel被depth图遮挡）
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline float
computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                             const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d, float mu,
                             int maxW, const CONSTPTR(float) * depth, const CONSTPTR(float) * confidence,
                             const CONSTPTR(Vector2i) & imgSize) {
  //! 将voxel投影到成像平面 project point into image
  Vector4f pt_camera = M_d * pt_model;
  if (pt_camera.z <= 0) return -1;

  Vector2f pt_image;    // 成像平面上的二维坐标
  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;   // 相机投影公式
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || 
      (pt_image.y < 1) || (pt_image.y > imgSize.y - 2))     // 超出图像边缘的不要
    return -1;
  
  //! 根据深度差 判断是否要更新voxel。  check whether voxel needs updating
  int locId = (int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x;  // 转一维坐标。+0.5是为了四舍五入
  float depth_measure = depth[locId];     // 获取输入图像对应位置的深度值
  if (depth_measure <= 0.0) return -1;    // 无效退出
   
  float eta = depth_measure - pt_camera.z;
  if (eta < -mu) return eta;              // <，说明voxel被depth遮挡  // TODO: 为啥不是> 或者取绝对值？？？？

  //! 更新 compute updated SDF value and reliability 
  float oldF = TVoxel::valueToFloat(voxel.sdf);   // 取出voxel的sdf和权重（即观测次数）
  int oldW = voxel.w_depth;
  float newF = MIN(1.0f, eta / mu);               // 计算深度图中depth值的sdf（上限为1），权重置为1
  int newW = 1;
  // 加权取平均
  newF = oldW * oldF + newW * newF;               
  newW = oldW + newW;
  newF /= newW;
  newW = MIN(newW, maxW);
  // 记录到voxel里。write back
  voxel.sdf = TVoxel::floatToValue(newF);         
  voxel.w_depth = newW;
  voxel.confidence += TVoxel::floatToValue(confidence[locId]);  // TODO：置信度跟权重有啥区别？？？

  return eta;
}

/**
 * 更新单个voxel的RGB
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in, out] voxel 要更新的voxel
 * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
 * @param[in] M_rgb 当前帧中RGB图的位姿（word to local）
 * @param[in] projParams_rgb RGB图的相机内参，fx fy cx cy
 * @param[in] mu TSDF的截断值对应的距离，单位米
 * @param[in] maxW voxel的最大观测次数
 * @param eta 【没用到】depth图中对应的深度值 与 voxel深度 的差
 * @param[in] rgb RGB图
 * @param[in] imgSize 当前帧深度图的分辨率
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline void
computeUpdatedVoxelColorInfo(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                             const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb, float mu,
                             uchar maxW, float eta, const CONSTPTR(Vector4u) * rgb,
                             const CONSTPTR(Vector2i) & imgSize) {
  //! 将voxel投影到成像平面
  Vector4f pt_camera = M_rgb * pt_model;  // 不判断z<=0 ????

  Vector2f pt_image;    // 成像平面上的二维坐标
  pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;   // 相机投影公式
  pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || 
      (pt_image.y < 1) || (pt_image.y > imgSize.y - 2))       // 超出图像边缘的不要
    return;

  //! 更新
  Vector3u buffV3u = voxel.clr;                   // TODO: 不要这个中间变量
  Vector3f oldC = TO_FLOAT3(buffV3u) / 255.0f;    // 取出voxel的RGB和权重（即观测次数）
  float oldW = (float) voxel.w_color;   

  Vector3f rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;  // 对RGB图插值，权重置为1
  //rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
  float newW = 1;
  // 加权取平均
  Vector3f newC = oldC * oldW + rgb_measure * newW;   // TODO：将rgb_measure换成newC，节省一个中间变量!
  newW = oldW + newW;
  newC /= newW;
  newW = MIN(newW, maxW);
  // 记录到voxel里
  voxel.clr = TO_UCHAR3(newC * 255.0f);
  voxel.w_color = (uchar) newW;
}

/**
 * @brief 根据voxel是否存储RGB或置信度（四种情况），对voxel进行更新
 * 
 * @tparam hasColor 是否存储RGB。是C++的非类型模板参数
 * @tparam hasConfidence 是否存储置信度。是C++的非类型模板参数
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 */
template<bool hasColor, bool hasConfidence, class TVoxel>
struct ComputeUpdatedVoxelInfo; 

// 上面ComputeUpdatedVoxelInfo的偏特化，无 RGB和置信度 信息 // TODO: 这里叫偏特化吗？
template <class TVoxel> 
struct ComputeUpdatedVoxelInfo<false, false, TVoxel> { 
  /**
   * 根据当前输入，更新单个voxel的depth
   * @param[in, out] voxel 要更新的voxel
   * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
   * @param[in] M_d  当前帧中深度图的位姿（word to local）
   * @param[in] projParams_d 深度图的相机内参
   * @param M_rgb 【没用到】 当前帧中RGB图的位姿（word to local）
   * @param projParams_rgb 【没用到】 RGB图的相机内参
   * @param[in] mu TSDF的截断值对应的距离，单位米
   * @param[in] maxW voxel的最大观测次数
   * @param[in] depth 深度图
   * @param confidence 【没用到】深度图的置信度（根据距离计算）
   * @param[in] imgSize_d 当前帧深度图的分辨率
   * @param rgb 【没用到】彩色图
   * @param imgSize_rgb 【没用到】当前帧RGB图的分辨率
   */
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                                         const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
                                         const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
                                         float mu, int maxW, const CONSTPTR(float) * depth,
                                         const CONSTPTR(float) * confidence, const CONSTPTR(Vector2i) & imgSize_d,
                                         const CONSTPTR(Vector4u) * rgb, const CONSTPTR(Vector2i) & imgSize_rgb) {
    computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
  }
};

// 上面ComputeUpdatedVoxelInfo的偏特化，有RGB信息、无置信度信息   // TODO: 这里叫偏特化吗？
template <class TVoxel> struct ComputeUpdatedVoxelInfo<true, false, TVoxel> {
  /**
   * 根据当前输入，更新单个voxel的depth和RGB
   * @param[in, out] voxel 要更新的voxel
   * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
   * @param[in] M_d  当前帧中深度图的位姿（word to local）
   * @param[in] projParams_d 深度图的相机内参
   * @param[in] M_rgb 当前帧中RGB图的位姿（word to local）
   * @param[in] projParams_rgb RGB图的相机内参
   * @param[in] mu TSDF的截断值对应的距离，单位米
   * @param[in] maxW voxel的最大观测次数
   * @param[in] depth 深度图
   * @param confidence 【没用到】深度图的置信度（根据距离计算）
   * @param[in] imgSize_d 当前帧深度图的分辨率
   * @param[in] rgb 彩色图
   * @param[in] imgSize_rgb 当前帧RGB图的分辨率
   */
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                                         const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
                                         const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
                                         float mu, int maxW, const CONSTPTR(float) * depth,
                                         const CONSTPTR(float) * confidence, const CONSTPTR(Vector2i) & imgSize_d,
                                         const CONSTPTR(Vector4u) * rgb, const THREADPTR(Vector2i) & imgSize_rgb) {
    // 更新depth
    float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
    if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
    // 更新RGB
    computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
  }
};

// 上面ComputeUpdatedVoxelInfo的偏特化，有置信度信息、无RGB信息   // TODO: 这里叫偏特化吗？
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, TVoxel> {
  /**
   * 根据当前输入，更新单个voxel的depth和置信度
   * @param[in, out] voxel 要更新的voxel
   * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
   * @param[in] M_d  当前帧中深度图的位姿（word to local）
   * @param[in] projParams_d 深度图的相机内参
   * @param M_rgb 【没用到】 当前帧中RGB图的位姿（word to local）
   * @param projParams_rgb 【没用到】 RGB图的相机内参
   * @param[in] mu TSDF的截断值对应的距离，单位米
   * @param[in] maxW voxel的最大观测次数
   * @param[in] depth 深度图
   * @param[in] confidence 深度图的置信度（根据距离计算）
   * @param[in] imgSize_d 当前帧深度图的分辨率
   * @param rgb 【没用到】彩色图
   * @param imgSize_rgb 【没用到】当前帧RGB图的分辨率
   */
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                                         const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,
                                         const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,
                                         float mu, int maxW, const CONSTPTR(float) * depth,
                                         const CONSTPTR(float) * confidence, const CONSTPTR(Vector2i) & imgSize_d,
                                         const CONSTPTR(Vector4u) * rgb, const CONSTPTR(Vector2i) & imgSize_rgb) {
    computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
  }
};

// 上面ComputeUpdatedVoxelInfo的偏特化，有置信度和RGB信息   // TODO: 这里叫偏特化吗？
template <class TVoxel> struct ComputeUpdatedVoxelInfo<true, true, TVoxel> {
  /**
   * 根据当前输入，更新单个voxel的depth、RGB和置信度
   * @param[in, out] voxel 要更新的voxel
   * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
   * @param[in] M_d  当前帧中深度图的位姿（word to local）
   * @param[in] projParams_d 深度图的相机内参
   * @param[in] M_rgb 当前帧中RGB图的位姿（word to local）
   * @param[in] projParams_rgb RGB图的相机内参
   * @param[in] mu TSDF的截断值对应的距离，单位米
   * @param[in] maxW voxel的最大观测次数
   * @param[in] depth 深度图
   * @param[in] confidence 深度图的置信度（根据距离计算）
   * @param[in] imgSize_d 当前帧深度图的分辨率
   * @param[in] rgb 彩色图
   * @param[in] imgSize_rgb 当前帧RGB图的分辨率
   */
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                                         const THREADPTR(Matrix4f) & M_d, const THREADPTR(Vector4f) & projParams_d,
                                         const THREADPTR(Matrix4f) & M_rgb, const THREADPTR(Vector4f) & projParams_rgb,
                                         float mu, int maxW, const CONSTPTR(float) * depth,
                                         const CONSTPTR(float) * confidence, const CONSTPTR(Vector2i) & imgSize_d,
                                         const CONSTPTR(Vector4u) * rgb, const THREADPTR(Vector2i) & imgSize_rgb) {
    // 更新depth和置信度
    float eta =
        computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
    if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
    // 更新RGB
    computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
  }
};

/**
 * 寻找单个像素对应三维点附近所有block，查看alloction情况和可见类型
 * @param[out] entriesAllocType entry存放位置。=1为order entry，=2为excess list
 * @param[out] entriesVisibleType 所有entry的可见类型。=1是正常可见，=2是可见但是被swap out
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[out] blockCoords 没有分配的entry会记录block坐标（short类型）
 * @param[in] depth 深度图
 * @param[in] invM_d 深度图位姿的逆（local to world？？？）
 * @param[in] projParams_d 深度图相机内参的反投影
 * @param[in] mu TSDF的截断值对应的距离。默认为0.02=4*voxel_size
 * @param[in] imgSize 图像分辨率（x*y)
 * @param[in] oneOverVoxelSize block实际边长的倒数。应该叫BlockSize
 * @param[in] hashTable
 * @param[in] viewFrustum_min 视锥中最小深度
 * @param[in] viewFrustum_max 视锥中最大深度
 * @note 单帧内出现哈希冲突咋办？？？
 */
_CPU_AND_GPU_CODE_ inline void
buildHashAllocAndVisibleTypePP(DEVICEPTR(uchar) * entriesAllocType, DEVICEPTR(uchar) * entriesVisibleType, int x, int y,
                               DEVICEPTR(Vector4s) * blockCoords, const CONSTPTR(float) * depth, Matrix4f invM_d,
                               Vector4f projParams_d, float mu, Vector2i imgSize, float oneOverVoxelSize,
                               const CONSTPTR(ITMHashEntry) * hashTable, float viewFrustum_min, float viewFrustum_max) {
  //! 深度图中的像素 反投影 成三维点
  float depth_measure = depth[x + y * imgSize.x];
  if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min
      || (depth_measure + mu) > viewFrustum_max)  // TODO: viewFrustum_min肯定比0大吧？？？
    return;

  Vector4f pt_camera_f;
  pt_camera_f.z = depth_measure;
  pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);
  //! 确定搜索范围：找到三维点前后距离mu的block坐标（朝向光心是前面）
  float norm = sqrt(pt_camera_f.x * pt_camera_f.x +                     // 光心到当前三维点的距离
                    pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);
  Vector4f pt_buff = pt_camera_f * (1.0f - mu / norm);      // 前面的block坐标（相机坐标系）。公式为相似三角△
  pt_buff.w = 1.0f; 
  Vector3f point = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;     // 先转到世界坐标系下，然后转成block坐标
  pt_buff = pt_camera_f * (1.0f + mu / norm);               // 后面的block坐标（相机坐标系）
  pt_buff.w = 1.0f;
  Vector3f point_e = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize; 
  //! 计算搜索步长        // TODO: 值得深入研究，保证前进的时候不会漏掉block、也不会在一个block中反复搜索
  Vector3f direction = point_e - point;     // 搜索步长
  norm = sqrt(direction.x * direction.x + direction.y * direction.y + 
              direction.z * direction.z);   // TODO: 不用再算了，norm=2*mu/BlockSize。这样计算还会导致浮点数误差呢！！！
  int noSteps = (int) ceil(2.0f * norm);    // TODO：有风险，如果mu*4<=BlockSize，noSteps=1，下面会除以0！！！
  direction /= (float) (noSteps - 1);       
  //! 寻找前后范围mu之内的所有block，查看alloction和可见类型 // add neighbouring blocks
  // NOTE: 下面的涉及的所有block都是可见的，不同的是可见类型
  unsigned int hashIdx;
  Vector3s blockPos;
  for (int i = 0; i < noSteps; i++) {
    // 获取当前block的hash id和entry
    blockPos = TO_SHORT_FLOOR3(point);
    hashIdx = hashIndex(blockPos);  //compute index in hash table 
    ITMHashEntry hashEntry = hashTable[hashIdx];

    // 查找对应block   //check if hash table contains entry
    bool isFound = false;
    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {  // 判断IS_EQUAL3是因为存在哈希冲突，>=-1说明已经分配过
      // 记录entry对应block的可见类型，=1是正常可见，=2是可见但是被swap out
      // entry has been streamed out but is visible or in memory and visible
      entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1; 
      isFound = true;
    }
    // 上面没找到可能是因为哈希冲突，继续遍历entry对应的bucket（其实InfiniTAM的bucket size=1）
    if (!isFound) {   
      bool isExcess = false;  // 是否存在于excess list（也叫unordered entries）
      if (hashEntry.ptr >= -1) { // >= -1说明已经被分配空间了。seach excess list only if there is no room in ordered part
        isExcess = true;
        while (hashEntry.offset >= 1) { // offset>= 1说明存在哈希冲突，要根据offset到unorder list(也叫excess list)中找
          hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;  // -1是因为offset在记录的时候+1
          hashEntry = hashTable[hashIdx];
          if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
            // 记录entry对应block的可见类型，=1是正常可见，=2是可见但是被swap out
            // entry has been streamed out but is visible or in memory and visible
            entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;  // hashEntry.ptr == -1表示被swapped out
            isFound = true;   // TODO: 单帧内出现哈希冲突咋办？？？ 后面来的hashIdx会把前面的覆盖掉的
            break;
          }
        }
      }
      // 遍历完bucket后还是没找到，说明这个block还没分配内存
      if (!isFound) {   
        entriesAllocType[hashIdx] = isExcess ? 2 : 1;   // 记录存放位置。=1存放于order list，=2存放于excess(unorder) list
        if (!isExcess) // entriesVisibleType只记录order entry的可见类型。因为其长度为bucket num。new entry is visible
          entriesVisibleType[hashIdx] = 1; 
        blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1); // 记录block位置，等后面再真正分配内存
        // TODO: blockCoords弄成stack<pair<hashIdx, blockCoord>>更好一些？这样之后不用再次遍历所有entry
        // TODO: cuda中可以使用前缀和，以及将不同hashIdx分开存
      }
    }

    point += direction;   // 前进四！！！
  }
}

/**
 * 查看三维点（真实坐标）在图像上是否可见
 * @details 将三维点投影到成像平面，在图像范围内就可见
 * @tparam useSwapping 是否支持交换内存-显存数据。C++的非类型模板参数https://blog.csdn.net/lanchunhui/article/details/49634077
 * @param[out] isVisible 在imgSize内是否可见
 * @param[out] isVisibleEnlarged 在扩大1/8后的imgSize内否可见
 * @param[in] pt_image 三维点
 * @param[in] M_d world to local的变换矩阵
 * @param[in] projParams_d 投影的相机内参
 * @param[in] imgSize 图像大小
 */
template <bool useSwapping>
_CPU_AND_GPU_CODE_ inline void
checkPointVisibility(THREADPTR(bool) & isVisible, THREADPTR(bool) & isVisibleEnlarged,
                     const THREADPTR(Vector4f) & pt_image, const CONSTPTR(Matrix4f) & M_d,
                     const CONSTPTR(Vector4f) & projParams_d, const CONSTPTR(Vector2i) & imgSize) {
  //! 投影到二维图像上
  Vector4f pt_buff = M_d * pt_image;  // 要先转到local坐标系下
  if (pt_buff.z < 1e-10f) return;

  pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
  pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;
  //! 查看是否可见
  if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && 
      pt_buff.y >= 0 && pt_buff.y < imgSize.y) {  // 投影在图像范围之内是否可见
    isVisible = true;
    isVisibleEnlarged = true;
  } else if (useSwapping) {                       // 不可见 && swap了，就看更大的图像范围（扩大1/8倍）是否可见 // TODO：why???
    Vector4i lims;
    lims.x = -imgSize.x / 8;    // 范围扩大1/8倍
    lims.y = imgSize.x + imgSize.x / 8;
    lims.z = -imgSize.y / 8;
    lims.w = imgSize.y + imgSize.y / 8;
    if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w)
      isVisibleEnlarged = true;
  }
}

/**
 * 检查单个block是否可见
 * @details 将block的8个顶点一次投影到成像平面，只要有一个顶点投影在图片范围内，就可见
 * @tparam useSwapping 是否支持交换内存-显存数据。C++的非类型模板参数https://blog.csdn.net/lanchunhui/article/details/49634077
 * @param[out] isVisible 在imgSize内是否可见
 * @param[out] isVisibleEnlarged 在扩大1/8后的imgSize内否可见
 * @param[in] hashPos block坐标（不是真实坐标）
 * @param[in] M_d world to local的变换矩阵
 * @param[in] projParams_d 投影的相机内参
 * @param[in] voxelSize voxel的实际尺寸。单位米
 * @param[in] imgSize 图像大小
 */
template <bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkBlockVisibility(THREADPTR(bool) & isVisible, THREADPTR(bool) & isVisibleEnlarged,
                                                    const THREADPTR(Vector3s) & hashPos, const CONSTPTR(Matrix4f) & M_d,
                                                    const CONSTPTR(Vector4f) & projParams_d,
                                                    const CONSTPTR(float) & voxelSize,
                                                    const CONSTPTR(Vector2i) & imgSize) {
  float factor = (float) SDF_BLOCK_SIZE * voxelSize;  // 单个block的实际尺寸。单位米

  isVisible = false;
  isVisibleEnlarged = false;

  //! 检查block的8个顶点的可见性
  Vector4f pt_image;      // 顶点坐标(0,0,0)
  pt_image.x = (float) hashPos.x * factor;
  pt_image.y = (float) hashPos.y * factor;
  pt_image.z = (float) hashPos.z * factor;
  pt_image.w = 1.0f;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;  // 只要有一个顶点可见，就算可见

  pt_image.z += factor;   // 顶点坐标(0,0,1)
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  pt_image.y += factor;   // 顶点坐标(0,1,1)
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  pt_image.x += factor;   // 顶点坐标(1,1,1)
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  pt_image.z -= factor;   // 顶点坐标以此类推
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  pt_image.y -= factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  pt_image.x -= factor;
  pt_image.y += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  pt_image.x += factor;
  pt_image.y -= factor;
  pt_image.z += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;
}
