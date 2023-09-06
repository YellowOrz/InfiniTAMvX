// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"

/**
 * 根据深度图，更新单个voxel的depth   // TODO: 下次从这儿开始
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in, out] voxel 要更新的voxel
 * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
 * @param[in] M_d  当前帧中深度图的位姿（word to local）
 * @param[in] projParams_d 深度图的相机内参
 * @param[in] mu TSDF的截断值对应的距离，单位米
 * @param[in] maxW voxel的最大观测次数
 * @param[in] depth 深度图
 * @param[in] imgSize 当前帧深度图的分辨率
 * @return 
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline float
computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                             const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d, float mu,
                             int maxW, const CONSTPTR(float) * depth, const CONSTPTR(Vector2i) & imgSize) {
  pt_camera;
  float depth_measure, eta, oldF, newF;
  int oldW, newW;
  // 类似于kinectfusion
  // project point into image
  Vector4f pt_camera = M_d * pt_model; //（4*4）*（4*1）=4*1
  if (pt_camera.z <= 0) return -1; //pt_camera 相机的位姿
  // 投影  //使用已知的旋转矩阵和平移向量将voxel坐标转化为深度相机坐标系的坐标,若体素块不可见则退出
  Vector2f pt_image;
  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z; //TODO（h）：数学公式的意义
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

  // get measured depth from image
  float depth_measure = depth[(int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x];
  if (depth_measure <= 0.0f) return -1;  //判断深度信息是否正确 否则退出函数

  // check whether voxel needs updating
  //如果体素靠近或在观测表面的前面，则将相应的观测值加到累积和中
  float eta = depth_measure - pt_camera.z;
  if (eta < -mu) return eta;

  // compute updated SDF value and reliability
  float oldF = TVoxel::valueToFloat(voxel.sdf);//上一帧体素块的sdf值
  int oldW = voxel.w_depth;//上一帧体素块的权重

  float newF = MIN(1.0f, eta / mu);//当前帧体素块的sdf值
  int newW = 1;//当前帧体素块的权重

  newF = oldW * oldF + newW * newF;
  newW = oldW + newW;
  newF /= newW;
  newW = MIN(newW, maxW);

  // write back
  voxel.sdf = TVoxel::floatToValue(newF);//计算坐标理论值和实际测量值的差值，若大于mu，则更新
  voxel.w_depth = newW;//更新体素块的权重

  return eta;
}

/**
 * 根据深度图和置信度图，更新单个voxel的depth和置信度 // TODO: 下次从这儿开始
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @param[in, out] voxel 要更新的voxel
 * @param[in] pt_model 当前voxel在全局下的真实坐标（单位米）
 * @param[in] M_d  当前帧中深度图的位姿（word to local）
 * @param[in] projParams_d 深度图的相机内参
 * @param[in] mu TSDF的截断值对应的距离，单位米
 * @param[in] maxW voxel的最大观测次数
 * @param[in] depth 深度图
 * @param[in] confidence 深度图的置信度（根据距离计算）
 * @param[in] imgSize 当前帧深度图的分辨率
 * @return 
 */
template <class TVoxel>
_CPU_AND_GPU_CODE_ inline float
computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                             const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d, float mu,
                             int maxW, const CONSTPTR(float) * depth, const CONSTPTR(float) * confidence,
                             const CONSTPTR(Vector2i) & imgSize) {
  // project point into image 投影点成图像
  Vector4f pt_camera = M_d * pt_model;
  if (pt_camera.z <= 0) return -1;

  //使用已知的旋转矩阵和平移向量将voxel坐标转化为深度相机坐标系的坐标,若体素块不可见则退出
  Vector2f pt_image;
  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

  int locId = (int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x;
  // get measured depth from image  获得深度信息
  float depth_measure = depth[locId];
  if (depth_measure <= 0.0) return -1;

  // check whether voxel needs updating   在表面背后的voxel不需要更新 或者不在视野里的
  float eta = depth_measure - pt_camera.z;
  if (eta < -mu) return eta;

  // compute updated SDF value and reliability   进行数据更新
  float oldF = TVoxel::valueToFloat(voxel.sdf);
  int oldW = voxel.w_depth;
  float newF = MIN(1.0f, eta / mu);
  int newW = 1;

  newF = oldW * oldF + newW * newF;
  newW = oldW + newW;
  newF /= newW;
  newW = MIN(newW, maxW);

  // write back^
  voxel.sdf = TVoxel::floatToValue(newF);
  voxel.w_depth = newW;
  voxel.confidence += TVoxel::floatToValue(confidence[locId]);//更新体素块的置信度

  return eta;
}

template <class TVoxel>
_CPU_AND_GPU_CODE_ inline void  // TODO: 下次从这儿开始
computeUpdatedVoxelColorInfo(DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,
                             const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb, float mu,
                             uchar maxW, float eta, const CONSTPTR(Vector4u) * rgb,
                             const CONSTPTR(Vector2i) & imgSize) {
  Vector4f pt_camera;
  Vector2f pt_image;
  Vector3f rgb_measure, oldC, newC;
  Vector3u buffV3u;
  float newW, oldW;

  buffV3u = voxel.clr;
  oldW = (float) voxel.w_color;//上一帧体素块的权重

  oldC = TO_FLOAT3(buffV3u) / 255.0f;//上一帧体素块的权重
  newC = oldC;

  //将点投影到图像中
  pt_camera = M_rgb * pt_model;

  //使用已知的旋转矩阵和平移向量将voxel坐标转化为深度相机坐标系的坐标
  pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
  pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

  //剔除不在图像范围内的点
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

  rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
  //rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
  newW = 1;

  newC = oldC * oldW + rgb_measure * newW;//当前帧体素块的rgb信息
  newW = oldW + newW;//当前帧体素块的权重
  newC /= newW;
  newW = MIN(newW, maxW);

  //更新体素块的rgb信息和权重
  voxel.clr = TO_UCHAR3(newC * 255.0f);
  voxel.w_color = (uchar) newW;
}

//根据体素是否存储颜色信息或置信信息，分四种情况对体素进行更新
template<bool hasColor, bool hasConfidence, class TVoxel>
struct ComputeUpdatedVoxelInfo;   // TODO: 为啥专门搞个结构体，在调用的地方判断一下if else不就好了吗？？？？

// 上面ComputeUpdatedVoxelInfo的偏特化，无 RGB和置信度 信息
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

// 上面ComputeUpdatedVoxelInfo的偏特化，有RGB信息、无置信度信息
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

// 上面ComputeUpdatedVoxelInfo的偏特化，有置信度信息、无RGB信息
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

// 上面ComputeUpdatedVoxelInfo的偏特化，有置信度和RGB信息
template <class TVoxel> struct ComputeUpdatedVoxelInfo<true, true, TVoxel> {
  /**
   * 根据当前输入，更新单个voxel的depth、RGB和置信度  // TODO: 下次从这儿开始
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
 * 寻找单个像素对应三维点附近所有block，查看alloction和可见情况
 * @param[out] entriesAllocType 没有分配的entry会记录存放位置。
 *                              =1为orderneeds allocation，=2为excess list
 * @param[out] entriesVisibleType 所有entry的可见情况。=1是正常可见，=2是应该可见但是被删除了（swapped out）
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[out] blockCoords 没有分配的entry会记录block坐标（short类型）
 * @param[in] depth 深度图
 * @param[in] invM_d 深度图位姿的逆（local to world？？？）
 * @param[in] projParams_d 深度图相机内参的反投影
 * @param[in] mu TSDF的截断值对应的距离
 * @param[in] imgSize 图像分辨率（x*y)
 * @param[in] oneOverVoxelSize block实际边长的倒数。不应该叫VoxelSize
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
  //! 找到三维点前后距离mu的block（朝向光心是前面）
  float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

  Vector4f pt_buff = pt_camera_f * (1.0f - mu / norm);
  pt_buff.w = 1.0f;
  Vector3f point = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;   // 前面的block坐标。先转到世界坐标系下，然后转成block坐标
  pt_buff = pt_camera_f * (1.0f + mu / norm);
  pt_buff.w = 1.0f;
  Vector3f point_e = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize; // 后面的block坐标
  Vector3f direction = point_e - point;
  norm = sqrt(direction.x * direction.x + direction.y * direction.y + 
              direction.z * direction.z);   // 若mu=4*voxel_size（即默认设置），则direction长度为一个block（即norm=1）
  int noSteps = (int) ceil(2.0f * norm);    // ？？？步长为半个mu？？？那为啥下面要减1？
  direction /= (float) (noSteps - 1);                                 // 单位方向向量

  //! 寻找前后范围mu之内的所有block，查看alloction和可见情况 // add neighbouring blocks
  unsigned int hashIdx;
  Vector3s blockPos;
  for (int i = 0; i < noSteps; i++) {
    // 获取当前block的hash id和entry
    blockPos = TO_SHORT_FLOOR3(point);
    hashIdx = hashIndex(blockPos);  //compute index in hash table 
    ITMHashEntry hashEntry = hashTable[hashIdx];

    //当前block是否已经allocation   //check if hash table contains entry
    bool isFound = false;
    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {  // 判断IS_EQUAL3是因为存在哈希冲突
      // 记录entry的可见情况，=1是正常可见，=2是应该可见但是被删除了（swapped out）
      //entry has been streamed out but is visible or in memory and visible
      entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;  // ???
      isFound = true;
    }
    // 上面没找到可能是因为哈希冲突，继续遍历entry对应的bucket（其实InfiniTAM的bucket size=1）
    if (!isFound) {   
      bool isExcess = false;  // 是否存在于excess list（也叫unordered entries）
      if (hashEntry.ptr >= -1) { // >= -1说明已经被分配空间了。seach excess list only if there is no room in ordered part
        while (hashEntry.offset >= 1) { // >= 1说明存在哈希冲突
          hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
          hashEntry = hashTable[hashIdx];
          if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
            // 记录entry的可见情况，=1是正常可见，=2是应该可见但是被删除了（swapped out）
            //entry has been streamed out but is visible or in memory and visible
            entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;  // hashEntry.ptr == -1表示被swapped out
            isFound = true;   // TODO: 单帧内出现哈希冲突咋办？？？ 后面来的hashIdx会把前面的覆盖掉的
            break;
          }
        }
        isExcess = true;
      }
      // 遍历完bucket后还是没找到，说明这个block就没有被allocate，需要被allocate
      if (!isFound) {   
        entriesAllocType[hashIdx] = isExcess ? 2 : 1;   // 记录存放位置。=1存放于orderneeds allocation，=2存放于excess list
        if (!isExcess) // 只对order entry标记可见（虽然没分配空间）。why？？？。new entry is visible
          entriesVisibleType[hashIdx] = 1; 
        blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1); // 记录block位置
      }
    }

    point += direction; // 下一个
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
  } else if (useSwapping) {                       // 不可见 && swap了，就看更大的图像范围（扩大1/8倍）之内是否可见 why???
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
