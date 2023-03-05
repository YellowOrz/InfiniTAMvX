// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel,
                                                             const THREADPTR(Vector4f) &pt_model,
                                                             const CONSTPTR(Matrix4f) &M_d,
                                                             const CONSTPTR(Vector4f) &projParams_d,
                                                             float mu,
                                                             int maxW,
                                                             const CONSTPTR(float) *depth,
                                                             const CONSTPTR(Vector2i) &imgSize) {
  Vector4f pt_camera;
  Vector2f pt_image;
  float depth_measure, eta, oldF, newF;
  int oldW, newW;
  // 类似于kinectfusion
  // project point into image
  pt_camera = M_d * pt_model; //（4*4）*（4*1）=4*1
  if (pt_camera.z <= 0) return -1; //pt_camera 相机的位姿
  // 投影  //使用已知的旋转矩阵和平移向量将voxel坐标转化为深度相机坐标系的坐标,若体素块不可见则退出
  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z; //TODO（h）：数学公式的意义
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;


  // get measured depth from image
  depth_measure = depth[(int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x];
  if (depth_measure <= 0.0f) return -1;  //判断深度信息是否正确 否则退出函数

  // check whether voxel needs updating
  //如果体素靠近或在观测表面的前面，则将相应的观测值加到累积和中
  eta = depth_measure - pt_camera.z;
  if (eta < -mu) return eta;

  // compute updated SDF value and reliability
  oldF = TVoxel::valueToFloat(voxel.sdf);//上一帧体素块的sdf值
  oldW = voxel.w_depth;//上一帧体素块的权重

  newF = MIN(1.0f, eta / mu);//当前帧体素块的sdf值
  newW = 1;//当前帧体素块的权重

  newF = oldW * oldF + newW * newF;
  newW = oldW + newW;
  newF /= newW;
  newW = MIN(newW, maxW);

  // write back
  voxel.sdf = TVoxel::floatToValue(newF);//计算坐标理论值和实际测量值的差值，若大于mu，则更新
  voxel.w_depth = newW;//更新体素块的权重

  return eta;
}

template<class TVoxel>
/**
 *
 * @tparam TVoxel
 * @param voxel 体素块？
 * @param pt_model
 * @param M_d 当前深度图像位姿
 * @param projParams_d 深度相机的内在参数中的  校准矩阵 4*1
 * @param mu
 * @param maxW  voxel的最大观测次数，用来融合
 * @param depth  深度图像
 * @param confidence 当前图像置信度
 * @param imgSize  当前图像的大小 像素
 * @return
 */
 // Integration 融合 将新的深度图融合到三维模型里面
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(DEVICEPTR(TVoxel) &voxel,
                                                             const THREADPTR(Vector4f) &pt_model,
                                                             const CONSTPTR(Matrix4f) &M_d,
                                                             const CONSTPTR(Vector4f) &projParams_d,
                                                             float mu,
                                                             int maxW,
                                                             const CONSTPTR(float) *depth,
                                                             const CONSTPTR(float) *confidence,
                                                             const CONSTPTR(Vector2i) &imgSize) {
  Vector4f pt_camera;
  Vector2f pt_image;
  float depth_measure, eta, oldF, newF;
  int oldW, newW, locId;

  // project point into image 投影点成图像
  pt_camera = M_d * pt_model;
  if (pt_camera.z <= 0) return -1;

  //使用已知的旋转矩阵和平移向量将voxel坐标转化为深度相机坐标系的坐标,若体素块不可见则退出
  pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
  pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
  if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

  locId = (int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x;
  // get measured depth from image  获得深度信息
  depth_measure = depth[locId];
  if (depth_measure <= 0.0) return -1;

  // check whether voxel needs updating   在表面背后的voxel不需要更新 或者不在视野里的
  eta = depth_measure - pt_camera.z;
  if (eta < -mu) return eta;

  // compute updated SDF value and reliability   进行数据更新
  oldF = TVoxel::valueToFloat(voxel.sdf);
  oldW = voxel.w_depth;
  newF = MIN(1.0f, eta / mu);
  newW = 1;

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

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(DEVICEPTR(TVoxel) &voxel,
                                                            const THREADPTR(Vector4f) &pt_model,
                                                            const CONSTPTR(Matrix4f) &M_rgb,
                                                            const CONSTPTR(Vector4f) &projParams_rgb,
                                                            float mu,
                                                            uchar maxW,
                                                            float eta,
                                                            const CONSTPTR(Vector4u) *rgb,
                                                            const CONSTPTR(Vector2i) &imgSize) {
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
struct ComputeUpdatedVoxelInfo;

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) &voxel,
                                         const THREADPTR(Vector4f) &pt_model,
                                         const CONSTPTR(Matrix4f) &M_d,
                                         const CONSTPTR(Vector4f) &projParams_d,
                                         const CONSTPTR(Matrix4f) &M_rgb,
                                         const CONSTPTR(Vector4f) &projParams_rgb,
                                         float mu,
                                         int maxW,
                                         const CONSTPTR(float) *depth,
                                         const CONSTPTR(float) *confidence,
                                         const CONSTPTR(Vector2i) &imgSize_d,
                                         const CONSTPTR(Vector4u) *rgb,
                                         const CONSTPTR(Vector2i) &imgSize_rgb) {
    computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
  }
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) &voxel,
                                         const THREADPTR(Vector4f) &pt_model,
                                         const THREADPTR(Matrix4f) &M_d,
                                         const THREADPTR(Vector4f) &projParams_d,
                                         const THREADPTR(Matrix4f) &M_rgb,
                                         const THREADPTR(Vector4f) &projParams_rgb,
                                         float mu,
                                         int maxW,
                                         const CONSTPTR(float) *depth,
                                         const CONSTPTR(float) *confidence,
                                         const CONSTPTR(Vector2i) &imgSize_d,
                                         const CONSTPTR(Vector4u) *rgb,
                                         const THREADPTR(Vector2i) &imgSize_rgb) {
    float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
    if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
    computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
  }
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) &voxel,
                                         const THREADPTR(Vector4f) &pt_model,
                                         const CONSTPTR(Matrix4f) &M_d,
                                         const CONSTPTR(Vector4f) &projParams_d,
                                         const CONSTPTR(Matrix4f) &M_rgb,
                                         const CONSTPTR(Vector4f) &projParams_rgb,
                                         float mu,
                                         int maxW,
                                         const CONSTPTR(float) *depth,
                                         const CONSTPTR(float) *confidence,
                                         const CONSTPTR(Vector2i) &imgSize_d,
                                         const CONSTPTR(Vector4u) *rgb,
                                         const CONSTPTR(Vector2i) &imgSize_rgb) {
    computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
  }
};

template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, TVoxel> {
  _CPU_AND_GPU_CODE_ static void compute(DEVICEPTR(TVoxel) &voxel,
                                         const THREADPTR(Vector4f) &pt_model,
                                         const THREADPTR(Matrix4f) &M_d,
                                         const THREADPTR(Vector4f) &projParams_d,
                                         const THREADPTR(Matrix4f) &M_rgb,
                                         const THREADPTR(Vector4f) &projParams_rgb,
                                         float mu,
                                         int maxW,
                                         const CONSTPTR(float) *depth,
                                         const CONSTPTR(float) *confidence,
                                         const CONSTPTR(Vector2i) &imgSize_d,
                                         const CONSTPTR(Vector4u) *rgb,
                                         const THREADPTR(Vector2i) &imgSize_rgb) {
    float
        eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
    if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;
    computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
  }
};

_CPU_AND_GPU_CODE_ inline void buildHashAllocAndVisibleTypePP(DEVICEPTR(uchar) *entriesAllocType,
                                                              DEVICEPTR(uchar) *entriesVisibleType,
                                                              int x,
                                                              int y,
                                                              DEVICEPTR(Vector4s) *blockCoords,
                                                              const CONSTPTR(float) *depth,
                                                              Matrix4f invM_d,
                                                              Vector4f projParams_d,
                                                              float mu,
                                                              Vector2i imgSize,
                                                              float oneOverVoxelSize,
                                                              const CONSTPTR(ITMHashEntry) *hashTable,
                                                              float viewFrustum_min,
                                                              float viewFrustum_max) {
  float depth_measure;
  unsigned int hashIdx;
  int noSteps;
  Vector4f pt_camera_f;
  Vector3f point_e, point, direction;
  Vector3s blockPos;

  depth_measure = depth[x + y * imgSize.x];
  if (depth_measure <= 0 || (depth_measure - mu) < 0 || (depth_measure - mu) < viewFrustum_min
      || (depth_measure + mu) > viewFrustum_max)
    return;

  //图像坐标中的深度
  pt_camera_f.z = depth_measure;
  pt_camera_f.x = pt_camera_f.z * ((float(x) - projParams_d.z) * projParams_d.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) - projParams_d.w) * projParams_d.y);

  float norm = sqrt(pt_camera_f.x * pt_camera_f.x + pt_camera_f.y * pt_camera_f.y + pt_camera_f.z * pt_camera_f.z);

  Vector4f pt_buff;

  pt_buff = pt_camera_f * (1.0f - mu / norm);
  pt_buff.w = 1.0f;
  point = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

  pt_buff = pt_camera_f * (1.0f + mu / norm);
  pt_buff.w = 1.0f;
  point_e = TO_VECTOR3(invM_d * pt_buff) * oneOverVoxelSize;

  direction = point_e - point;
  norm = sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
  noSteps = (int) ceil(2.0f * norm);

  direction /= (float) (noSteps - 1);

  //add neighbouring blocks
  for (int i = 0; i < noSteps; i++) {
    blockPos = TO_SHORT_FLOOR3(point);//在此线段上获取voxel对应的voxel block坐标

    //compute index in hash table
    hashIdx = hashIndex(blockPos);

    //check if hash table contains entry
    bool isFound = false;

    ITMHashEntry hashEntry = hashTable[hashIdx];

    //检查该voxel block是否已经allocation
    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
      //entry has been streamed out but is visible or in memory and visible
      entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

      isFound = true;
    }

    if (!isFound) {
      bool isExcess = false;
      if (hashEntry.ptr >= -1) //seach excess list only if there is no room in ordered part
      {
        while (hashEntry.offset >= 1) {
          hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
          hashEntry = hashTable[hashIdx];

          if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
            //entry has been streamed out but is visible or in memory and visible
            entriesVisibleType[hashIdx] = (hashEntry.ptr == -1) ? 2 : 1;

            isFound = true;
            break;
          }
        }

        isExcess = true;
      }

      if (!isFound) //still not found
      {
        entriesAllocType[hashIdx] = isExcess ? 2 : 1; //needs allocation
        if (!isExcess) entriesVisibleType[hashIdx] = 1; //new entry is visible

        //没有allocation，在哈希表上进行标记，之后遍历哈希表进行更新
        blockCoords[hashIdx] = Vector4s(blockPos.x, blockPos.y, blockPos.z, 1);
      }
    }

    point += direction;
  }
}

template<bool useSwapping>//TODO:这里不太明白useSwapping的true或false对下面的函数起什么作用
_CPU_AND_GPU_CODE_ inline void checkPointVisibility(THREADPTR(bool) &isVisible,
                                                    THREADPTR(bool) &isVisibleEnlarged,
                                                    const THREADPTR(Vector4f) &pt_image,
                                                    const CONSTPTR(Matrix4f) &M_d,
                                                    const CONSTPTR(Vector4f) &projParams_d,
                                                    const CONSTPTR(Vector2i) &imgSize) {
  Vector4f pt_buff;

  pt_buff = M_d * pt_image;

  if (pt_buff.z < 1e-10f) return;

  pt_buff.x = projParams_d.x * pt_buff.x / pt_buff.z + projParams_d.z;
  pt_buff.y = projParams_d.y * pt_buff.y / pt_buff.z + projParams_d.w;

  if (pt_buff.x >= 0 && pt_buff.x < imgSize.x && pt_buff.y >= 0 && pt_buff.y < imgSize.y) {//若点在图像范围之内，可见也放大可见
    isVisible = true;
    isVisibleEnlarged = true;
  } else if (useSwapping) {
    Vector4i lims;//否则对图像范围进行一定转换
    lims.x = -imgSize.x / 8;
    lims.y = imgSize.x + imgSize.x / 8;
    lims.z = -imgSize.y / 8;
    lims.w = imgSize.y + imgSize.y / 8;

    //若点在转换后的图像范围内，点放大可见
    if (pt_buff.x >= lims.x && pt_buff.x < lims.y && pt_buff.y >= lims.z && pt_buff.y < lims.w)
      isVisibleEnlarged = true;
  }
}

template<bool useSwapping>
_CPU_AND_GPU_CODE_ inline void checkBlockVisibility(THREADPTR(bool) &isVisible,
                                                    THREADPTR(bool) &isVisibleEnlarged,
                                                    const THREADPTR(Vector3s) &hashPos,
                                                    const CONSTPTR(Matrix4f) &M_d,
                                                    const CONSTPTR(Vector4f) &projParams_d,
                                                    const CONSTPTR(float) &voxelSize,
                                                    const CONSTPTR(Vector2i) &imgSize) {
  Vector4f pt_image;
  float factor = (float) SDF_BLOCK_SIZE * voxelSize;

  isVisible = false;
  isVisibleEnlarged = false;

  // 0 0 0 初始图像范围内块是否可见
  pt_image.x = (float) hashPos.x * factor;
  pt_image.y = (float) hashPos.y * factor;
  pt_image.z = (float) hashPos.z * factor;
  pt_image.w = 1.0f;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 0 0 1 z方向扩大后块是否可见
  pt_image.z += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 0 1 1 y,z方向扩大后块是否可见
  pt_image.y += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 1 1 1 x,y,z方向扩大后块是否可见
  pt_image.x += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 1 1 0 x,y方向扩大后块是否可见
  pt_image.z -= factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 1 0 0 x方向扩大后块是否可见
  pt_image.y -= factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 0 1 0 y方向扩大后块是否可见
  pt_image.x -= factor;
  pt_image.y += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;

  // 1 0 1 x,z方向扩大后块是否可见
  pt_image.x += factor;
  pt_image.y -= factor;
  pt_image.z += factor;
  checkPointVisibility<useSwapping>(isVisible, isVisibleEnlarged, pt_image, M_d, projParams_d, imgSize);
  if (isVisible) return;
}
