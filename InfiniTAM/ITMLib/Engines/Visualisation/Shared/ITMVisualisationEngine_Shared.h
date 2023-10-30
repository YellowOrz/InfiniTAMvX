// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/ITMRepresentationAccess.h"

static const CONSTPTR(int) MAX_RENDERING_BLOCKS = 65536 * 4;  // 单次渲染的最大块数。每块的大小为下面的renderingBlockSizeX/Y
// static const int MAX_RENDERING_BLOCKS = 16384;
static const CONSTPTR(int) minmaximg_subsample = 8; // 对ray深度范围图的下采样倍数 // TODO:为了节省计算量???

#if !(defined __METALC__)
/** raycasting中用来确定搜索深度范围的小块 */
struct RenderingBlock {
  Vector2s upperLeft;   // 左上角在深度范围图上的像素坐标
  Vector2s lowerRight;  // 右下角在深度范围图上的像素坐标
  Vector2f zRange;      // 深度范围
};

#ifndef FAR_AWAY
#define FAR_AWAY 999999.9f  // ray的搜索距离的默认最大值
#endif

#ifndef VERY_CLOSE
#define VERY_CLOSE 0.05f    // ray的搜索距离的默认最小值
#endif

static const CONSTPTR(int) renderingBlockSizeX = 16;  // 渲染的时候的最小分块大小
static const CONSTPTR(int) renderingBlockSizeY = 16;

/** 
 * @brief 计算相机内参的逆。
 * @note fx、fy取倒数，cx、cy取负
*/
_CPU_AND_GPU_CODE_ inline Vector4f InvertProjectionParams(const THREADPTR(Vector4f) & projParams) {
  return Vector4f(1.0f / projParams.x, 1.0f / projParams.y, -projParams.z, -projParams.w);
}

/**
 * @brief 将单个可见的block投影到 当前视角下，并计算包围盒 && 深度范围
 * @param[in] blockPos      block在场景中的全局位姿（id）
 * @param[in] pose          当前视角的相机位姿
 * @param[in] intrinsics    相机内参
 * @param[in] imgSize       图像大小
 * @param[in] voxelSize     voxel的实际尺寸
 * @param[out] upperLeft    block在 深度范围图 投影的包围盒 左上角坐标
 * @param[out] lowerRight   block在 深度范围图 投影的包围盒 右下角坐标
 * @param[out] zRange       block在当前帧坐标系下的 深度范围
 * @return                  是否投影成功。深度值为非正数 && 投影不到图像上，都为false
 */
_CPU_AND_GPU_CODE_ inline bool
ProjectSingleBlock(const THREADPTR(Vector3s) & blockPos, const THREADPTR(Matrix4f) & pose,
                   const THREADPTR(Vector4f) & intrinsics, const THREADPTR(Vector2i) & imgSize, float voxelSize,
                   THREADPTR(Vector2i) & upperLeft, THREADPTR(Vector2i) & lowerRight, THREADPTR(Vector2f) & zRange) {
  upperLeft = imgSize / minmaximg_subsample;    // NOTE: 这里从渲染图片的坐标 转换到了 深度范围图的坐标
  lowerRight = Vector2i(-1, -1);
  zRange = Vector2f(FAR_AWAY, VERY_CLOSE);
  //! 依次将block的8个顶点投影到二维平面上
  for (int corner = 0; corner < 8; ++corner) {
    // project all 8 corners down to 2D image
    // 获取当前顶点的id坐标
    Vector3s tmp = blockPos;
    tmp.x += (corner & 1) ? 1 : 0;
    tmp.y += (corner & 2) ? 1 : 0;
    tmp.z += (corner & 4) ? 1 : 0;
    // 获取真实世界坐标 && 变换到相机坐标系下
    Vector4f pt3d(TO_FLOAT3(tmp) * (float)SDF_BLOCK_SIZE * voxelSize, 1.0f);
    pt3d = pose * pt3d;
    if (pt3d.z < 1e-6)
      continue;
    // 投影到二维平面上
    Vector2f pt2d;
    pt2d.x = (intrinsics.x * pt3d.x / pt3d.z + intrinsics.z) / minmaximg_subsample;
    pt2d.y = (intrinsics.y * pt3d.y / pt3d.z + intrinsics.w) / minmaximg_subsample;

    // 确定最大的包围盒。remember bounding box, zmin and zmax
    if (upperLeft.x > floor(pt2d.x))
      upperLeft.x = (int)floor(pt2d.x);
    if (lowerRight.x < ceil(pt2d.x))
      lowerRight.x = (int)ceil(pt2d.x);
    if (upperLeft.y > floor(pt2d.y))
      upperLeft.y = (int)floor(pt2d.y);
    if (lowerRight.y < ceil(pt2d.y))
      lowerRight.y = (int)ceil(pt2d.y);
    if (zRange.x > pt3d.z)
      zRange.x = pt3d.z;
    if (zRange.y < pt3d.z)
      zRange.y = pt3d.z;
  }

  // 防止越界。do some sanity checks and respect image bounds
  if (upperLeft.x < 0)
    upperLeft.x = 0;
  if (upperLeft.y < 0)
    upperLeft.y = 0;
  if (lowerRight.x >= imgSize.x)
    lowerRight.x = imgSize.x - 1;
  if (lowerRight.y >= imgSize.y)
    lowerRight.y = imgSize.y - 1;
  if (upperLeft.x > lowerRight.x)
    return false;
  if (upperLeft.y > lowerRight.y)
    return false;
  // if (zRange.y <= VERY_CLOSE) return false; never seems to happen
  if (zRange.x < VERY_CLOSE)
    zRange.x = VERY_CLOSE;
  if (zRange.y < VERY_CLOSE)
    return false;

  return true;
}

/**
 * @brief 将可见的block在当前视角下的二维投影分块。 为啥不放到ProjectSingleBlock？？？？
 * @param[in,out] renderingBlockList  raycast当前帧所有小块的数组首地址
 * @param[in] offset                  当前block的小块的起始id
 * @param[in] upperLeft               当前block 包围盒的左上角
 * @param[in] lowerRight              当前block 包围盒的右下角
 * @param[in] zRange                  当前block的深度范围
 */
_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks(DEVICEPTR(RenderingBlock) * renderingBlockList, int offset,
                                                     const THREADPTR(Vector2i) & upperLeft,
                                                     const THREADPTR(Vector2i) & lowerRight,
                                                     const THREADPTR(Vector2f) & zRange) {
  // split bounding box into 16x16 pixel rendering blocks
  //! 将包围盒分割成16x16的小块 为啥不把前面算好的块数传进来？？？
  for (int by = 0; by < ceil((float)(1 + lowerRight.y - upperLeft.y) / renderingBlockSizeY); ++by) {
    for (int bx = 0; bx < ceil((float)(1 + lowerRight.x - upperLeft.x) / renderingBlockSizeX); ++bx) {
      if (offset >= MAX_RENDERING_BLOCKS)
        return; // 没有必要吧？？？CreateExpectedDepths中已经保证超过范围的不进入这个函数了
      // for each rendering block: add it to the list
      DEVICEPTR(RenderingBlock) & b(renderingBlockList[offset++]);
      // 计算每个小块自己的包围盒
      b.upperLeft.x = upperLeft.x + bx * renderingBlockSizeX;
      b.upperLeft.y = upperLeft.y + by * renderingBlockSizeY;
      b.lowerRight.x = upperLeft.x + (bx + 1) * renderingBlockSizeX - 1;
      b.lowerRight.y = upperLeft.y + (by + 1) * renderingBlockSizeY - 1;
      if (b.lowerRight.x > lowerRight.x)
        b.lowerRight.x = lowerRight.x;
      if (b.lowerRight.y > lowerRight.y)
        b.lowerRight.y = lowerRight.y;
      // 深度值范围不变
      b.zRange = zRange;
    }
  }
}

#endif
/**
 * @brief 计算单个ray的投射结果
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @tparam modifyVisibleEntries 是否要顺带修改visile entry列表。为啥弄成模板参数、不弄成函数参数？？？
 * @param[out] pt_out 要计算的ray的交点。最后一位是权重
 * @param[out] entriesVisibleType  visible entry列表。如果modifyVisibleEntries为true，则顺带更新
 * @param[in] x raycasting得到的图像上的像素坐标
 * @param[in] y raycasting得到的图像上的像素坐标
 * @param[in] voxelData voxel block array
 * @param[in] voxelIndex hash table
 * @param[in] invM 当前视角的相机 到 世界坐标系 的变换矩阵？？？
 * @param[in] invProjParams 相机内参的逆。fx、fy取倒数，cx、cy取负
 * @param[in] oneOverVoxelSize voxel size的倒数
 * @param[in] mu TSDF的截断值对应的距离
 * @param[in] viewFrustum_minmax ray的最大最小深度
 * @return  
 */
template <class TVoxel, class TIndex, bool modifyVisibleEntries>
_CPU_AND_GPU_CODE_ inline bool
castRay(DEVICEPTR(Vector4f) & pt_out, DEVICEPTR(uchar) * entriesVisibleType, int x, int y,
        const CONSTPTR(TVoxel) * voxelData, const CONSTPTR(typename TIndex::IndexData) * voxelIndex, Matrix4f invM,
        Vector4f invProjParams, float oneOverVoxelSize, float mu, const CONSTPTR(Vector2f) & viewFrustum_minmax) {

  float stepScale = mu * oneOverVoxelSize;  // TSDF截断值对应的voxel数量
  //! 使用depth的最大最小值 && 相机内参的反投影 来确定三维空间中ray的起点、终点、单位方向
  Vector4f pt_camera_f;   // ray的三维齐次坐标
  pt_camera_f.z = viewFrustum_minmax.x;
  pt_camera_f.x = pt_camera_f.z * ((float(x) + invProjParams.z) * invProjParams.x); // X = Z_min * (u - cx) / fx
  pt_camera_f.y = pt_camera_f.z * ((float(y) + invProjParams.w) * invProjParams.y); // Y = Z_min * (v - cy) / fy
  pt_camera_f.w = 1.0f;
  float totalLength = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;     // ray的最小长度，作为后面的初始长度
  Vector3f pt_block_s = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;    // ray的起点，为voxel坐标

  pt_camera_f.z = viewFrustum_minmax.y;
  pt_camera_f.x = pt_camera_f.z * ((float(x) + invProjParams.z) * invProjParams.x);
  pt_camera_f.y = pt_camera_f.z * ((float(y) + invProjParams.w) * invProjParams.y);
  pt_camera_f.w = 1.0f;
  float totalLengthMax = length(TO_VECTOR3(pt_camera_f)) * oneOverVoxelSize;  // ray的最大长度。用来限定ray的距离
  Vector3f pt_block_e = TO_VECTOR3(invM * pt_camera_f) * oneOverVoxelSize;    // ray的终点

  Vector3f rayDirection = pt_block_e - pt_block_s;
  float direction_norm =
      1.0f / sqrt(rayDirection.x * rayDirection.x + rayDirection.y * rayDirection.y + rayDirection.z * rayDirection.z);
  rayDirection *= direction_norm;                                             // ray的单位方向

  //! 从起点开始 增加ray的长度 直到找到第一个SDF为负的voxel
  Vector3f pt_result = pt_block_s;
  typename TIndex::IndexCache cache;
  float stepLength, sdfValue = 1.0f;
  int vmIndex;  // ray所在block的hash id。表示是否找到了非空的block，若找不到=0
  while (totalLength < totalLengthMax) {
    // 读取tsdf
    sdfValue = readFromSDF_float_uninterpolated(voxelData, voxelIndex, pt_result, vmIndex, cache);
    // 找到了block，顺带更新可见性（如果需要的话）
    if (modifyVisibleEntries) { // entry的可见性在这儿更新
      if (vmIndex)
        entriesVisibleType[vmIndex - 1] = 1;  // 减1是因为读取TSDF中vmIndex=hash_id+1（∵hash_id从0开始）
    }
    // 更新步长
    if (!vmIndex) {   // 还没找到有效block，使用大步长（=block size）
      stepLength = SDF_BLOCK_SIZE;
    } else {          // 找到有效block，使用小步长（在block内部搜索）
      if ((sdfValue <= 0.1f) && (sdfValue >= -0.5f))  // 距离等值面很近了，插值读取TSDF(第一次三线性插值)
        sdfValue = readFromSDF_float_interpolated(voxelData, voxelIndex, pt_result, vmIndex, cache);
      if (sdfValue <= 0.0f) break;                    //! 找到第一个SDF值为负的voxel停止 
      stepLength = MAX(sdfValue * stepScale, 1.0f);   // 计算当前位置到等值面有多少个voxel，作为步长
    }
    // ray前进
    pt_result += stepLength * rayDirection;
    totalLength += stepLength;
  }
  //! 找到后往回退就可以找到等值面 
  bool pt_found;
  float confidence;
  if (sdfValue <= 0.0f) {
    // 步子回退
    stepLength = sdfValue * stepScale;
    pt_result += stepLength * rayDirection;
    // 读取TSDF和confidence，然后再次更新步长。因为是拿等值面附近的voxel再次插值，所以更准确
    sdfValue =
        readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, pt_result, vmIndex, cache);

    stepLength = sdfValue * stepScale;
    pt_result += stepLength * rayDirection;

    pt_found = true;
  } else
    pt_found = false;  // TODO: 什么情况下会出现false？？？或者说什么时候sdfValue会一直不小于0？？？

  pt_out.x = pt_result.x;   // TODO: 为啥不放到if (pt_found)里面？？？难道pt_found=false也要用到？？？
  pt_out.y = pt_result.y;
  pt_out.z = pt_result.z;
  if (pt_found)
    pt_out.w = confidence + 1.0f; // +1是为了防止置信度之前没设置过一直为0
  else
    pt_out.w = 0.0f;

  return pt_found;
}
/**
 * 将三维点（真实坐标）投影到成像平面
 * @param[in] pixel       三维点（真实世界坐标）
 * @param[in] M           当前相机的位姿
 * @param[in] projParams  相机内参
 * @param[in] imgSize     图像大小
 * @return 三维点在图像中的像素坐标（从二维转成一维）
 */
_CPU_AND_GPU_CODE_ inline int forwardProjectPixel(Vector4f pixel, const CONSTPTR(Matrix4f) & M,
                                                  const CONSTPTR(Vector4f) & projParams,
                                                  const THREADPTR(Vector2i) & imgSize) {
  pixel.w = 1;
  pixel = M * pixel;  // 从世界坐标 到 相机的局部坐标

  Vector2f pt_image;
  pt_image.x = projParams.x * pixel.x / pixel.z + projParams.z; // 投影
  pt_image.y = projParams.y * pixel.y / pixel.z + projParams.w;

  if ((pt_image.x < 0) || (pt_image.x > imgSize.x - 1) || (pt_image.y < 0) || (pt_image.y > imgSize.y - 1))
    return -1;

  return (int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x; // +0.5是为了四舍五入
}
/**
 * 直接从TSDF中计算 单个voxel坐标（小数）的单位法向量
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[in] foundPoint      当前voxel坐标是否有有效数据
 * @param[in] point           三维点（voxel坐标）。注意是小数
 * @param[in] voxelBlockData  voxel block array
 * @param[in] indexData       hash table
 * @param[in] lightSource     相机光心位置
 * @param[out] outNormal      计算到的法向量
 * @param[out] angle          法向量与相机光心的夹角
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool) & foundPoint, const THREADPTR(Vector3f) & point,
                                                     const CONSTPTR(TVoxel) * voxelBlockData,
                                                     const CONSTPTR(typename TIndex::IndexData) * indexData,
                                                     const THREADPTR(Vector3f) & lightSource,
                                                     THREADPTR(Vector3f) & outNormal, THREADPTR(float) & angle) {
  if (!foundPoint) return;
  //! 计算单位法向量
  outNormal = computeSingleNormalFromSDF(voxelBlockData, indexData, point);
  float normScale = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
  outNormal *= normScale;   // 归一化
  //! 计算夹角的cosine
  angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
  if (!(angle > 0.0)) // 角度<0说明是从背面看过来的，丢弃
    foundPoint = false;
}
/**
 * 计算有序点云中单个点法向量和夹角，使用有限差分法
 * @tparam useSmoothing 是否使用更大的范围计算，从而达到平滑的目的
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[in, out] foundPoint 该点是否有效。在图像边缘被强制设为无效（因为没法算法向量）
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[in] pointsRay 有序点云（voxel坐标）。第四个维度是权重
 * @param[in] lightSource 相机光心位置。用来计算夹角
 * @param[in] voxelSize
 * @param[in] imgSize 有序点云对应的图像大小（x+y）
 * @param[out] outNormal 该点的法向量
 * @param[out] angle 该点的法向量与lightSource的夹角的cosine。范围0-1
 */
template <bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void
computeNormalAndAngle(THREADPTR(bool) & foundPoint, const THREADPTR(int) & x, const THREADPTR(int) & y,
                      const CONSTPTR(Vector4f) * pointsRay, const THREADPTR(Vector3f) & lightSource,
                      const THREADPTR(float) & voxelSize, const THREADPTR(Vector2i) & imgSize,
                      THREADPTR(Vector3f) & outNormal, THREADPTR(float) & angle) {
  if (!foundPoint)
    return;
  //! 取相邻像素
  Vector4f xp1_y, xm1_y, x_yp1, x_ym1;
  if (useSmoothing) {   // 如果要平滑，则用更大的
    if (y <= 2 || y >= imgSize.y - 3 || x <= 2 || x >= imgSize.x - 3) {   // 图像边缘的不算
      foundPoint = false;
      return;
    }

    xp1_y = pointsRay[(x + 2) + y * imgSize.x], x_yp1 = pointsRay[x + (y + 2) * imgSize.x];
    xm1_y = pointsRay[(x - 2) + y * imgSize.x], x_ym1 = pointsRay[x + (y - 2) * imgSize.x];
  } else {
    if (y <= 1 || y >= imgSize.y - 2 || x <= 1 || x >= imgSize.x - 2) {
      foundPoint = false;
      return;
    }

    xp1_y = pointsRay[(x + 1) + y * imgSize.x], x_yp1 = pointsRay[x + (y + 1) * imgSize.x];
    xm1_y = pointsRay[(x - 1) + y * imgSize.x], x_ym1 = pointsRay[x + (y - 1) * imgSize.x];
  }
  //! 计算差分
  Vector4f diff_x(0.0f, 0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f, 0.0f);
  bool doPlus1 = false;   // 是否重新计算
  if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0) // 只要相邻点有一个无效，就重算
    doPlus1 = true;
  else {
    diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

    float length_diff = MAX(diff_x.x * diff_x.x + diff_x.y * diff_x.y + diff_x.z * diff_x.z,
                            diff_y.x * diff_y.x + diff_y.y * diff_y.y + diff_y.z * diff_y.z);

    if (length_diff * voxelSize * voxelSize > (0.15f * 0.15f))      // 差分的模长>0.15米 就重算（可能是物体边缘）
      doPlus1 = true;
  }

  if (doPlus1) {  
    if (useSmoothing) {   // 只有前面用更大范围计算的时候才要重算
      xp1_y = pointsRay[(x + 1) + y * imgSize.x];
      x_yp1 = pointsRay[x + (y + 1) * imgSize.x];
      xm1_y = pointsRay[(x - 1) + y * imgSize.x];
      x_ym1 = pointsRay[x + (y - 1) * imgSize.x];
      diff_x = xp1_y - xm1_y;
      diff_y = x_yp1 - x_ym1;
    }

    if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0) {
      foundPoint = false;
      return;
    }
  }
  //! 叉乘得到法向量 && 归一化
  outNormal.x = -(diff_x.y * diff_y.z - diff_x.z * diff_y.y);
  outNormal.y = -(diff_x.z * diff_y.x - diff_x.x * diff_y.z);
  outNormal.z = -(diff_x.x * diff_y.y - diff_x.y * diff_y.x);

  if (flipNormals)
    outNormal = -outNormal;

  float normScale = 1.0f / sqrt(outNormal.x * outNormal.x + outNormal.y * outNormal.y + outNormal.z * outNormal.z);
  outNormal *= normScale;
  //! 计算夹角的cosine
  angle = outNormal.x * lightSource.x + outNormal.y * lightSource.y + outNormal.z * lightSource.z;
  if (!(angle > 0.0))
    foundPoint = false;
}
/**
 * @brief 将夹角转成灰色
 * @param[out] dest       灰色。范围0-255 
 * @param[in] angle       当前像素法向量跟相机光心的夹角的cosine。范围0-1
 */
_CPU_AND_GPU_CODE_ inline void drawPixelGrey(DEVICEPTR(Vector4u) & dest, const THREADPTR(float) & angle) {
  float outRes = (0.8f * angle + 0.2f) * 255.0f;
  dest = Vector4u((uchar)outRes);
}
// TODO: ???
_CPU_AND_GPU_CODE_ inline float interpolateCol(float val, float y0, float x0, float y1, float x1) {
  return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}
// TODO: ???
_CPU_AND_GPU_CODE_ inline float baseCol(float val) {
  if (val <= -0.75f)
    return 0.0f;
  else if (val <= -0.25f)
    return interpolateCol(val, 0.0f, -0.75f, 1.0f, -0.25f);
  else if (val <= 0.25f)
    return 1.0f;
  else if (val <= 0.75f)
    return interpolateCol(val, 1.0f, 0.25f, 0.0f, 0.75f);
  else
    return 0.0;
}
/**
 * @brief 根据夹角将置信度转成伪彩色
 * @param[out] dest       伪彩色。范围0-255 
 * @param[in] angle       当前像素法向量跟相机光心的夹角的cosine。范围0-1
 * @param[in] confidence  置信度
 */
_CPU_AND_GPU_CODE_ inline void drawPixelConfidence(DEVICEPTR(Vector4u) & dest, const THREADPTR(float) & angle,
                                                   const THREADPTR(float) & confidence) {
  // Vector4f color_red(255, 0, 0, 255), color_green(0, 255, 0, 255);
  float confidenceNorm = CLAMP(confidence, 0, 100.f) / 100.0f;

  Vector4f color;
  color.r = (uchar)(baseCol(confidenceNorm) * 255.0f);
  color.g = (uchar)(baseCol(confidenceNorm - 0.5f) * 255.0f);
  color.b = (uchar)(baseCol(confidenceNorm + 0.5f) * 255.0f);
  color.a = 255;

  Vector4f outRes = (0.8f * angle + 0.2f) * color;  // 夹角越小（cos越接近1），颜色越纯正
  dest = TO_UCHAR4(outRes);
}
/**
 * @brief 将单位法向量转成伪彩色
 * @param[out] dest        伪彩色。范围0-255 
 * @param[in] normal_obj  单位法向量
 */
_CPU_AND_GPU_CODE_ inline void drawPixelNormal(DEVICEPTR(Vector4u) & dest, const THREADPTR(Vector3f) & normal_obj) {
  dest.r = (uchar)((0.3f + (-normal_obj.r + 1.0f) * 0.35f) * 255.0f); // TODO：找到公式来源
  dest.g = (uchar)((0.3f + (-normal_obj.g + 1.0f) * 0.35f) * 255.0f);
  dest.b = (uchar)((0.3f + (-normal_obj.b + 1.0f) * 0.35f) * 255.0f);
}

/**
 * 将单个三维点 投影成 彩色像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] dest           彩色信息。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] point           三维点（voxel坐标）
 * @param[in] voxelBlockData  voxel block array
 * @param[in] indexData       hash table
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawPixelColour(DEVICEPTR(Vector4u) & dest, const CONSTPTR(Vector3f) & point,
                                               const CONSTPTR(TVoxel) * voxelBlockData,
                                               const CONSTPTR(typename TIndex::IndexData) * indexData) {
  Vector4f clr =
      VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelBlockData, indexData, point);

  dest.x = (uchar)(clr.x * 255.0f);
  dest.y = (uchar)(clr.y * 255.0f);
  dest.z = (uchar)(clr.z * 255.0f);
  dest.w = 255;       // 不透明度
}
/**
 * 计算 voxel点云 中 单个像素的真实坐标 && 法向量
 * @tparam useSmoothing 是否使用更大的范围计算，从而达到平滑的目的
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] pointsMap 最终点云（真实坐标）
 * @param[out] normalsMap 点云对应的法向量
 * @param[in] pointsRay 有序点云（voxel坐标）。第四个维度是权重
 * @param[in] imgSize 有序点云对应的图像大小（x+y）
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[in] voxelSize   voxel的真实尺寸
 * @param[in] lightSource 相机光心位置。用来计算夹角
 * @note 好像是raycasting专用的
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelICP(DEVICEPTR(Vector4f) & pointsMap, DEVICEPTR(Vector4f) & normalsMap,
                                               const THREADPTR(Vector3f) & point, bool foundPoint,
                                               const CONSTPTR(TVoxel) * voxelData,
                                               const CONSTPTR(typename TIndex::IndexData) * voxelIndex, float voxelSize,
                                               const THREADPTR(Vector3f) & lightSource) {
  Vector3f outNormal;
  float angle;

  computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);
  //! 转真实坐标 && 记录
  if (foundPoint) {
    Vector4f outPoint4;
    outPoint4.x = point.x * voxelSize;
    outPoint4.y = point.y * voxelSize;
    outPoint4.z = point.z * voxelSize;
    outPoint4.w = 1.0f;
    pointsMap = outPoint4;

    Vector4f outNormal4;
    outNormal4.x = outNormal.x;
    outNormal4.y = outNormal.y;
    outNormal4.z = outNormal.z;
    outNormal4.w = 0.0f;
    normalsMap = outNormal4;
  } else {
    Vector4f out4;
    out4.x = 0.0f;
    out4.y = 0.0f;
    out4.z = 0.0f;
    out4.w = -1.0f;

    pointsMap = out4;
    normalsMap = out4;
  }
}
/**
 * 计算有序voxel点云中 单个像素的真实坐标 && 法向量
 * @tparam useSmoothing 是否使用更大的范围计算，从而达到平滑的目的
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] pointsMap 最终点云（真实坐标）
 * @param[out] normalsMap 点云对应的法向量
 * @param[in] pointsRay 有序点云（voxel坐标）。第四个维度是权重
 * @param[in] imgSize 有序点云对应的图像大小（x+y）
 * @param[in] x 像素坐标
 * @param[in] y 像素坐标
 * @param[in] voxelSize   voxel的真实尺寸
 * @param[in] lightSource 相机光心位置。用来计算夹角
 * @note 好像是raycasting专用的
 */
template <bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void
processPixelICP(DEVICEPTR(Vector4f) * pointsMap, DEVICEPTR(Vector4f) * normalsMap, const CONSTPTR(Vector4f) * pointsRay,
                const THREADPTR(Vector2i) & imgSize, const THREADPTR(int) & x, const THREADPTR(int) & y,
                float voxelSize, const THREADPTR(Vector3f) & lightSource) {
  //! 计算 normal && 夹角的cosine
  Vector3f outNormal;
  float angle;  // 没用到

  int locId = x + y * imgSize.x;
  Vector4f point = pointsRay[locId];
  bool foundPoint = point.w > 0.0f;

  computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize,
                                                   outNormal, angle);
  //! 转真实坐标 && 记录
  if (foundPoint) {
    Vector4f outPoint4;
    outPoint4.x = point.x * voxelSize;    // 真实坐标
    outPoint4.y = point.y * voxelSize;
    outPoint4.z = point.z * voxelSize;
    outPoint4.w = point.w; // outPoint4.w = 1.0f;
    pointsMap[locId] = outPoint4;

    Vector4f outNormal4;
    outNormal4.x = outNormal.x;
    outNormal4.y = outNormal.y;
    outNormal4.z = outNormal.z;
    outNormal4.w = 0.0f;
    normalsMap[locId] = outNormal4;
  } else {
    Vector4f out4;
    out4.x = 0.0f;
    out4.y = 0.0f;
    out4.z = 0.0f;
    out4.w = -1.0f;

    pointsMap[locId] = out4;
    normalsMap[locId] = out4;
  }
}
/**
 * @brief 从有序点云中渲染 法向量夹角图 的单个像素（灰色）
 * @tparam useSmoothing 是否使用更大的范围计算，从而达到平滑的目的
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] outRendering 灰度信息。通过邻域像素插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] pointsRay     有序点云投影到的三维点（voxel坐标）。第4个通道是有效性？？？
 * @param[in] imgSize       图像分辨率
 * @param[in] x             像素坐标
 * @param[in] y             像素坐标
 * @param[in] voxelSize     voxel的真实尺寸
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void
processPixelGrey_ImageNormals(DEVICEPTR(Vector4u) * outRendering, const CONSTPTR(Vector4f) * pointsRay,
                              const THREADPTR(Vector2i) & imgSize, const THREADPTR(int) & x, const THREADPTR(int) & y,
                              float voxelSize, const THREADPTR(Vector3f) & lightSource) {
  //! 计算法向量和夹角的cosine
  Vector3f outNormal;
  float angle;

  int locId = x + y * imgSize.x;
  Vector4f point = pointsRay[locId];

  bool foundPoint = point.w > 0.0f;
  computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize,
                                                   outNormal, angle);
  //! 根据夹角伪彩色渲染
  if (foundPoint)
    drawPixelGrey(outRendering[locId], angle);  // 因为是有序点云（非三维场景），不会存在夹角>90的情况，所以用角度渲染就好灰度
  else
    outRendering[locId] = Vector4u((uchar)0);
}
/**
 * @brief 从有序点云中渲染 法向量图 的单个像素
 * @tparam useSmoothing 是否使用更大的范围计算，从而达到平滑的目的
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] outRendering 伪彩色信息。通过邻域像素插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] pointsRay     有序点云投影到的三维点（voxel坐标）。第4个通道是有效性？？？
 * @param[in] x             像素坐标
 * @param[in] y             像素坐标
 * @param[in] voxelSize     voxel的真实尺寸
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void
processPixelNormals_ImageNormals(DEVICEPTR(Vector4u) * outRendering, const CONSTPTR(Vector4f) * pointsRay,
                                 const THREADPTR(Vector2i) & imgSize, const THREADPTR(int) & x,
                                 const THREADPTR(int) & y, float voxelSize, Vector3f lightSource) {
  //! 计算法向量和夹角的cosine
  Vector3f outNormal;
  float angle;

  int locId = x + y * imgSize.x;
  Vector4f point = pointsRay[locId];

  bool foundPoint = point.w > 0.0f;
  computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize,
                                                   outNormal, angle);
  //! 根据法向量伪彩色渲染
  if (foundPoint)
    drawPixelNormal(outRendering[locId], outNormal);
  else
    outRendering[locId] = Vector4u((uchar)0);
}
/**
 * @brief 从有序点云中渲染 置信度图 的单个像素
 * @tparam useSmoothing 是否使用更大的范围计算，从而达到平滑的目的
 * @tparam flipNormals 计算出来的法向量是否要翻转，∵某些相机内参的焦距为负
 * @param[out] outRendering 伪彩色信息。通过邻域像素插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] pointsRay     有序点云投影到的三维点（voxel坐标）。第4个通道是有效性？？？
 * @param[in] x             像素坐标
 * @param[in] y             像素坐标
 * @param[in] voxelSize     voxel的真实尺寸
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void
processPixelConfidence_ImageNormals(DEVICEPTR(Vector4u) * outRendering, const CONSTPTR(Vector4f) * pointsRay,
                                    const THREADPTR(Vector2i) & imgSize, const THREADPTR(int) & x,
                                    const THREADPTR(int) & y, float voxelSize, Vector3f lightSource) {
  Vector3f outNormal;
  float angle;

  int locId = x + y * imgSize.x;
  Vector4f point = pointsRay[locId];

  bool foundPoint = point.w > 0.0f;
  computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize,
                                                   outNormal, angle);

  if (foundPoint)
    drawPixelConfidence(outRendering[locId], angle, point.w - 1.0f);  // -1是因为raycast的时候+1，见castray()
  else
    outRendering[locId] = Vector4u((uchar)0);
}
/**
 * @brief 从三维场景中渲染 法向量夹角图 的单个像素（灰色）
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 灰色信息。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] point         三维点（voxel坐标）
 * @param[in] foundPoint    三维点是否有效
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelGrey(DEVICEPTR(Vector4u) & outRendering, const CONSTPTR(Vector3f) & point,
                                                bool foundPoint, const CONSTPTR(TVoxel) * voxelData,
                                                const CONSTPTR(typename TIndex::IndexData) * voxelIndex,
                                                Vector3f lightSource) {
  Vector3f outNormal;
  float angle;

  computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

  if (foundPoint)
    drawPixelGrey(outRendering, angle);
  else
    outRendering = Vector4u((uchar)0);
}
/**
 * @brief 从三维场景渲染 彩色图 的单个像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 彩色信息。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] point         三维点（voxel坐标）
 * @param[in] foundPoint    三维点是否有效
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelColour(DEVICEPTR(Vector4u) & outRendering, const CONSTPTR(Vector3f) & point,
                                                  bool foundPoint, const CONSTPTR(TVoxel) * voxelData,
                                                  const CONSTPTR(typename TIndex::IndexData) * voxelIndex) {
  if (foundPoint)
    drawPixelColour<TVoxel, TIndex>(outRendering, point, voxelData, voxelIndex);
  else
    outRendering = Vector4u((uchar)0);
}
/**
 * @brief 从三维场景渲染 法向量图 的单个像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 伪彩色信息。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] point         三维点（voxel坐标）
 * @param[in] foundPoint    三维点是否有效
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] lightSource   相机光心位置。用来计算夹角
 * @note 是单位法向量
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelNormal(DEVICEPTR(Vector4u) & outRendering, const CONSTPTR(Vector3f) & point,
                                                  bool foundPoint, const CONSTPTR(TVoxel) * voxelData,
                                                  const CONSTPTR(typename TIndex::IndexData) * voxelIndex,
                                                  Vector3f lightSource) {
  //! 直接从TSDF中计算得到当前voxel坐标的单位法向量和夹角
  Vector3f outNormal;
  float angle;    // 法向量与相机光心的夹角的cosine。范围0-1
  computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);
  //! 伪彩色渲染
  if (foundPoint)
    drawPixelNormal(outRendering, outNormal);
  else
    outRendering = Vector4u((uchar)0);
}
/**
 * @brief 从三维场景中渲染 置信度图 的单个像素
 * @tparam TVoxel voxel的存储类型。比如用short还是float存TSDF值，要不要存RGB
 * @tparam TIndex voxel的索引方法。用 hashing 还是 下标（跟KinectFusion一样）
 * @param[out] outRendering 伪彩色信息。通过邻域voxel三线性插值得到，范围0-255。第四个通道应该是不透明度???
 * @param[in] point         三维点（voxel坐标）
 * @param[in] foundPoint    三维点是否有效
 * @param[in] voxelData     voxel block array
 * @param[in] voxelIndex    hash table
 * @param[in] lightSource   相机光心位置。用来计算夹角
 */
template <class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void
processPixelConfidence(DEVICEPTR(Vector4u) & outRendering, const CONSTPTR(Vector4f) & point, bool foundPoint,
                       const CONSTPTR(TVoxel) * voxelData, const CONSTPTR(typename TIndex::IndexData) * voxelIndex,
                       Vector3f lightSource) {
  //! 直接从TSDF中计算得到当前voxel坐标的单位法向量和夹角
  Vector3f outNormal;
  float angle;  // 法向量与相机光心的夹角的cosine。范围0-1
  computeNormalAndAngle<TVoxel, TIndex>(foundPoint, TO_VECTOR3(point), voxelData, voxelIndex, lightSource, outNormal,
                                        angle); // 使用TO_VECTOR3是因为point是4个通道。其他的渲染函数是直接传入3个通道的point
  //! 伪彩色渲染
  if (foundPoint)
    drawPixelConfidence(outRendering, angle, point.w - 1.0f);   // -1是因为raycast的时候+1，见castray()
  else
    outRendering = Vector4u((uchar)0);
}
