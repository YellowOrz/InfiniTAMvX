// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Objects/Scene/ITMPlainVoxelArray.h"
#include "Objects/Scene/ITMSurfelTypes.h"
#include "Objects/Scene/ITMVoxelBlockHash.h"
#include "Objects/Scene/ITMVoxelTypes.h"

/** Surfel的存储类型。根据是否存储RGB，可选项有ITMSurfel_grey、ITMSurfel_rgb。
 * This chooses the information stored at each surfel. At the moment, valid
    options are ITMSurfel_grey and ITMSurfel_rgb.
*/
typedef ITMLib::ITMSurfel_rgb ITMSurfelT;

/** voxel的存储类型。根据STF值的类型(short|float) && 是否存储RGB，可选项有：ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb,
 * ITMVoxel_f_rgb。具体见文件ITMLib/Objects/Scene/ITMVoxelTypes.h。
 * This chooses the information stored at each voxel. At the moment, valid options are ITMVoxel_s, ITMVoxel_f,
 * ITMVoxel_s_rgb and ITMVoxel_f_rgb.*/
typedef ITMVoxel_s ITMVoxel;

/** voxel的索引方式。有 哈希索引（ITMVoxelBlockHash）和 下标索引（ITMPlainVoxelArray，即KinectFusion那种）两个选项。
 *  This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::ITMVoxelBlockHash ITMVoxelIndex;
// typedef ITMLib::ITMPlainVoxelArray ITMVoxelIndex;
