// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <iosfwd>

#include "ITMRGBDCalib.h"

namespace ITMLib {
/** 从打开的文件流中读取内参（4个数字） */
bool readIntrinsics(std::istream &src, ITMIntrinsics &dest);
/** 根据文件路径读取内参（4个数字） */
bool readIntrinsics(const char *fileName, ITMIntrinsics &dest);
/** 从打开的文件流中读取外参（4x4的齐次矩阵） */
bool readExtrinsics(std::istream &src, ITMExtrinsics &dest);
/** 根据文件路径读取外参（4x4的齐次矩阵） */
bool readExtrinsics(const char *fileName, ITMExtrinsics &dest);
bool readDisparityCalib(std::istream &src, ITMDisparityCalib &dest);
bool readDisparityCalib(const char *fileName, ITMDisparityCalib &dest);
bool readRGBDCalib(std::istream &src, ITMRGBDCalib &dest);
bool readRGBDCalib(const char *fileName, ITMRGBDCalib &dest);
bool readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile,
                   const char *extrinsicsFile, ITMRGBDCalib &dest);
void writeIntrinsics(std::ostream &dest, const ITMIntrinsics &src);
void writeExtrinsics(std::ostream &dest, const ITMExtrinsics &src);
void writeDisparityCalib(std::ostream &dest, const ITMDisparityCalib &src);
void writeRGBDCalib(std::ostream &dest, const ITMRGBDCalib &src);
void writeRGBDCalib(const char *fileName, const ITMRGBDCalib &src);
}
