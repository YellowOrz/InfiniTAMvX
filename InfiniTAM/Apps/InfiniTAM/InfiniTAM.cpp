// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>

#include "UIEngine.h"

#include "../../InputSource/OpenNIEngine.h"
#include "../../InputSource/Kinect2Engine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/PicoFlexxEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/LibUVCEngine.h"
#include "../../InputSource/RealSenseEngine.h"
#include "../../InputSource/FFMPEGReader.h"
#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

/** Create a default source of depth images from a list of command line
    arguments. Typically, @para arg1 would identify the calibration file to
    use, @para arg2 the colour images, @para arg3 the depth images and
    @para arg4 the IMU images. If images are omitted, some live sources will
    be tried.
*/
/**
 * 创建图像数据源
 * @param[out] imageSource
 * @param[out] imuSource
 * @param[in] arg1 相机标定文件路径
 * @param[in] arg2 彩色图所在文件夹路径
 * @param[in] arg3 深度图所在文件夹路径
 * @param[in] arg4 IMU文件路径
 */
static void CreateDefaultImageSource(ImageSourceEngine *&imageSource,
                                     IMUSourceEngine *&imuSource,
                                     const char *arg1,
                                     const char *arg2,
                                     const char *arg3,
                                     const char *arg4) {
  const char *calibFile = arg1;
  const char *filename1 = arg2;
  const char *filename2 = arg3;
  const char *filename_imu = arg4;
  //! 相机标定文件路径为viewer，则创建空白图片数据源，之后使用viewer mode
  if (strcmp(calibFile, "viewer") == 0) {
    imageSource = new BlankImageGenerator("", Vector2i(640, 480));
    printf("starting in viewer mode: make sure to press n first to initiliase the views ... \n");
    return;
  }

  printf("using calibration file: %s\n", calibFile);
  //! 创建彩色图、深度图[、IMU]的数据源
  if ((imageSource == NULL) && (filename2 != NULL)) {
    printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
    if (filename_imu == NULL) {   // 读取深度图和彩色图
      ImageMaskPathGenerator pathGenerator(filename1, filename2);
      imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
    } else {                      // 读取深度图和彩色图和IMU
      printf("using imu data: %s\n", filename_imu);
      // TODO(xzf): 为啥不用上面的ImageFileReader?
      imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
      imuSource = new IMUSourceEngine(filename_imu);
    }
    // 深度图读取失败，则删除数据源
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      if (imuSource != NULL) delete imuSource;
      imuSource = NULL;
      imageSource = NULL;
    }
  }
  //! 如果上面图片的数据源读取失败，则用FFMPEG读取
  if ((imageSource == NULL) && (filename1 != NULL) && (filename_imu == NULL)) {
    imageSource = new InputSource::FFMPEGReader(calibFile, filename1, filename2);
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      imageSource = NULL;
    }
  }
  //! 还是读取失败，则用OpenNI打开相机
  if (imageSource == NULL) {
    // If no calibration file specified, use the factory default calibration
    bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

    printf("trying OpenNI device: %s - calibration: %s\n",
           filename1 ? filename1 : "<OpenNI default device>",
           useInternalCalibration ? "internal" : "from file");
    imageSource = new OpenNIEngine(calibFile, filename1, useInternalCalibration);
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      imageSource = NULL;
    }
  }
  //! 还是读取失败，使用UVC设备（USB相机）
  if (imageSource == NULL) {
    printf("trying UVC device\n");
    imageSource = new LibUVCEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      imageSource = NULL;
    }
  }
  //! 还是读取失败，使用RealSense相机
  if (imageSource == NULL) {
    printf("trying RealSense device\n");
    imageSource = new RealSenseEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      imageSource = NULL;
    }
  }
  //! 还是读取失败，使用Kinect2相机
  if (imageSource == NULL) {
    printf("trying MS Kinect 2 device\n");
    imageSource = new Kinect2Engine(calibFile);
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      imageSource = NULL;
    }
  }
  //! 还是读取失败，使用Pico Flexx相机
  if (imageSource == NULL) {
    printf("trying PMD PicoFlexx device\n");
    imageSource = new PicoFlexxEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0) {
      delete imageSource;
      imageSource = NULL;
    }
  }
}

int main(int argc, char **argv)
try {
  //! 解析命令行参数
  const char *arg1 = "";    // 相机标定文件路径
  const char *arg2 = NULL;  // 彩色图所在文件夹路径
  const char *arg3 = NULL;  // 深度图所在文件夹路径
  const char *arg4 = NULL;  // IMU文件路径

  int arg = 1;
  do {
    if (argv[arg] != NULL) arg1 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg2 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg3 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg4 = argv[arg]; else break;
  } while (false);

  if (arg == 1) {
    printf("usage: %s [<calibfile> [<imagesource>] ]\n"
           "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
           "  <imagesource> : either one argument to specify OpenNI device ID\n"
           "                  or two arguments specifying rgb and depth file masks\n"
           "\n"
           "examples:\n"
           "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
           "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0], argv[0]);
  }
  //! 初始化 图像和IMU的数据源
  printf("initialising ...\n");
  ImageSourceEngine *imageSource = NULL;
  IMUSourceEngine *imuSource = NULL;

  CreateDefaultImageSource(imageSource, imuSource, arg1, arg2, arg3, arg4);
  if (imageSource == NULL) {    // 图像必须有，IMU可选
    std::cout << "failed to open any image stream" << std::endl;
    return -1;
  }
  //! 初始化 系统设置
  ITMLibSettings *internalSettings = new ITMLibSettings();
  //! 根据设置中的模式，构建SLAM系统
  ITMMainEngine *mainEngine = NULL;
  switch (internalSettings->libMode) {
    case ITMLibSettings::LIBMODE_BASIC:
      mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings,
                                                               imageSource->getCalib(),
                                                               imageSource->getRGBImageSize(),
                                                               imageSource->getDepthImageSize());
      break;
    case ITMLibSettings::LIBMODE_BASIC_SURFELS:
      mainEngine = new ITMBasicSurfelEngine<ITMSurfelT>(internalSettings,
                                                        imageSource->getCalib(),
                                                        imageSource->getRGBImageSize(),
                                                        imageSource->getDepthImageSize());
      break;
    case ITMLibSettings::LIBMODE_LOOPCLOSURE:
      mainEngine = new ITMMultiEngine<ITMVoxel, ITMVoxelIndex>(internalSettings,
                                                               imageSource->getCalib(),
                                                               imageSource->getRGBImageSize(),
                                                               imageSource->getDepthImageSize());
      break;
    default: throw std::runtime_error("Unsupported library mode!");
      break;
  }
  //! 初始化SLAM系统的UI界面
  UIEngine::Instance()->Initialise(argc,
                                   argv,
                                   imageSource,
                                   imuSource,
                                   mainEngine,
                                   "./Files/Out",
                                   internalSettings->deviceType);
  //! 运行SLAM系统
  UIEngine::Instance()->Run();
  //! 关闭SLAM系统
  UIEngine::Instance()->Shutdown();

  delete mainEngine;
  delete internalSettings;
  delete imageSource;
  if (imuSource != NULL) delete imuSource;
  return 0;
}
catch (std::exception &e) {
  std::cerr << e.what() << '\n';
  return EXIT_FAILURE;
}

