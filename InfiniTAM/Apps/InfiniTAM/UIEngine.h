// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../InputSource/ImageSourceEngine.h"
#include "../../InputSource/IMUSourceEngine.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Core/ITMMainEngine.h"
#include "../../ITMLib/Utils/ITMLibSettings.h"
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/NVTimer.h"

#include <vector>

namespace InfiniTAM {
namespace Engine {
class UIEngine {
  static UIEngine *instance;
  /** 处理模式 */
  enum MainLoopAction {
    PROCESS_PAUSED,   /**< 暂停处理 */
    PROCESS_FRAME,    /**< 单帧处理 */
    PROCESS_VIDEO,    /**< 连续处理 */
    EXIT,             /**< 退出 */
    SAVE_TO_DISK      /**< 保存图片序列到硬盘 */
  } mainLoopAction;
  /** 可视化界面色彩显示模式 */
  struct UIColourMode {
    const char *name;     // 模式名称
    ITMLib::ITMMainEngine::GetImageType type;   // 模式类型
    UIColourMode(const char *_name, ITMLib::ITMMainEngine::GetImageType _type)
        : name(_name), type(_type) {}
  };
  std::vector<UIColourMode> colourModes_main,       // 固定视角（跟随相机）下，可视化界面色彩显示模式的多个选项
                            colourModes_freeview;   // 自由视角下，可视化界面色彩显示模式的多个选项
  int currentColourMode;                            // 当前可视化界面色彩显示模式

  InputSource::ImageSourceEngine *imageSource;  // 图像序列数据来源
  InputSource::IMUSourceEngine *imuSource;      // IMU数据来源
//  ITMLib::ITMLibSettings internalSettings;      // 没用到
  ITMLib::ITMMainEngine *mainEngine;            // 主引擎

  StopWatchInterface *timer_instant;
  StopWatchInterface *timer_average;

 private: // For UI layout
  /* UI布局 */
  static const int NUM_WIN = 3;     // 子窗口数量
  Vector4f winReg[NUM_WIN];         // 子窗口范围。(x1, y1, x2, y2)，取值范围0-1
  Vector2i winSize;                 // 整个窗口的尺寸
  uint textureId[NUM_WIN];          // 纹理索引
  ITMUChar4Image *outImage[NUM_WIN];  // 子窗口显示的图片
  ITMLib::ITMMainEngine::GetImageType outImageType[NUM_WIN];  // 子窗口显示图片的类型

  ITMUChar4Image *inputRGBImage;      // 当前帧彩色图
  ITMShortImage *inputRawDepthImage;  // 当前帧原始深度图
  ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

  bool freeviewActive;      // 是否自由视角
  bool integrationActive;   // 是否（重建的时候）进行融合
  ORUtils::SE3Pose freeviewPose;              // 自由视角的相机位姿
  ITMLib::ITMIntrinsics freeviewIntrinsics;   // 自由视角的相机内参

  int mouseState;           // 鼠标状态。0-没有任何操作，1-左键，2-右键，3-中键
  Vector2i mouseLastClick;  // 鼠标上次点击的位置
  bool mouseWarped; // To avoid the extra motion generated by glutWarpPointer TODO(xzf):?

  int currentFrameNo;   // 当前帧数
  bool isRecording;     // 是否同步记录输入的彩色图和深度图
  InputSource::FFMPEGWriter *rgbVideoWriter;    // 使用FFMPEG保存彩色图
  InputSource::FFMPEGWriter *depthVideoWriter;  // 使用FFMPEG保存深度图
 public:
  static UIEngine *Instance(void) {
    if (instance == NULL) instance = new UIEngine();
    return instance;
  }

  static void glutDisplayFunction();
  static void glutIdleFunction();
  /**
   * @brief UI界面中，键盘按键对应功能
   * @param[in] key 键盘按键
   * @param x 没用到。因为OpenGL要求这么写
   * @param y 没用到。因为OpenGL要求这么写
   */
  static void glutKeyUpFunction(unsigned char key, int x, int y);
  static void glutMouseButtonFunction(int button, int state, int x, int y);
  static void glutMouseMoveFunction(int x, int y);
  static void glutMouseWheelFunction(int button, int dir, int x, int y);

  const Vector2i &getWindowSize(void) const { return winSize; }

  float processedTime;
  int processedFrameNo;
  int trackingResult;
  char *outFolder;
  bool needsRefresh;
  ITMUChar4Image *saveImage;
  /**
   * SLAM系统初始化
   * @param[in] argc 命令行参数个数
   * @param[in] argv 命令行参数
   * @param[in] imageSource 图片输入
   * @param[in] imuSource   IMU输入
   * @param[in] mainEngine  主引擎
   * @param[in] outFolder   输出文件夹
   * @param[in] deviceType  设备类型
   */
  void Initialise(int &argc,
                  char **argv,
                  InputSource::ImageSourceEngine *imageSource,
                  InputSource::IMUSourceEngine *imuSource,
                  ITMLib::ITMMainEngine *mainEngine,
                  const char *outFolder,
                  ITMLib::ITMLibSettings::DeviceType deviceType);
  void Shutdown();

  void Run();
  void ProcessFrame();

  void GetScreenshot(ITMUChar4Image *dest) const;
  void SaveScreenshot(const char *filename) const;
};
}
}
