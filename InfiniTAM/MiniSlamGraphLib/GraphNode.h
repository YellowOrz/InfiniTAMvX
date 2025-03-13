// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdlib.h>

namespace MiniSlamGraph {
/** 图节点 */
class GraphNode {
 public:
  GraphNode(void) {
    mFixed = false;
    mId = -1;
  }
  GraphNode(const GraphNode &src) {
    mFixed = src.mFixed;
    mId = src.mId;
  }
  virtual ~GraphNode(void) {}

  virtual GraphNode *clone(void) const = 0;
  /**
   * @brief                     更新节点的变量
   * @param[in] delta           变量的变化量
   * @param[in] startingPoint   变量的初始值
   * @note                      更新后的变量存在当前节点
   */
  virtual void applyDelta(const double *delta, const GraphNode *startingPoint = NULL) = 0;

  virtual int numParameters(void) const = 0;

  virtual void setParameters(const double *v) = 0;
  virtual void getParameters(double *v) = 0;

  bool isFixed(void) const { return mFixed; }
  void setFixed(bool value = true) { mFixed = value; }

  int getId(void) const { return mId; }
  void setId(int id) { mId = id; }

 private:
  bool mFixed;  // 当前节点是否固定。固定后就不参与优化
  int mId;      // 对应子图的全局id
};
}
