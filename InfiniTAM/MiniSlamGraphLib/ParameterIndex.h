// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

namespace MiniSlamGraph {
/** 所有节点的参数量（或者叫维度？）信息 */
class ParameterIndex {
 private:
  typedef std::map<int, int> Index;

  Index mIdx;   // 每个节点的参数量。节点对应的子图的全局id + 参数量
  int numPara;  // 所有节点的参数量之和

 public:
  ParameterIndex(void) { numPara = 0; }
  /**
   * @brief 记录单个节点的参数量
   * @param[in] id      节点对应的子图的全局id
   * @param[in] num     节点的参数量
   */
  void addIndex(int id, int num) {
    mIdx[id] = numPara;
    numPara += num;
  }
  /** 根据节点对应的子图的全局id，获取该节点的参数量 */
  int findIndex(int id) const {
    Index::const_iterator it = mIdx.find(id);
    if (it == mIdx.end()) return -1;
    return it->second;
  }
  /** 获取所有节点的参数量之和 */
  int numTotalParameters(void) const {
    return numPara;
  }
};
}