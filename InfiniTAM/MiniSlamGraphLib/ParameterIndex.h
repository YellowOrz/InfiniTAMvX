// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

namespace MiniSlamGraph {
/** 所有节点的参数索引信息 */
class ParameterIndex {
 private:
  typedef std::map<int, int> Index;

  Index mIdx;   // 记录节点对应的子图的全局id，以及它的参数的起始位置
  int numPara;  // 所有节点的参数量之和

 public:
  ParameterIndex(void) { numPara = 0; }
  /**
   * @brief 记录单个节点的参数的起始位置
   * @param[in] id      节点对应的子图的全局id
   * @param[in] num     节点的参数量
   */
  void addIndex(int id, int num) {
    mIdx[id] = numPara;
    numPara += num;
  }
  /**
   * @brief 根据节点对应的子图的全局id，获取其参数在总数组中的起始位置
   * @param[in] id 节点对应的子图全局ID
   * @return 起始位置（未找到返回-1）
   */
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