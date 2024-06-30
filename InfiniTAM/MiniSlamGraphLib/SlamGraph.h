// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>
#include <vector>

#include "GraphNode.h"
#include "GraphEdge.h"
#include "ParameterIndex.h"
#include "VariableLengthVector.h"
#include "SparseBlockMatrix.h"

namespace MiniSlamGraph {
class SlamGraph {
 public:
  typedef std::map<int, GraphNode *> NodeIndex; // 节点和它对应子图id的map
  typedef std::vector<GraphEdge *> EdgeList;

  static NodeIndex cloneNodeIndex(const NodeIndex &src);
  static void clearNodeIndex(NodeIndex &src);

  virtual ~SlamGraph(void);

  void addNode(GraphNode *node);
  void addEdge(GraphEdge *edge);

  const NodeIndex &getNodeIndex(void) const { return mNodes; }
  void setNodeIndex(const NodeIndex &src);

  /**
   * @brief 记录所有节点的变量维度，为后续的优化做准备
   * @note  其实都是6，因为只有SE3一种节点。在evaluateF()之前调用
   * Before any calls to evaluateF() or related functions, the evaluations have to be initialized with prepareEvaluations(). This will internally assign the parameters of all nodes to places in the gradient vector and hessian matrix.
   */
  void prepareEvaluations(void);
  const ParameterIndex &getParameters(void) const {
    return mParameterIndex;
  }

  virtual double evaluateF(const NodeIndex *override_nodes = NULL) const;
  virtual void evaluateGradientAndHessian(VariableLengthVector *&g, SparseBlockMatrix *&H,
                                          const NodeIndex *override_nodes = NULL) const;

protected:
  /** This function is internally called by evaluateGradientAndHessian()
      and is supposed to allocate the structures for the gradient vector
      and, most crucially, the sparse Hessian matrix.
  */
  virtual void allocateGradientAndHessian(VariableLengthVector *&g, SparseBlockMatrix *&H) const = 0;

 private:
  NodeIndex mNodes;   // 位姿图中所有的节点。数据结构为map，记录子图id和节点本身信息
  EdgeList mEdges;    // 位姿图中所有的边。数据结构为vector

  ParameterIndex mParameterIndex;   // 记录每个节点的参数量（其实就是6，因为只有SE3一种节点）
};
}

