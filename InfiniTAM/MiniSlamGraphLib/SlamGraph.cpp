// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "SlamGraph.h"

#include "SparseRegularBlockMatrix.h"

using namespace MiniSlamGraph;

SlamGraph::~SlamGraph(void) {
  for (EdgeList::iterator it = mEdges.begin(); it != mEdges.end(); ++it) delete *it;
  for (NodeIndex::iterator it = mNodes.begin(); it != mNodes.end(); ++it) delete it->second;
}

SlamGraph::NodeIndex SlamGraph::cloneNodeIndex(const SlamGraph::NodeIndex &src) { // TODO：把返回弄成引用，减少拷贝
  NodeIndex ret;
  for (NodeIndex::const_iterator it = src.begin(); it != src.end(); ++it)
    ret[it->first] = it->second->clone();
  return ret;
}

void SlamGraph::clearNodeIndex(SlamGraph::NodeIndex &src) {
  for (NodeIndex::const_iterator it = src.begin(); it != src.end(); ++it) delete it->second;
  src.clear();
}

void SlamGraph::setNodeIndex(const NodeIndex &src) {
  clearNodeIndex(mNodes);
  mNodes = cloneNodeIndex(src);
}

void SlamGraph::addNode(GraphNode *node) {
  mNodes[node->getId()] = node;
}

void SlamGraph::addEdge(GraphEdge *edge) {
  mEdges.push_back(edge);
}

void SlamGraph::prepareEvaluations(void) {
  // 遍历图中每个节点
  for (NodeIndex::const_iterator it = mNodes.begin(); it != mNodes.end(); ++it) {
    // 跳过固定的节点
    if (it->second->isFixed()) continue;
    // 获取每个节点的维度（其实就是6，因为只有SE3一种节点）
    int num = it->second->numParameters();
    mParameterIndex.addIndex(it->first, num);
  }
}

double SlamGraph::evaluateF(const NodeIndex *nodes) const {
  if (nodes == NULL) nodes = &mNodes;

  double ret = 0.0f;
  for (EdgeList::const_iterator it = mEdges.begin(); it != mEdges.end(); ++it) {
    ret += (*it)->computeError(*nodes);
  }
  return ret;
}

void SlamGraph::evaluateGradientAndHessian(VariableLengthVector *&g,
                                           SparseBlockMatrix *&H,
                                           const NodeIndex *nodes) const {
  if (nodes == NULL) nodes = &mNodes;

  allocateGradientAndHessian(g, H);

  for (EdgeList::const_iterator it = mEdges.begin(); it != mEdges.end(); ++it) {
    (*it)->computeGradientAndHessian(*nodes, mParameterIndex, *g, *H);
  }

  g->setOverallSize(mParameterIndex.numTotalParameters());
}

