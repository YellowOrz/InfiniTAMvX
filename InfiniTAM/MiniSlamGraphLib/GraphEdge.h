// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>
#include <map>

#include "GraphNode.h"
#include "ParameterIndex.h"
#include "VariableLengthVector.h"
#include "SparseBlockMatrix.h"

namespace MiniSlamGraph {
/** 位姿图中的边 */
class GraphEdge {
 public:
  typedef std::map<int, GraphNode *> NodeIndex; // 所有节点信息， 从 节点对应的子图的全局id 到 节点指针 的映射

  virtual ~GraphEdge(void) {}
  /** 起始节点对应的子图的全局id */
  int fromNodeId(void) const { return idFrom; }
  /** 设置 起始节点对应的子图的全局id */
  void setFromNodeId(int id) { idFrom = id; }
  /** 终止节点对应的子图的全局id */
  int toNodeId(void) const { return idTo; }
  /** 设置 终止节点对应的子图的全局id */
  void setToNodeId(int id) { idTo = id; }
  /** 观测的维度 */
virtual int getMeasureDimensions(void) const = 0;
  virtual void setMeasurement(const double *v) = 0;
  virtual void getMeasurement(double *v) const = 0;

  /**
   * @brief 计算当前边（即约束）的误差向量
   * @details This method is supposed to compute the residual vector of the
      Edge/Contraint. The result will be written to a vector of length
      getMeasureDimensions().
   * @param[in] nodes   所有的节点。会根据成员变量 @p idFrom 和 @p idTo 找到相关的节点。
   * @param[out] dest   误差向量。具体维度由getMeasureDimensions()决定。但是在这里就是误差矩阵的MQT形式（四元数+平移向量）
   */
  virtual void computeResidualVector(const NodeIndex &nodes, double *dest) const = 0;

  /**
   * @brief 计算指定节点的一阶导矩阵（雅可比矩阵）
   * @details This method is supposed to compute the Jacobian of the Edge/Contraint. Here the "Jacobian" means the
   * derivative of the residual vector from computeResidualVector() w.r.t. the parameters of the Node with id @p id. The
   * result is stored in a row-major "matrix" @p j, where each row contains the derivatives of a single entry of the
   * residual vector w.r.t. all the parameters. This means the derivative of residual @p i_r w.r.t. parameter @p i_p is
   * addressed as @p j[i_r * dimMeasure + i_p] .
   * If the node identified py @p id does not affect the residual, @p j is untouched and the return value is set to
   * false. In all other cases, the return value is set to true.
   * @param[in] nodes   所有的节点
   * @param[in] id      对应的子图的全局id
   * @param[out] j      一阶导矩阵（雅可比矩阵），维度=dimMeasure*numPara
   * @return            是否存在一阶导
   */
  virtual bool computeJacobian(const NodeIndex &nodes, int id, double *j) const = 0;

  /**
   * @brief 计算当前边（即约束）的误差
   * @details This method computes the contribution of an Edge/Constraint to the overall error function. It comes with a
   * default implementation using the simple squared differences.
   * @param[in] nodes   所有的节点。
   * @return            误差的平方
   * @note              内部会调用computeResidualVector()来计算误差向量，然后计算误差向量的模长，即为误差
   */
  virtual double computeError(const NodeIndex &nodes) const;

  /**
   * @brief 计算当前边（即约束）的梯度和二阶导（H矩阵）
   * @details This method computes the contribution of an Edge/Constraint to the overall gradient vector and hessian
   * matrix. It comes with a default implementation suitable for simple squared differences.
   * @param[in] nodes       所有节点
   * @param[in] index       指定节点信息
   * @param[out] gradient   所有edge的梯度
   * @param[out] hessian    所有edge的二阶导（一个大的稀疏矩阵矩阵，对角线上每个block是每条边的H矩阵）
   */
  virtual void computeGradientAndHessian(const NodeIndex &nodes, const ParameterIndex &index,
                                         VariableLengthVector &gradient, SparseBlockMatrix &hessian) const;

private:
  int idFrom;   // 起始节点对应的子图的全局id
  int idTo;     // 终止节点对应的子图的全局id
};

}

