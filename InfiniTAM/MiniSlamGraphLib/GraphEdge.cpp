// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "GraphEdge.h"

using namespace MiniSlamGraph;
/**
 * @brief                 计算对单个节点的 梯度 和 二阶导
 * @param[in] residual    误差向量
 * @param[in] jacobian    一阶导矩阵，维度dimMeasure*numPara，以行存储
 * @param[in] dimMeasure  观测的维度
 * @param[in] numPara     变量数量
 * @param[out] Gblock     梯度向量，维度numPara*1
 * @param[out] Hblock     二阶导，维度numPara*numPara，以行存储
 */
static void jacobianToHessian_diagonalpart(
    double *residual, double *jacobian, int dimMeasure, int numPara, double *Gblock, double *Hblock) {
  for (int para = 0; para < numPara; ++para) {
    //! 计算梯度向量的每个元素
    Gblock[para] = 0.0f;
    for (int idx = 0; idx < dimMeasure; ++idx) {
      // Gblock[row] += jacobian[row*dimMeasure+idx] * residual[idx];
      Gblock[para] += jacobian[idx * numPara + para] * residual[idx];
    }
    //! 计算二阶导矩阵的上三角：H(i,j)=J的i列和j列的内积
    for (int para2 = para; para2 < numPara; ++para2) {
      Hblock[para * numPara + para2] = 0.0f;
      // for (int idx = 0; idx < dimMeasure; ++idx) Hblock[row*numPara+col] += jacobian[row*dimMeasure+idx] *
      // jacobian[col*dimMeasure+idx];
      for (int idx = 0; idx < dimMeasure; ++idx)
        Hblock[para * numPara + para2] += jacobian[idx * numPara + para] * jacobian[idx * numPara + para2];
    }
  }
  //! 二阶导矩阵的下三角对称一下就好
  for (int row = 0; row < numPara; ++row) {
    for (int col = 0; col < row; ++col) {
      Hblock[row * numPara + col] = Hblock[col * numPara + row];
    }
  }
}

/**
 * @brief                     计算对2个节点的 二阶导
 * @param[in] jacobian_from   起始节点的一阶导矩阵，维度dimMeasure*numPara_from，以行存储
 * @param[in] jacobian_to     终止节点的一阶导矩阵，维度dimMeasure*numPara_to，以行存储
 * @param[in] dimMeasure      观测的维度
 * @param[in] numPara_from    起始节点的变量数量
 * @param[in] numPara_to      终止节点的变量数量
 * @param[out] Hblock         二阶导，维度numPara_from*numPara_to，以行存储
 */
static void jacobianToHessian_offdiagonal(
    double *jacobian_from, double *jacobian_to, int dimMeasure, int numPara_from, int numPara_to, double *Hblock) {
  for (int row = 0; row < numPara_from; ++row) {
    for (int col = 0; col < numPara_to; ++col) {
      Hblock[row * numPara_to + col] = 0.0f;
      // for (int idx = 0; idx < dimMeasure; ++idx)
      //  Hblock[row*numPara_to+col] += jacobian_from[row*dimMeasure+idx] * jacobian_to[col*dimMeasure+idx];
      for (int idx = 0; idx < dimMeasure; ++idx)  // H的r行j列=J_f的r列和J_t的c列的内积
        Hblock[row * numPara_to + col] += jacobian_from[idx * numPara_from + row] * jacobian_to[idx * numPara_to + col];
    }
  }
}

double GraphEdge::computeError(const GraphEdge::NodeIndex &nodes) const {
  int dim = getMeasureDimensions();
  std::vector<double> residual(dim);
  computeResidualVector(nodes, &(residual[0]));  // 计算误差向量

  // TODO: information matrix
  double ret = 0.0f;
  for (int i = 0; i < dim; ++i) ret += residual[i] * residual[i];  // 误差向量的平方和即为误差

  return 0.5f * ret;  // 视觉slam十四讲 公式6.35 中也乘了0.5,但其实去掉这个0.5对结果没有影响
}

void GraphEdge::computeGradientAndHessian(const GraphEdge::NodeIndex &nodes, const ParameterIndex &index,
    VariableLengthVector &gradient, SparseBlockMatrix &hessian) const {
  //! 准备
  int id_from = fromNodeId();            // 起始节点对应的子图的全局id
  int id_to = toNodeId();                // 终止节点对应的子图的全局id
  int row_f = index.findIndex(id_from);  // 起始节点的参数量
  int row_t = index.findIndex(id_to);    // 终止节点的参数量
  // fprintf(stderr, "grad and hessian for edge from %i to %i (rows %i %i)\n", id_from, id_to, row_f, row_t);

  GraphNode *fromNode = nodes.find(id_from)->second;  // 起始节点
  GraphNode *toNode = nodes.find(id_to)->second;      // 终止节点
  int numPara_from = fromNode->numParameters();       // 起始节点的变量维度，就是row_f
  int numPara_to = toNode->numParameters();           // 终止节点的变量维度，就是row_t
  int dimMeasure = getMeasureDimensions();            // 观测的维度

  // TODO: "Measurement Matrix"
  //! 计算误差向量
  std::vector<double> residual(dimMeasure); // 误差向量，维度=观测的维度（即dimMeasure）
  computeResidualVector(nodes, &(residual[0]));

  //! 计算当前edge中两个节点的一阶导
  std::vector<double> jacobian_from(dimMeasure * numPara_from);
  std::vector<double> jacobian_to(dimMeasure * numPara_to);
  bool do_from = (row_f >= 0);                                        //? 什么情况会导致false？
  bool do_to = (row_t >= 0);
  if (do_from) computeJacobian(nodes, id_from, &(jacobian_from[0]));
  if (do_to) computeJacobian(nodes, id_to, &(jacobian_to[0]));

  // NOTE: 误差e对edge中的两个节点(f和t)求二阶导，总共有四个二阶导数，可以看作是一个大矩阵分块成2*2，即   //? 这是拿H矩阵做信息矩阵吗？？？
  // H11 = ∂e/(∂f∂f)  H12 = ∂e/(∂f∂t)
  // H21 = ∂e/(∂t∂f)  H22 = ∂e/(∂t∂t)
  //! 计算对角线的梯度和二阶导，即对单个节点求导
  if (do_from) {  // deal with "from" node
    std::vector<double> Hblock_diag_f(numPara_from * numPara_from);
    std::vector<double> Gblock_f(numPara_from); // NOTE: 当目标函数为标量函数时，Jacobian矩阵是梯度向量
    jacobianToHessian_diagonalpart(
        &(residual[0]), &(jacobian_from[0]), dimMeasure, numPara_from, &(Gblock_f[0]), &(Hblock_diag_f[0]));

    gradient.addData(row_f, numPara_from, &(Gblock_f[0]));                            // 记录梯度
    hessian.addBlock(row_f, row_f, numPara_from, numPara_from, &(Hblock_diag_f[0]));  // 记录二阶导
  }
  if (do_to) {  // deal with "to" node
    std::vector<double> Hblock_diag_t(numPara_to * numPara_to);
    std::vector<double> Gblock_t(numPara_to);

    jacobianToHessian_diagonalpart(
        &(residual[0]), &(jacobian_to[0]), dimMeasure, numPara_to, &(Gblock_t[0]), &(Hblock_diag_t[0]));

    gradient.addData(row_t, numPara_to, &(Gblock_t[0]));
    hessian.addBlock(row_t, row_t, numPara_to, numPara_to, &(Hblock_diag_t[0]));
  }
  //! 计算非对角线的二阶导矩阵，即对edge的两个节点都求导
  if (do_from && do_to) { // off diagonal part
    std::vector<double> Hblock_off(numPara_from * numPara_to);
    jacobianToHessian_offdiagonal(
        &(jacobian_from[0]), &(jacobian_to[0]), dimMeasure, numPara_from, numPara_to, &(Hblock_off[0]));

    hessian.addBlock(row_f, row_t, numPara_from, numPara_to, &(Hblock_off[0]));
    hessian.addBlockTranspose(row_t, row_f, numPara_to, numPara_from, &(Hblock_off[0]));
  }
}