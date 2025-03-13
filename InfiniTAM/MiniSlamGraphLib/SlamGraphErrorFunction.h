// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MatrixWrapper.h"
#include "SlamGraph.h"

namespace MiniSlamGraph {
/** 误差优化函数 */ // ? 为啥要搞成这么复杂，SlamGraphErrorFunction，里面包了Parameters和EvaluationPoint两个类，但是又没有相关的成员变量
class SlamGraphErrorFunction /*: public K_OPTIM::ErrorFunctionLeastSquares*/ {
 public:
  /** 所有节点的优化参数 */
  class Parameters/* : public K_OPTIM::OptimizationParameter */ {
   public:
    Parameters(const SlamGraph &graph);
    Parameters(const Parameters &src);
    ~Parameters(void);
    Parameters *clone(void) const { return new Parameters(*this); }
    void copyFrom(const /*Optimization*/Parameters &_src);
    //void copyValuesFrom(const OptimizationParameter & src);
    void clear(void);

    const SlamGraph::NodeIndex &getNodes(void) const {
      return mNodes;
    }
    SlamGraph::NodeIndex &getNodes(void) {
      return mNodes;
    }

   private:
    SlamGraph::NodeIndex mNodes;  // 位姿图的所有节点，根据节点的id索引
  };
  /** 用于计算优化问题中的目标函数、误差、导数 */
  class EvaluationPoint /*: public K_OPTIM::ErrorFunctionLeastSquares::EvaluationPoint*/ {
   public:
    EvaluationPoint(const SlamGraphErrorFunction *parent, Parameters *para);
    ~EvaluationPoint(void);
    /** 得到所有边的误差的平方和。误差在构造函数里就算好了 */
    double f(void);
    /**
     * @brief 获取整张graph的梯度
     * @return  整张graph的梯度。每个edge的梯度首尾相连成一个很长的向量
     * @note    获取之前会看看要不要计算
     */
    const double *nabla_f(void);
    /**
     * @brief 获取整张graph的二阶导
     * @return  整张graph的二阶导。其实类型是MatrixSymPosDef
     * @note    获取之前会看看要不要计算
     */
    const Matrix *hessian_GN(void);
    const Parameters &getParameter(void) const { return *mPara; }

   private:
    /** 计算整张graph的梯度和二阶导 */
    void cacheGH(void);

    const SlamGraphErrorFunction *mParent;  // 误差优化函数。包含了
    const Parameters *mPara;                // 优化参数，其实就只有位姿图的节点
    double cacheF;                          // 所有边的误差的平方和
    VariableLengthVector *cacheG;           // 整张graph的梯度。每个edge的梯度首尾相连成一个很长的向量
    Matrix *cacheH;                         // 整张graph的二阶导。其实类型是MatrixSymPosDef
  };

  SlamGraphErrorFunction(const SlamGraph &graph);

  ~SlamGraphErrorFunction(void);

  int numParameters(void) const;
  /**
   * @brief 初始化 评估点，并计算所有边的误差
   * @param[in] para  优化参数
   * @return          评估点，记录了误差
   */
  EvaluationPoint *evaluateAt(/*K_OPTIM::Optimization*/Parameters *para) const;
  /**
   * @brief               更新参数
   * @param[in] para_old  所有节点的旧的优化参数
   * @param[in] delta     所有节点的参数的变化量
   * @param[out] para_new 新的优化参数
   * @note                需要保证para_new和para_old的节点数量一致
   */
  void applyDelta(const /*K_OPTIM::Optimization*/Parameters &para_old, const double *delta,
      /*K_OPTIM::Optimization*/ Parameters &para_new) const;

  //Matrix_CSparse::Pattern* & getHessianSparsityPattern(void)
  void *&getHessianSparsityPattern(void) {
    return mSparsityPattern;
  }

  const SlamGraph *getGraph(void) const {
    return mGraph;
  }

 private:
  const SlamGraph *mGraph;  // 位姿图
  void *mSparsityPattern;   // ?稀疏矩阵的存储结构
};
}

