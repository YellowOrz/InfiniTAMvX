// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "MatrixWrapper.h"
#include "SlamGraph.h"

namespace MiniSlamGraph {
/** 误差优化函数 */
class SlamGraphErrorFunction /*: public K_OPTIM::ErrorFunctionLeastSquares*/
{
 public:
  /** 优化参数，其实就只有位姿图的节点 */ // TODO: 下次从这儿开始
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
    SlamGraph::NodeIndex mNodes;  // 位姿图的所有节点
  };
  
  class EvaluationPoint /*: public K_OPTIM::ErrorFunctionLeastSquares::EvaluationPoint*/ {
   public:
    EvaluationPoint(const SlamGraphErrorFunction *parent, Parameters *para);
    ~EvaluationPoint(void);

    double f(void);
    const double *nabla_f(void);
    const Matrix *hessian_GN(void);
    const Parameters &getParameter(void) const { return *mPara; }

   private:
    void cacheGH(void);

    const SlamGraphErrorFunction *mParent;
    const Parameters *mPara;
    double cacheF;
    VariableLengthVector *cacheG;
    Matrix *cacheH;
  };

  SlamGraphErrorFunction(const SlamGraph &graph);

  ~SlamGraphErrorFunction(void);

  int numParameters(void) const;

  EvaluationPoint *evaluateAt(/*K_OPTIM::Optimization*/Parameters *para) const;

  void applyDelta(const /*K_OPTIM::Optimization*/Parameters &para_old,
                  const double *delta, /*K_OPTIM::Optimization*/
                  Parameters &para_new) const;

  //Matrix_CSparse::Pattern* & getHessianSparsityPattern(void)
  void *&getHessianSparsityPattern(void) {
    return mSparsityPattern;
  }

  const SlamGraph *getGraph(void) const {
    return mGraph;
  }

 private:
  const SlamGraph *mGraph;
  void *mSparsityPattern;
};
}

