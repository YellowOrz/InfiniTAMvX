// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "LevenbergMarquardtMethod.h"
#include "../ORUtils/MathUtils.h"

#include <vector>
#include <math.h>

//#define DEBUG
//#include <stdio.h>

using namespace MiniSlamGraph;

static const double TR_QUALITY_GAMMA1 = 0.75;
static const double TR_QUALITY_GAMMA2 = 0.25;
static const double TR_REGION_INCREASE = 2.0;
static const double TR_REGION_DECREASE = 0.25;
static const double MIN_STEPLENGTH = 1e-6f;
static const int MAX_NUMBER_STEPS = 100;
static const double MIN_DECREASE = 1e-6f;

bool stepConsideredSmallMAX(const SlamGraphErrorFunction &f, const double *step) {
  double MAXnorm = 0.0;
  for (int i = 0; i < f.numParameters(); i++) {
    double tmp = fabs(step[i]);
    if (tmp > MAXnorm) MAXnorm = tmp;
  }

  return (MAXnorm < MIN_STEPLENGTH);
}

/**
 * @brief             评估优化结果的质量
 * @param[in] x       更新前的评估点
 * @param[in] x2      更新后的评估点
 * @param[in] step    参数的变化量，即△x
 * @param[in] grad    整张graph的梯度
 * @param[in] B       整张graph的二阶导
 * @return            质量分数 = 误差的平方和的实际减少量 / 预测减少量，后者 = Grad^T * step + 0.5 * step^T * H * step
 */
static inline double stepQuality(SlamGraphErrorFunction::EvaluationPoint *x,
    SlamGraphErrorFunction::EvaluationPoint *x2, const double *step, const double *grad, const Matrix *B) {
  int numPara = B->numRows();
  double actual_reduction = x->f() - x2->f();   // 误差的平方和的减少量
  double predicted_reduction = 0.0;             // 预测的误差的平方和的减少量， = Grad^T * step + 0.5 * step^T * H * step
  double *tmp = new double[numPara];
  B->multiply(step, tmp);
  for (int i = 0; i < numPara; i++) {
    predicted_reduction -= grad[i] * step[i] + 0.5 * step[i] * tmp[i];
  }
  delete[] tmp;
  return actual_reduction / fabs(predicted_reduction);
}

int LevenbergMarquardtMethod::minimize(
    const SlamGraphErrorFunction &f, SlamGraphErrorFunction::Parameters &initialization) {
  int ret = 0;
  int numPara = f.numParameters();  // 优化的参数量，其实就是节点数量
  std::vector<double> d(numPara);   // 参数的变化量，即△x，《SLAM十四讲》公式6.34
  double lambda = 0.01;             // 优化的步长
  int step_counter = 0;             // 优化的步数

  SlamGraphErrorFunction::EvaluationPoint *x = f.evaluateAt(initialization.clone());  // 初始的评估点，内部会计算误差
  SlamGraphErrorFunction::EvaluationPoint *x2 = NULL;                                 // 更新后的评估点
  initialization.clear();
  //! 确保初始误差不是无穷大
  if (!portable_finite((float) x->f())) {
    delete x;
    return -1;
  }
  //! 不停的优化迭代
  do {
#ifdef DEBUG  // debug output
    fprintf(stderr, "step number: %i\n", step_counter);
    //x->getParameter().print();
    fprintf(stderr, "function value: %f\n", x->f());
    fprintf(stderr, "LM: lambda %f\n", lambda);
#endif
    const double *grad = x->nabla_f();    // 获取整张graph的梯度    //? 迭代的过程中会更新梯度吗？
    const Matrix *B = x->hessian_GN();    // 获取整张graph的二阶导

    bool success;
    { //! 求解方程
      Matrix *A = B->clone(); // TODO: 为啥要用指针？是为了用完马上delete？但都用花括号括起来了，不用指针也能马上释放啊
      /*if (regularize_sphere) A->addDiagonal(lambda);
      else*/ A->multDiagonal(lambda);   // 对角元素增大lambda，因为(H+λI)△x_k=g 《SLAM十四讲》公式6.37
      success = A->solve(grad, &(d[0]));
      delete A;
    }

    //! 更新步长
    if (success) {            // 求解成功
      if (stepConsideredSmallMAX(f, &(d[0]))) break;      // 步长太小，说明收敛，退出
      // 应用step，更新EvaluationPoint。make step
      for (int i = 0; i < numPara; i++) d[i] = -d[i];     // 因为要下降，所以取负数
      SlamGraphErrorFunction::Parameters *tmp_para = x->getParameter().clone(); // 更新后的EvaluationPoint
      f.applyDelta(x->getParameter(), &(d[0]), *tmp_para);                      // 更新所有节点的参数

      // 检查更新后的误差，然后再算个lambada。check whether step reduces error function and compute a new value of lambda
      x2 = f.evaluateAt(tmp_para);                        // 再次计算误差
      double q = stepQuality(x, x2, &(d[0]), grad, B);    // 评估优化结果的质量 = 误差的平方和的 实际减少量 / 预测减少量
      if (q > TR_QUALITY_GAMMA1) {                        // 实际减少量 > 预测的75%，质量好，步长减少一半
        // very successful step
        success = true;
        lambda = lambda / TR_REGION_INCREASE;
      } else if (q > TR_QUALITY_GAMMA2) {                 // 实际减少量 为预测的25%-75%，步长不变
        // kind of successful step
        success = true;
        //lambda = lambda; //lambda doesn't change
      } else {                                            // 实际减少量 < 预测的25%，质量差，步长增加4倍
        // step failed
        success = false;
        lambda = lambda / TR_REGION_DECREASE;
      }
    } else {                // 求解方程失败，步长增加4倍
      x2 = NULL;
      // can't compute a step quality here...
      lambda = lambda / TR_REGION_DECREASE;
    }
    //! 判断是否继续迭代 && 释放内存
    if (success) {
      // accept step
#ifdef DEBUG
      fprintf(stderr, "accept %p (new function value: %f)\n", (void*)x2, x2->f());
#endif

      bool continueIteration = true;
      // did the function decrease sufficiently?
      // 误差的平方和 几乎没啥变化的时候就不继续迭代了
      if (!(fabs(x->f()) * MIN_DECREASE < (x->f() - x2->f()))) continueIteration = false;

      delete x;
      x = x2;

      if (!continueIteration) break;
    } else {
#ifdef DEBUG
      if (x2 != NULL) fprintf(stderr, "reject %p (function value would increase to: %f)\n", (void*)x2, x2->f());
      else fprintf(stderr, "reject (could not solve for new parameters)\n");
#endif
      if (x2 != NULL) delete x2;
    }
    // C allows a nice syntax with ->, --> and even ++>
    // ...just mentioned to make bored programmers happy
    //! 限制最大迭代次数
    if (step_counter++ >= MAX_NUMBER_STEPS) break;
  } while (1);
  //! 计算完成后再把数据拷贝出来
  initialization.copyFrom(x->getParameter());
  delete x;

#ifdef DEBUG
  fprintf(stderr, "total number of steps: %i\n", step_counter);
#endif
  return ret;
}

