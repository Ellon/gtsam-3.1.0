/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once


#include <gtsam/linear/ConjugateGradientSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/inference/Ordering.h>

#include <boost/tuple/tuple.hpp>

namespace gtsam {

  // Forward declarations
  class GaussianFactorGraph;
  class GaussianBayesNet;

class GTSAM_EXPORT SubgraphSolverParameters : public ConjugateGradientParameters {
public:
  typedef ConjugateGradientParameters Base;
  SubgraphSolverParameters() : Base() {}
  virtual void print() const { Base::print(); }
};

/**
 * This class implements the SPCG solver presented in Dellaert et al in IROS'10.
 *
 * Given a linear least-squares problem \f$ f(x) = |A x - b|^2 \f$. We split the problem into
 * \f$ f(x) = |A_t - b_t|^2 + |A_c - b_c|^2 \f$ where \f$ A_t \f$ denotes the "tree" part, and \f$ A_c \f$ denotes the "constraint" part.
 * \f$ A_t \f$ is factorized into \f$ Q_t R_t \f$, and we compute \f$ c_t = Q_t^{-1} b_t \f$, and \f$ x_t = R_t^{-1} c_t \f$ accordingly.
 * Then we solve a reparametrized problem \f$ f(y) = |y|^2 + |A_c R_t^{-1} y - \bar{b_y}|^2 \f$, where \f$ y = R_t(x - x_t) \f$, and \f$ \bar{b_y} = (b_c - A_c x_t) \f$
 *
 * In the matrix form, it is equivalent to solving \f$ [A_c R_t^{-1} ; I ] y = [\bar{b_y} ; 0] \f$. We can solve it
 * with the least-squares variation of the conjugate gradient method.
 *
 * To use it in nonlinear optimization, please see the following example
 *
 *  LevenbergMarquardtParams parameters;
 *  parameters.linearSolverType = NonlinearOptimizerParams::CONJUGATE_GRADIENT;
 *  parameters.iterativeParams = boost::make_shared<SubgraphSolverParameters>();
 *  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, parameters);
 *  Values result = optimizer.optimize();
 *
 * \nosubgrouping
 */

class GTSAM_EXPORT SubgraphSolver : public IterativeSolver {

public:
  typedef SubgraphSolverParameters Parameters;

protected:
  Parameters parameters_;
  Ordering ordering_;
  SubgraphPreconditioner::shared_ptr pc_;  ///< preconditioner object

public:
  /* Given a gaussian factor graph, split it into a spanning tree (A1) + others (A2) for SPCG */
  SubgraphSolver(const GaussianFactorGraph &A, const Parameters &parameters, const Ordering& ordering);
  SubgraphSolver(const boost::shared_ptr<GaussianFactorGraph> &A, const Parameters &parameters, const Ordering& ordering);

  /* The user specify the subgraph part and the constraint part, may throw exception if A1 is underdetermined */
  SubgraphSolver(const GaussianFactorGraph &Ab1, const GaussianFactorGraph &Ab2, const Parameters &parameters, const Ordering& ordering);
  SubgraphSolver(const boost::shared_ptr<GaussianFactorGraph> &Ab1, const boost::shared_ptr<GaussianFactorGraph> &Ab2, const Parameters &parameters, const Ordering& ordering);

  /* The same as above, but the A1 is solved before */
  SubgraphSolver(const boost::shared_ptr<GaussianBayesNet> &Rc1, const GaussianFactorGraph &Ab2, const Parameters &parameters, const Ordering& ordering);
  SubgraphSolver(const boost::shared_ptr<GaussianBayesNet> &Rc1, const boost::shared_ptr<GaussianFactorGraph> &Ab2, const Parameters &parameters, const Ordering& ordering);

  virtual ~SubgraphSolver() {}
  virtual VectorValues optimize () ;
  virtual VectorValues optimize (const VectorValues &initial) ;

protected:

  void initialize(const GaussianFactorGraph &jfg);
  void initialize(const boost::shared_ptr<GaussianBayesNet> &Rc1, const boost::shared_ptr<GaussianFactorGraph> &Ab2);

  boost::tuple<boost::shared_ptr<GaussianFactorGraph>, boost::shared_ptr<GaussianFactorGraph> >
  splitGraph(const GaussianFactorGraph &gfg) ;
};

} // namespace gtsam
