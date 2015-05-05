/**
 * @file   NonlinearOptimizerParams.cpp
 * @brief  Parameters for nonlinear optimization
 * @date   Jul 24, 2012
 * @author Yong-Dian Jian
 * @author Richard Roberts
 * @author Frank Dellaert
 */

#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <boost/algorithm/string.hpp>

namespace gtsam {

/* ************************************************************************* */
NonlinearOptimizerParams::Verbosity NonlinearOptimizerParams::verbosityTranslator(
    const std::string &src) const {
  std::string s = src;
  boost::algorithm::to_upper(s);
  if (s == "SILENT")
    return NonlinearOptimizerParams::SILENT;
  if (s == "ERROR")
    return NonlinearOptimizerParams::ERROR;
  if (s == "VALUES")
    return NonlinearOptimizerParams::VALUES;
  if (s == "DELTA")
    return NonlinearOptimizerParams::DELTA;
  if (s == "LINEAR")
    return NonlinearOptimizerParams::LINEAR;
  if (s == "TERMINATION")
    return NonlinearOptimizerParams::TERMINATION;

  /* default is silent */
  return NonlinearOptimizerParams::SILENT;
}

/* ************************************************************************* */
std::string NonlinearOptimizerParams::verbosityTranslator(
    Verbosity value) const {
  std::string s;
  switch (value) {
  case NonlinearOptimizerParams::SILENT:
    s = "SILENT";
    break;
  case NonlinearOptimizerParams::TERMINATION:
    s = "TERMINATION";
    break;
  case NonlinearOptimizerParams::ERROR:
    s = "ERROR";
    break;
  case NonlinearOptimizerParams::VALUES:
    s = "VALUES";
    break;
  case NonlinearOptimizerParams::DELTA:
    s = "DELTA";
    break;
  case NonlinearOptimizerParams::LINEAR:
    s = "LINEAR";
    break;
  default:
    s = "UNDEFINED";
    break;
  }
  return s;
}

/* ************************************************************************* */
void NonlinearOptimizerParams::setIterativeParams(
    const SubgraphSolverParameters &params) {
  iterativeParams = boost::make_shared<SubgraphSolverParameters>(params);
}

/* ************************************************************************* */
void NonlinearOptimizerParams::print(const std::string& str) const {

  //NonlinearOptimizerParams::print(str);
  std::cout << str << "\n";
  std::cout << "relative decrease threshold: " << relativeErrorTol << "\n";
  std::cout << "absolute decrease threshold: " << absoluteErrorTol << "\n";
  std::cout << "      total error threshold: " << errorTol << "\n";
  std::cout << "         maximum iterations: " << maxIterations << "\n";
  std::cout << "                  verbosity: " << verbosityTranslator(verbosity)
      << "\n";
  std::cout.flush();

  switch (linearSolverType) {
  case MULTIFRONTAL_CHOLESKY:
    std::cout << "         linear solver type: MULTIFRONTAL CHOLESKY\n";
    break;
  case MULTIFRONTAL_QR:
    std::cout << "         linear solver type: MULTIFRONTAL QR\n";
    break;
  case SEQUENTIAL_CHOLESKY:
    std::cout << "         linear solver type: SEQUENTIAL CHOLESKY\n";
    break;
  case SEQUENTIAL_QR:
    std::cout << "         linear solver type: SEQUENTIAL QR\n";
    break;
  case CHOLMOD:
    std::cout << "         linear solver type: CHOLMOD\n";
    break;
  case CONJUGATE_GRADIENT:
    std::cout << "         linear solver type: CONJUGATE GRADIENT\n";
    break;
  default:
    std::cout << "         linear solver type: (invalid)\n";
    break;
  }

  if (ordering)
    std::cout << "                   ordering: custom\n";
  else
    std::cout << "                   ordering: COLAMD\n";

  std::cout.flush();
}

/* ************************************************************************* */
std::string NonlinearOptimizerParams::linearSolverTranslator(
    LinearSolverType linearSolverType) const {
  switch (linearSolverType) {
  case MULTIFRONTAL_CHOLESKY:
    return "MULTIFRONTAL_CHOLESKY";
  case MULTIFRONTAL_QR:
    return "MULTIFRONTAL_QR";
  case SEQUENTIAL_CHOLESKY:
    return "SEQUENTIAL_CHOLESKY";
  case SEQUENTIAL_QR:
    return "SEQUENTIAL_QR";
  case CONJUGATE_GRADIENT:
    return "CONJUGATE_GRADIENT";
  case CHOLMOD:
    return "CHOLMOD";
  default:
    throw std::invalid_argument(
        "Unknown linear solver type in SuccessiveLinearizationOptimizer");
  }
}

/* ************************************************************************* */
NonlinearOptimizerParams::LinearSolverType NonlinearOptimizerParams::linearSolverTranslator(
    const std::string& linearSolverType) const {
  if (linearSolverType == "MULTIFRONTAL_CHOLESKY")
    return MULTIFRONTAL_CHOLESKY;
  if (linearSolverType == "MULTIFRONTAL_QR")
    return MULTIFRONTAL_QR;
  if (linearSolverType == "SEQUENTIAL_CHOLESKY")
    return SEQUENTIAL_CHOLESKY;
  if (linearSolverType == "SEQUENTIAL_QR")
    return SEQUENTIAL_QR;
  if (linearSolverType == "CONJUGATE_GRADIENT")
    return CONJUGATE_GRADIENT;
  if (linearSolverType == "CHOLMOD")
    return CHOLMOD;
  throw std::invalid_argument(
      "Unknown linear solver type in SuccessiveLinearizationOptimizer");
}
/* ************************************************************************* */

} // namespace
