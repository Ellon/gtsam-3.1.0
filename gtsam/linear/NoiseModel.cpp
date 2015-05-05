/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NoiseModel.cpp
 * @date Jan 13, 2010
 * @author Richard Roberts
 * @author Frank Dellaert
 */

#include <limits>
#include <iostream>
#include <typeinfo>
#include <stdexcept>

#include <boost/foreach.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include <gtsam/base/timing.h>
#include <gtsam/linear/NoiseModel.h>

static double inf = std::numeric_limits<double>::infinity();
using namespace std;

namespace gtsam {
namespace noiseModel {

/* ************************************************************************* */
// update A, b
// A' \define A_{S}-ar and b'\define b-ad
// Linear algebra: takes away projection on latest orthogonal
// Graph: make a new factor on the separator S
// __attribute__ ((noinline))  // uncomment to prevent inlining when profiling
template<class MATRIX>
void updateAb(MATRIX& Ab, int j, const Vector& a, const Vector& rd) {
  size_t n = Ab.cols()-1;
  Ab.middleCols(j+1,n-j) -= a * rd.segment(j+1, n-j).transpose();
}

/* ************************************************************************* */
// check *above the diagonal* for non-zero entries
boost::optional<Vector> checkIfDiagonal(const Matrix M) {
  size_t m = M.rows(), n = M.cols();
  // check all non-diagonal entries
  bool full = false;
  size_t i, j;
  for (i = 0; i < m; i++)
    if (!full)
      for (j = i + 1; j < n; j++)
        if (fabs(M(i, j)) > 1e-9) {
          full = true;
          break;
        }
  if (full) {
    return boost::none;
  } else {
    Vector diagonal(n);
    for (j = 0; j < n; j++)
      diagonal(j) = M(j, j);
    return diagonal;
  }
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::SqrtInformation(const Matrix& R, bool smart) {
  size_t m = R.rows(), n = R.cols();
  if (m != n)
    throw invalid_argument("Gaussian::SqrtInformation: R not square");
  boost::optional<Vector> diagonal = boost::none;
  if (smart)
    diagonal = checkIfDiagonal(R);
  if (diagonal)
    return Diagonal::Sigmas(reciprocal(*diagonal), true);
  else
    return shared_ptr(new Gaussian(R.rows(), R));
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Information(const Matrix& M, bool smart) {
  size_t m = M.rows(), n = M.cols();
  if (m != n)
    throw invalid_argument("Gaussian::Information: R not square");
  boost::optional<Vector> diagonal = boost::none;
  if (smart)
    diagonal = checkIfDiagonal(M);
  if (diagonal)
    return Diagonal::Precisions(*diagonal, true);
  else {
    Matrix R = RtR(M);
    return shared_ptr(new Gaussian(R.rows(), R));
  }
}

/* ************************************************************************* */
Gaussian::shared_ptr Gaussian::Covariance(const Matrix& covariance,
    bool smart) {
  size_t m = covariance.rows(), n = covariance.cols();
  if (m != n)
    throw invalid_argument("Gaussian::Covariance: covariance not square");
  boost::optional<Vector> variances = boost::none;
  if (smart)
    variances = checkIfDiagonal(covariance);
  if (variances)
    return Diagonal::Variances(*variances, true);
  else
    return shared_ptr(new Gaussian(n, inverse_square_root(covariance)));
}

/* ************************************************************************* */
void Gaussian::print(const string& name) const {
  gtsam::print(thisR(), "Gaussian");
}

/* ************************************************************************* */
bool Gaussian::equals(const Base& expected, double tol) const {
  const Gaussian* p = dynamic_cast<const Gaussian*> (&expected);
  if (p == NULL) return false;
  if (typeid(*this) != typeid(*p)) return false;
  //if (!sqrt_information_) return true; // ALEX todo;
  return equal_with_abs_tol(R(), p->R(), sqrt(tol));
}

/* ************************************************************************* */
Vector Gaussian::whiten(const Vector& v) const {
  return thisR() * v;
}

/* ************************************************************************* */
Vector Gaussian::unwhiten(const Vector& v) const {
  return backSubstituteUpper(thisR(), v);
}

/* ************************************************************************* */
double Gaussian::Mahalanobis(const Vector& v) const {
  // Note: for Diagonal, which does ediv_, will be correct for constraints
  Vector w = whiten(v);
  return w.dot(w);
}

/* ************************************************************************* */
Matrix Gaussian::Whiten(const Matrix& H) const {
  return thisR() * H;
}

/* ************************************************************************* */
void Gaussian::WhitenInPlace(Matrix& H) const {
  H = thisR() * H;
}

/* ************************************************************************* */
void Gaussian::WhitenInPlace(Eigen::Block<Matrix> H) const {
  H = thisR() * H;
}

/* ************************************************************************* */
// General QR, see also special version in Constrained
SharedDiagonal Gaussian::QR(Matrix& Ab) const {

  gttic(Gaussian_noise_model_QR);

  static const bool debug = false;

  // get size(A) and maxRank
  // TODO: really no rank problems ?
  // size_t m = Ab.rows(), n = Ab.cols()-1;
  // size_t maxRank = min(m,n);

  // pre-whiten everything (cheaply if possible)
  WhitenInPlace(Ab);

  if(debug) gtsam::print(Ab, "Whitened Ab: ");

  // Eigen QR - much faster than older householder approach
  inplace_QR(Ab);

  // hand-coded householder implementation
  // TODO: necessary to isolate last column?
  // householder(Ab, maxRank);

  return SharedDiagonal();
}

void Gaussian::WhitenSystem(vector<Matrix>& A, Vector& b) const {
  BOOST_FOREACH(Matrix& Aj, A) { WhitenInPlace(Aj); }
  whitenInPlace(b);
}

void Gaussian::WhitenSystem(Matrix& A, Vector& b) const {
  WhitenInPlace(A);
  whitenInPlace(b);
}

void Gaussian::WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const {
  WhitenInPlace(A1);
  WhitenInPlace(A2);
  whitenInPlace(b);
}

void Gaussian::WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const{
  WhitenInPlace(A1);
  WhitenInPlace(A2);
  WhitenInPlace(A3);
  whitenInPlace(b);
}

/* ************************************************************************* */
// Diagonal
/* ************************************************************************* */
Diagonal::Diagonal() :
    Gaussian(1) // TODO: Frank asks: really sure about this?
{
}

/* ************************************************************************* */
Diagonal::Diagonal(const Vector& sigmas) :
    Gaussian(sigmas.size()), sigmas_(sigmas), invsigmas_(reciprocal(sigmas)), precisions_(
        emul(invsigmas_, invsigmas_)) {
}

/* ************************************************************************* */
Diagonal::shared_ptr Diagonal::Variances(const Vector& variances, bool smart) {
  if (smart) {
    // check whether all the same entry
    size_t n = variances.size();
    for (size_t j = 1; j < n; j++)
      if (variances(j) != variances(0)) goto full;
    return Isotropic::Variance(n, variances(0), true);
  }
  full: return shared_ptr(new Diagonal(esqrt(variances)));
}

/* ************************************************************************* */
Diagonal::shared_ptr Diagonal::Sigmas(const Vector& sigmas, bool smart) {
  if (smart) {
    size_t n = sigmas.size();
    if (n==0) goto full;
    // look for zeros to make a constraint
    for (size_t j=0; j< n; ++j)
      if (sigmas(j)<1e-8)
        return Constrained::MixedSigmas(sigmas);
    // check whether all the same entry
    for (size_t j = 1; j < n; j++)
      if (sigmas(j) != sigmas(0)) goto full;
    return Isotropic::Sigma(n, sigmas(0), true);
  }
  full: return Diagonal::shared_ptr(new Diagonal(sigmas));
}

/* ************************************************************************* */
void Diagonal::print(const string& name) const {
  gtsam::print(sigmas_, name + "diagonal sigmas");
}

/* ************************************************************************* */
Vector Diagonal::whiten(const Vector& v) const {
  return emul(v, invsigmas());
}

/* ************************************************************************* */
Vector Diagonal::unwhiten(const Vector& v) const {
  return emul(v, sigmas_);
}

/* ************************************************************************* */
Matrix Diagonal::Whiten(const Matrix& H) const {
  return vector_scale(invsigmas(), H);
}

/* ************************************************************************* */
void Diagonal::WhitenInPlace(Matrix& H) const {
  vector_scale_inplace(invsigmas(), H);
}

/* ************************************************************************* */
void Diagonal::WhitenInPlace(Eigen::Block<Matrix> H) const {
  H = invsigmas().asDiagonal() * H;
}

/* ************************************************************************* */
// Constrained
/* ************************************************************************* */

/* ************************************************************************* */
Constrained::Constrained(const Vector& sigmas)
  : Diagonal(sigmas), mu_(repeat(sigmas.size(), 1000.0)) {
  for (int i=0; i<sigmas.size(); ++i) {
    if (!std::isfinite(1./sigmas(i))) {
      precisions_(i) = 0.0; // Set to finite value
      invsigmas_(i) = 0.0;
    }
  }
}

/* ************************************************************************* */
Constrained::Constrained(const Vector& mu, const Vector& sigmas)
  : Diagonal(sigmas), mu_(mu) {
//  assert(sigmas.size() == mu.size());
  for (int i=0; i<sigmas.size(); ++i) {
    if (!std::isfinite(1./sigmas(i))) {
      precisions_(i) = 0.0; // Set to finite value
      invsigmas_(i) = 0.0;
    }
  }
}

/* ************************************************************************* */
Constrained::shared_ptr Constrained::MixedSigmas(const Vector& mu, const Vector& sigmas, bool smart) {
  // FIXME: can't return a diagonal shared_ptr due to conversion
//  if (smart) {
//    // look for zeros to make a constraint
//    for (size_t i=0; i< (size_t) sigmas.size(); ++i)
//      if (sigmas(i)<1e-8)
//        return MixedSigmas(mu, sigmas);
//    return Diagonal::Sigmas(sigmas);
//  }
  return shared_ptr(new Constrained(mu, sigmas));
}

/* ************************************************************************* */
void Constrained::print(const std::string& name) const {
  gtsam::print(sigmas_, name + "constrained sigmas");
  gtsam::print(mu_, name + "constrained mu");
}

/* ************************************************************************* */
Vector Constrained::whiten(const Vector& v) const {
  // If sigmas[i] is not 0 then divide v[i] by sigmas[i], as usually done in
  // other normal Gaussian noise model. Otherwise, sigmas[i] = 0 indicating
  // a hard constraint, we don't do anything.
  const Vector& a = v;
  const Vector& b = sigmas_;
  // Now allows for whiten augmented vector with a new additional part coming
  // from the Lagrange multiplier. So a.size() >= b.size()
  Vector c = a;
  for( DenseIndex i = 0; i < b.size(); i++ ) {
    const double& ai = a(i), &bi = b(i);
    if (bi!=0) c(i) = ai/bi;
  }
  return c;
}

/* ************************************************************************* */
double Constrained::distance(const Vector& v) const {
  Vector w = Diagonal::whiten(v); // get noisemodel for constrained elements
  // TODO Find a better way of doing these checks
  for (size_t i=0; i<dim_; ++i)  // add mu weights on constrained variables
    if (!std::isfinite(1./sigmas_[i])) // whiten makes constrained variables zero
      w[i] = v[i] * sqrt(mu_[i]); // TODO: may want to store sqrt rather than rebuild
  return w.dot(w);
}

/* ************************************************************************* */
Matrix Constrained::Whiten(const Matrix& H) const {
  // selective scaling
  // Now allow augmented Matrix with a new additional part coming
  // from the Lagrange multiplier.
  Matrix M(H.block(0, 0, dim(), H.cols()));
  Constrained::WhitenInPlace(M);
  return M;
}

/* ************************************************************************* */
void Constrained::WhitenInPlace(Matrix& H) const {
  // selective scaling
  // Scale row i of H by sigmas[i], basically multiplying H with diag(sigmas)
  // Set inf_mask flag is true so that if invsigmas[i] is inf, i.e. sigmas[i] = 0,
  // indicating a hard constraint, we leave H's row i in place.
  // Now allow augmented Matrix with a new additional part coming
  // from the Lagrange multiplier.
//  Inlined: vector_scale_inplace(invsigmas(), H, true);
  // vector_scale_inplace(v, A, true);
  for (DenseIndex i=0; i<(DenseIndex)dim_; ++i) {
    const double& invsigma = invsigmas_(i);
    if (std::isfinite(1./sigmas_(i)))
      H.row(i) *= invsigma;
  }
}

/* ************************************************************************* */
void Constrained::WhitenInPlace(Eigen::Block<Matrix> H) const {
  // selective scaling
  // Scale row i of H by sigmas[i], basically multiplying H with diag(sigmas)
  // Set inf_mask flag is true so that if invsigmas[i] is inf, i.e. sigmas[i] = 0,
  // indicating a hard constraint, we leave H's row i in place.
  const Vector& _invsigmas = invsigmas();
  for(DenseIndex row = 0; row < _invsigmas.size(); ++row)
    if(isfinite(_invsigmas(row)))
      H.row(row) *= _invsigmas(row);
}

/* ************************************************************************* */
Constrained::shared_ptr Constrained::unit(size_t augmentedDim) const {
  Vector sigmas = ones(dim()+augmentedDim);
  for (size_t i=0; i<dim(); ++i)
    if (this->sigmas_(i) == 0.0)
      sigmas(i) = 0.0;
  Vector augmentedMu = zero(dim()+augmentedDim);
  subInsert(augmentedMu, mu_, 0);
  return MixedSigmas(augmentedMu, sigmas);
}

/* ************************************************************************* */
// Special version of QR for Constrained calls slower but smarter code
// that deals with possibly zero sigmas
// It is Gram-Schmidt orthogonalization rather than Householder
// Previously Diagonal::QR
SharedDiagonal Constrained::QR(Matrix& Ab) const {
  bool verbose = false;
  if (verbose) cout << "\nStarting Constrained::QR" << endl;

  // get size(A) and maxRank
  size_t m = Ab.rows(), n = Ab.cols()-1;
  size_t maxRank = min(m,n);

  // create storage for [R d]
  typedef boost::tuple<size_t, Vector, double> Triple;
  list<Triple> Rd;

  Vector pseudo(m); // allocate storage for pseudo-inverse
  Vector invsigmas = reciprocal(sigmas_);
  Vector weights = emul(invsigmas,invsigmas); // calculate weights once

  // We loop over all columns, because the columns that can be eliminated
  // are not necessarily contiguous. For each one, estimate the corresponding
  // scalar variable x as d-rS, with S the separator (remaining columns).
  // Then update A and b by substituting x with d-rS, zero-ing out x's column.
  for (size_t j=0; j<n; ++j) {
    // extract the first column of A
    Vector a = Ab.col(j);

    // Calculate weighted pseudo-inverse and corresponding precision
    gttic(constrained_QR_weightedPseudoinverse);
    double precision = weightedPseudoinverse(a, weights, pseudo);
    gttoc(constrained_QR_weightedPseudoinverse);

    // If precision is zero, no information on this column
    // This is actually not limited to constraints, could happen in Gaussian::QR
    // In that case, we're probably hosed. TODO: make sure Householder is rank-revealing
    if (precision < 1e-8) continue;

    gttic(constrained_QR_create_rd);
    // create solution [r d], rhs is automatically r(n)
    Vector rd(n+1); // uninitialized !
    rd(j)=1.0; // put 1 on diagonal
    for (size_t j2=j+1; j2<n+1; ++j2) // and fill in remainder with dot-products
      rd(j2) = pseudo.dot(Ab.col(j2));
    gttoc(constrained_QR_create_rd);

    // construct solution (r, d, sigma)
    Rd.push_back(boost::make_tuple(j, rd, precision));

    // exit after rank exhausted
    if (Rd.size()>=maxRank) break;

    // update Ab, expensive, using outer product
    gttic(constrained_QR_update_Ab);
    Ab.middleCols(j+1,n-j) -= a * rd.segment(j+1, n-j).transpose();
    gttoc(constrained_QR_update_Ab);
  }

  // Create storage for precisions
  Vector precisions(Rd.size());

  gttic(constrained_QR_write_back_into_Ab);
  // Write back result in Ab, imperative as we are
  // TODO: test that is correct if a column was skipped !!!!
  size_t i = 0; // start with first row
  bool mixed = false;
  BOOST_FOREACH(const Triple& t, Rd) {
    const size_t& j  = t.get<0>();
    const Vector& rd = t.get<1>();
    precisions(i)    = t.get<2>();
    if (precisions(i)==inf) mixed = true;
    for (size_t j2=0; j2<j; ++j2)
      Ab(i,j2) = 0.0; // fill in zeros below diagonal anway
    for (size_t j2=j; j2<n+1; ++j2)
      Ab(i,j2) = rd(j2);
    i+=1;
  }
  gttoc(constrained_QR_write_back_into_Ab);

  // Must include mu, as the defaults might be higher, resulting in non-convergence
  return mixed ? Constrained::MixedPrecisions(mu_, precisions) : Diagonal::Precisions(precisions);
}

/* ************************************************************************* */
// Isotropic
/* ************************************************************************* */
Isotropic::shared_ptr Isotropic::Sigma(size_t dim, double sigma, bool smart)  {
  if (smart && fabs(sigma-1.0)<1e-9) return Unit::Create(dim);
  return shared_ptr(new Isotropic(dim, sigma));
}

/* ************************************************************************* */
Isotropic::shared_ptr Isotropic::Variance(size_t dim, double variance, bool smart)  {
  if (smart && fabs(variance-1.0)<1e-9) return Unit::Create(dim);
  return shared_ptr(new Isotropic(dim, sqrt(variance)));
}

/* ************************************************************************* */
void Isotropic::print(const string& name) const {
  cout << name << "isotropic sigma " << " " << sigma_ << endl;
}

/* ************************************************************************* */
double Isotropic::Mahalanobis(const Vector& v) const {
  return v.dot(v) * invsigma_ * invsigma_;
}

/* ************************************************************************* */
Vector Isotropic::whiten(const Vector& v) const {
  return v * invsigma_;
}

/* ************************************************************************* */
Vector Isotropic::unwhiten(const Vector& v) const {
  return v * sigma_;
}

/* ************************************************************************* */
Matrix Isotropic::Whiten(const Matrix& H) const {
  return invsigma_ * H;
}

/* ************************************************************************* */
void Isotropic::WhitenInPlace(Matrix& H) const {
  H *= invsigma_;
}

/* ************************************************************************* */
void Isotropic::WhitenInPlace(Eigen::Block<Matrix> H) const {
  H *= invsigma_;
}

/* ************************************************************************* */
// Unit
/* ************************************************************************* */
void Unit::print(const std::string& name) const {
  cout << name << "unit (" << dim_ << ") " << endl;
}

/* ************************************************************************* */
// M-Estimator
/* ************************************************************************* */

namespace mEstimator {

/** produce a weight vector according to an error vector and the implemented
 * robust function */
Vector Base::weight(const Vector &error) const {
  const size_t n = error.rows();
  Vector w(n);
  for ( size_t i = 0 ; i < n ; ++i )
    w(i) = weight(error(i));
  return w;
}

/** square root version of the weight function */
Vector Base::sqrtWeight(const Vector &error) const {
  const size_t n = error.rows();
  Vector w(n);
  for ( size_t i = 0 ; i < n ; ++i )
    w(i) = sqrtWeight(error(i));
  return w;
}


/** The following three functions reweight block matrices and a vector
 * according to their weight implementation */

void Base::reweight(Vector& error) const {
  if(reweight_ == Block) {
    const double w = sqrtWeight(error.norm());
    error *= w;
  } else {
    const Vector w = sqrtWeight(error);
    error.array() *= w.array();
  }
}

/** Reweight n block matrices with one error vector */
void Base::reweight(vector<Matrix> &A, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    BOOST_FOREACH(Matrix& Aj, A) {
      Aj *= w;
    }
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    BOOST_FOREACH(Matrix& Aj, A) {
      vector_scale_inplace(W,Aj);
    }
    error = emul(W, error);
  }
}

/** Reweight one block matrix with one error vector */
void Base::reweight(Matrix &A, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A);
    error = emul(W, error);
  }
}

/** Reweight two block matrix with one error vector */
void Base::reweight(Matrix &A1, Matrix &A2, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A1 *= w;
    A2 *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A1);
    vector_scale_inplace(W,A2);
    error = emul(W, error);
  }
}

/** Reweight three block matrix with one error vector */
void Base::reweight(Matrix &A1, Matrix &A2, Matrix &A3, Vector &error) const {
  if ( reweight_ == Block ) {
    const double w = sqrtWeight(error.norm());
    A1 *= w;
    A2 *= w;
    A3 *= w;
    error *= w;
  }
  else {
    const Vector W = sqrtWeight(error);
    vector_scale_inplace(W,A1);
    vector_scale_inplace(W,A2);
    vector_scale_inplace(W,A3);
    error = emul(W, error);
  }
}

/* ************************************************************************* */
// Null model
/* ************************************************************************* */

void Null::print(const std::string &s="") const
{ cout << s << "null ()" << endl; }

Null::shared_ptr Null::Create()
{ return shared_ptr(new Null()); }

Fair::Fair(const double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
  if ( c_ <= 0 ) {
    cout << "mEstimator Fair takes only positive double in constructor. forced to 1.0" << endl;
    c_ = 1.0;
  }
}

/* ************************************************************************* */
// Fair
/* ************************************************************************* */

double Fair::weight(const double &error) const
{ return 1.0 / (1.0 + fabs(error)/c_); }

void Fair::print(const std::string &s="") const
{ cout << s << "fair (" << c_ << ")" << endl; }

bool Fair::equals(const Base &expected, const double tol) const {
  const Fair* p = dynamic_cast<const Fair*> (&expected);
  if (p == NULL) return false;
  return fabs(c_ - p->c_ ) < tol;
}

Fair::shared_ptr Fair::Create(const double c, const ReweightScheme reweight)
{ return shared_ptr(new Fair(c, reweight)); }

/* ************************************************************************* */
// Huber
/* ************************************************************************* */

Huber::Huber(const double k, const ReweightScheme reweight)
  : Base(reweight), k_(k) {
  if ( k_ <= 0 ) {
    cout << "mEstimator Huber takes only positive double in constructor. forced to 1.0" << endl;
    k_ = 1.0;
  }
}

double Huber::weight(const double &error) const {
  return (error < k_) ? (1.0) : (k_ / fabs(error));
}

void Huber::print(const std::string &s="") const {
  cout << s << "huber (" << k_ << ")" << endl;
}

bool Huber::equals(const Base &expected, const double tol) const {
  const Huber* p = dynamic_cast<const Huber*>(&expected);
  if (p == NULL) return false;
  return fabs(k_ - p->k_) < tol;
}

Huber::shared_ptr Huber::Create(const double c, const ReweightScheme reweight) {
  return shared_ptr(new Huber(c, reweight));
}

/* ************************************************************************* */
// Cauchy
/* ************************************************************************* */

Cauchy::Cauchy(const double k, const ReweightScheme reweight)
  : Base(reweight), k_(k) {
  if ( k_ <= 0 ) {
    cout << "mEstimator Cauchy takes only positive double in constructor. forced to 1.0" << endl;
    k_ = 1.0;
  }
}

double Cauchy::weight(const double &error) const {
  return k_*k_ / (k_*k_ + error*error);
}

void Cauchy::print(const std::string &s="") const {
  cout << s << "cauchy (" << k_ << ")" << endl;
}

bool Cauchy::equals(const Base &expected, const double tol) const {
  const Cauchy* p = dynamic_cast<const Cauchy*>(&expected);
  if (p == NULL) return false;
  return fabs(k_ - p->k_) < tol;
}

Cauchy::shared_ptr Cauchy::Create(const double c, const ReweightScheme reweight) {
  return shared_ptr(new Cauchy(c, reweight));
}

/* ************************************************************************* */
// Tukey
/* ************************************************************************* */
Tukey::Tukey(const double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
}

double Tukey::weight(const double &error) const {
  if (fabs(error) <= c_) {
    double xc2 = (error/c_)*(error/c_);
    double one_xc22 = (1.0-xc2)*(1.0-xc2);
    return one_xc22;
  }
  return 0.0;
}

void Tukey::print(const std::string &s="") const {
  std::cout << s << ": Tukey (" << c_ << ")" << std::endl;
}

bool Tukey::equals(const Base &expected, const double tol) const {
  const Tukey* p = dynamic_cast<const Tukey*>(&expected);
  if (p == NULL) return false;
  return fabs(c_ - p->c_) < tol;
}

Tukey::shared_ptr Tukey::Create(const double c, const ReweightScheme reweight) {
  return shared_ptr(new Tukey(c, reweight));
}

/* ************************************************************************* */
// Welsh
/* ************************************************************************* */
Welsh::Welsh(const double c, const ReweightScheme reweight)
  : Base(reweight), c_(c) {
}

double Welsh::weight(const double &error) const {
  double xc2 = (error/c_)*(error/c_);
  return std::exp(-xc2);
}

void Welsh::print(const std::string &s="") const {
  std::cout << s << ": Welsh (" << c_ << ")" << std::endl;
}

bool Welsh::equals(const Base &expected, const double tol) const {
  const Welsh* p = dynamic_cast<const Welsh*>(&expected);
  if (p == NULL) return false;
  return fabs(c_ - p->c_) < tol;
}

Welsh::shared_ptr Welsh::Create(const double c, const ReweightScheme reweight) {
  return shared_ptr(new Welsh(c, reweight));
}

} // namespace mEstimator

/* ************************************************************************* */
// Robust
/* ************************************************************************* */

void Robust::print(const std::string& name) const {
  robust_->print(name);
  noise_->print(name);
}

bool Robust::equals(const Base& expected, double tol) const {
  const Robust* p = dynamic_cast<const Robust*> (&expected);
  if (p == NULL) return false;
  return noise_->equals(*p->noise_,tol) && robust_->equals(*p->robust_,tol);
}

void Robust::WhitenSystem(Vector& b) const {
  noise_->whitenInPlace(b);
  robust_->reweight(b);
}

void Robust::WhitenSystem(vector<Matrix>& A, Vector& b) const {
  noise_->WhitenSystem(A,b);
  robust_->reweight(A,b);
}

void Robust::WhitenSystem(Matrix& A, Vector& b) const {
  noise_->WhitenSystem(A,b);
  robust_->reweight(A,b);
}

void Robust::WhitenSystem(Matrix& A1, Matrix& A2, Vector& b) const {
  noise_->WhitenSystem(A1,A2,b);
  robust_->reweight(A1,A2,b);
}

void Robust::WhitenSystem(Matrix& A1, Matrix& A2, Matrix& A3, Vector& b) const{
  noise_->WhitenSystem(A1,A2,A3,b);
  robust_->reweight(A1,A2,A3,b);
}

Robust::shared_ptr Robust::Create(
  const RobustModel::shared_ptr &robust, const NoiseModel::shared_ptr noise){
  return shared_ptr(new Robust(robust,noise));
}

/* ************************************************************************* */

}
} // gtsam
