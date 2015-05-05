/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor.cpp
 * @author  Richard Roberts
 * @date    Dec 8, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

#include <sstream>
#include <limits>

using namespace std;
using namespace boost::assign;
namespace br { using namespace boost::range; using namespace boost::adaptors; }

namespace gtsam {

/* ************************************************************************* */
string SlotEntry::toString() const {
  ostringstream oss;
  oss << "SlotEntry: slot=" << slot << ", dim=" << dimension;
  return oss.str();
}

/* ************************************************************************* */
Scatter::Scatter(const GaussianFactorGraph& gfg,
    boost::optional<const Ordering&> ordering) {
  gttic(Scatter_Constructor);
  static const size_t none = std::numeric_limits<size_t>::max();

  // First do the set union.
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, gfg) {
    if (factor) {
      for (GaussianFactor::const_iterator variable = factor->begin();
          variable != factor->end(); ++variable) {
        // TODO: Fix this hack to cope with zero-row Jacobians that come from BayesTreeOrphanWrappers
        const JacobianFactor* asJacobian =
            dynamic_cast<const JacobianFactor*>(factor.get());
        if (!asJacobian || asJacobian->cols() > 1)
          this->insert(
              make_pair(*variable, SlotEntry(none, factor->getDim(variable))));
      }
    }
  }

  // If we have an ordering, pre-fill the ordered variables first
  size_t slot = 0;
  if (ordering) {
    BOOST_FOREACH(Key key, *ordering) {
      const_iterator entry = find(key);
      if (entry == end())
        throw std::invalid_argument(
            "The ordering provided to the HessianFactor Scatter constructor\n"
                "contained extra variables that did not appear in the factors to combine.");
      at(key).slot = (slot++);
    }
  }

  // Next fill in the slot indices (we can only get these after doing the set
  // union.
  BOOST_FOREACH(value_type& var_slot, *this) {
    if (var_slot.second.slot == none)
      var_slot.second.slot = (slot++);
  }
}

/* ************************************************************************* */
HessianFactor::HessianFactor() :
                          info_(cref_list_of<1>(1))
{
  linearTerm().setZero();
  constantTerm() = 0.0;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j, const Matrix& G, const Vector& g, double f) :
                          GaussianFactor(cref_list_of<1>(j)), info_(cref_list_of<2>(G.cols())(1))
{
  if(G.rows() != G.cols() || G.rows() != g.size()) throw invalid_argument(
      "Attempting to construct HessianFactor with inconsistent matrix and/or vector dimensions");
  info_(0,0) = G;
  info_(0,1) = g;
  info_(1,1)(0,0) = f;
}

/* ************************************************************************* */
// error is 0.5*(x-mu)'*inv(Sigma)*(x-mu) = 0.5*(x'*G*x - 2*x'*G*mu + mu'*G*mu)
// where G = inv(Sigma), g = G*mu, f = mu'*G*mu = mu'*g
HessianFactor::HessianFactor(Key j, const Vector& mu, const Matrix& Sigma) :
                            GaussianFactor(cref_list_of<1>(j)),
                            info_(cref_list_of<2> (Sigma.cols()) (1) )
{
  if (Sigma.rows() != Sigma.cols() || Sigma.rows() != mu.size()) throw invalid_argument(
      "Attempting to construct HessianFactor with inconsistent matrix and/or vector dimensions");
  info_(0,0) = Sigma.inverse(); // G
  info_(0,1) = info_(0,0).selfadjointView() * mu; // g
  info_(1,1)(0,0) = mu.dot(info_(0,1).knownOffDiagonal().col(0)); // f
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j1, Key j2,
    const Matrix& G11, const Matrix& G12, const Vector& g1,
    const Matrix& G22, const Vector& g2, double f) :
                        GaussianFactor(cref_list_of<2>(j1)(j2)),
                        info_(cref_list_of<3> (G11.cols()) (G22.cols()) (1) )
{
  info_(0,0) = G11;
  info_(0,1) = G12;
  info_(0,2) = g1;
  info_(1,1) = G22;
  info_(1,2) = g2;
  info_(2,2)(0,0) = f;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j1, Key j2, Key j3,
    const Matrix& G11, const Matrix& G12, const Matrix& G13, const Vector& g1,
    const Matrix& G22, const Matrix& G23, const Vector& g2,
    const Matrix& G33, const Vector& g3, double f) :
                        GaussianFactor(cref_list_of<3>(j1)(j2)(j3)),
                        info_(cref_list_of<4> (G11.cols()) (G22.cols()) (G33.cols()) (1) )
{
  if(G11.rows() != G11.cols() || G11.rows() != G12.rows() || G11.rows() != G13.rows()  || G11.rows() != g1.size() ||
      G22.cols() != G12.cols() || G33.cols() != G13.cols() ||  G22.cols() != g2.size() || G33.cols() != g3.size())
    throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
  info_(0,0) = G11;
  info_(0,1) = G12;
  info_(0,2) = G13;
  info_(0,3) = g1;
  info_(1,1) = G22;
  info_(1,2) = G23;
  info_(1,3) = g2;
  info_(2,2) = G33;
  info_(2,3) = g3;
  info_(3,3)(0,0) = f;
}

/* ************************************************************************* */
namespace {
DenseIndex _getSizeHF(const Vector& m) { return m.size(); }
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const std::vector<Key>& js, const std::vector<Matrix>& Gs,
    const std::vector<Vector>& gs, double f) :
                        GaussianFactor(js), info_(gs | br::transformed(&_getSizeHF), true)
{
  // Get the number of variables
  size_t variable_count = js.size();

  // Verify the provided number of entries in the vectors are consistent
  if(gs.size() != variable_count || Gs.size() != (variable_count*(variable_count+1))/2)
    throw invalid_argument("Inconsistent number of entries between js, Gs, and gs in HessianFactor constructor.\nThe number of keys provided \
        in js must match the number of linear vector pieces in gs. The number of upper-diagonal blocks in Gs must be n*(n+1)/2");

  // Verify the dimensions of each provided matrix are consistent
  // Note: equations for calculating the indices derived from the "sum of an arithmetic sequence" formula
  for(size_t i = 0; i < variable_count; ++i){
    DenseIndex block_size = gs[i].size();
    // Check rows
    for(size_t j = 0; j < variable_count-i; ++j){
      size_t index = i*(2*variable_count - i + 1)/2 + j;
      if(Gs[index].rows() != block_size){
        throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
      }
    }
    // Check cols
    for(size_t j = 0; j <= i; ++j){
      size_t index = j*(2*variable_count - j + 1)/2 + (i-j);
      if(Gs[index].cols() != block_size){
        throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
      }
    }
  }

  // Fill in the blocks
  size_t index = 0;
  for(size_t i = 0; i < variable_count; ++i){
    for(size_t j = i; j < variable_count; ++j){
      info_(i, j) = Gs[index++];
    }
    info_(i, variable_count) = gs[i];
  }
  info_(variable_count, variable_count)(0,0) = f;
}

/* ************************************************************************* */
namespace {
void _FromJacobianHelper(const JacobianFactor& jf, SymmetricBlockMatrix& info)
{
  gttic(HessianFactor_fromJacobian);
  const SharedDiagonal& jfModel = jf.get_model();
  if(jfModel)
  {
    if(jf.get_model()->isConstrained())
      throw invalid_argument("Cannot construct HessianFactor from JacobianFactor with constrained noise model");
    info.full().triangularView() = jf.matrixObject().full().transpose() *
        (jfModel->invsigmas().array() * jfModel->invsigmas().array()).matrix().asDiagonal() *
        jf.matrixObject().full();
  } else {
    info.full().triangularView() = jf.matrixObject().full().transpose() * jf.matrixObject().full();
  }
}
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const JacobianFactor& jf) :
                          GaussianFactor(jf), info_(SymmetricBlockMatrix::LikeActiveViewOf(jf.matrixObject()))
{
  _FromJacobianHelper(jf, info_);
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const GaussianFactor& gf) :
                          GaussianFactor(gf)
{
  // Copy the matrix data depending on what type of factor we're copying from
  if(const JacobianFactor* jf = dynamic_cast<const JacobianFactor*>(&gf))
  {
    info_ = SymmetricBlockMatrix::LikeActiveViewOf(jf->matrixObject());
    _FromJacobianHelper(*jf, info_);
  }
  else if(const HessianFactor* hf = dynamic_cast<const HessianFactor*>(&gf))
  {
    info_ = hf->info_;
  }
  else
  {
    throw std::invalid_argument("In HessianFactor(const GaussianFactor& gf), gf is neither a JacobianFactor nor a HessianFactor");
  }
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const GaussianFactorGraph& factors,
    boost::optional<const Scatter&> scatter)
{
  gttic(HessianFactor_MergeConstructor);
  boost::optional<Scatter> computedScatter;
  if(!scatter) {
    computedScatter = Scatter(factors);
    scatter = computedScatter;
  }

  // Allocate and copy keys
  gttic(allocate);
  // Allocate with dimensions for each variable plus 1 at the end for the information vector
  keys_.resize(scatter->size());
  FastVector<DenseIndex> dims(scatter->size() + 1);
  BOOST_FOREACH(const Scatter::value_type& key_slotentry, *scatter) {
    keys_[key_slotentry.second.slot] = key_slotentry.first;
    dims[key_slotentry.second.slot] = key_slotentry.second.dimension;
  }
  dims.back() = 1;
  info_ = SymmetricBlockMatrix(dims);
  info_.full().triangularView().setZero();
  gttoc(allocate);

  // Form A' * A
  gttic(update);
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors)
  {
    if(factor) {
      if(const HessianFactor* hessian = dynamic_cast<const HessianFactor*>(factor.get()))
        updateATA(*hessian, *scatter);
      else if(const JacobianFactor* jacobian = dynamic_cast<const JacobianFactor*>(factor.get()))
        updateATA(*jacobian, *scatter);
      else
        throw invalid_argument("GaussianFactor is neither Hessian nor Jacobian");
    }
  }
  gttoc(update);
}

/* ************************************************************************* */
void HessianFactor::print(const std::string& s, const KeyFormatter& formatter) const {
  cout << s << "\n";
  cout << " keys: ";
  for(const_iterator key=this->begin(); key!=this->end(); ++key)
    cout << formatter(*key) << "(" << this->getDim(key) << ") ";
  cout << "\n";
  gtsam::print(Matrix(info_.full().selfadjointView()), "Augmented information matrix: ");
}

/* ************************************************************************* */
bool HessianFactor::equals(const GaussianFactor& lf, double tol) const {
  if(!dynamic_cast<const HessianFactor*>(&lf))
    return false;
  else {
    if(!Factor::equals(lf, tol))
      return false;
    Matrix thisMatrix = this->info_.full().selfadjointView();
    thisMatrix(thisMatrix.rows()-1, thisMatrix.cols()-1) = 0.0;
    Matrix rhsMatrix = static_cast<const HessianFactor&>(lf).info_.full().selfadjointView();
    rhsMatrix(rhsMatrix.rows()-1, rhsMatrix.cols()-1) = 0.0;
    return equal_with_abs_tol(thisMatrix, rhsMatrix, tol);
  }
}

/* ************************************************************************* */
Matrix HessianFactor::augmentedInformation() const
{
  return info_.full().selfadjointView();
}

/* ************************************************************************* */
Matrix HessianFactor::information() const
{
  return info_.range(0, this->size(), 0, this->size()).selfadjointView();
}

/* ************************************************************************* */
VectorValues HessianFactor::hessianDiagonal() const {
  VectorValues d;
  // Loop over all variables
  for (DenseIndex j = 0; j < (DenseIndex)size(); ++j) {
    // Get the diagonal block, and insert its diagonal
    Matrix B = info_(j, j).selfadjointView();
    d.insert(keys_[j],B.diagonal());
  }
  return d;
}

/* ************************************************************************* */
// TODO: currently assumes all variables of the same size 9 and keys arranged from 0 to n
void HessianFactor::hessianDiagonal(double* d) const {

  // Use eigen magic to access raw memory
  typedef Eigen::Matrix<double, 9, 1> DVector;
  typedef Eigen::Map<DVector> DMap;

  // Loop over all variables in the factor
  for (DenseIndex pos = 0; pos < (DenseIndex)size(); ++pos) {
    Key j = keys_[pos];
    // Get the diagonal block, and insert its diagonal
    const Matrix& B = info_(pos, pos).selfadjointView();
    DMap(d + 9 * j) += B.diagonal();
  }
}

/* ************************************************************************* */
map<Key,Matrix> HessianFactor::hessianBlockDiagonal() const {
  map<Key,Matrix> blocks;
  // Loop over all variables
  for (DenseIndex j = 0; j < (DenseIndex)size(); ++j) {
    // Get the diagonal block, and insert it
    Matrix B = info_(j, j).selfadjointView();
    blocks.insert(make_pair(keys_[j],B));
  }
  return blocks;
}

/* ************************************************************************* */
Matrix HessianFactor::augmentedJacobian() const
{
  return JacobianFactor(*this).augmentedJacobian();
}

/* ************************************************************************* */
std::pair<Matrix, Vector> HessianFactor::jacobian() const
{
  return JacobianFactor(*this).jacobian();
}

/* ************************************************************************* */
double HessianFactor::error(const VectorValues& c) const {
  // error 0.5*(f - 2*x'*g + x'*G*x)
  const double f = constantTerm();
  double xtg = 0, xGx = 0;
  // extract the relevant subset of the VectorValues
  // NOTE may not be as efficient
  const Vector x = c.vector(this->keys());
  xtg = x.dot(linearTerm());
  xGx = x.transpose() * info_.range(0, this->size(), 0, this->size()).selfadjointView() *  x;
  return 0.5 * (f - 2.0 * xtg +  xGx);
}

/* ************************************************************************* */
void HessianFactor::updateATA(const HessianFactor& update, const Scatter& scatter)
{
  gttic(updateATA);
  // This function updates 'combined' with the information in 'update'. 'scatter' maps variables in
  // the update factor to slots in the combined factor.

  // First build an array of slots
  gttic(slots);
  FastVector<DenseIndex> slots(update.size());
  DenseIndex slot = 0;
  BOOST_FOREACH(Key j, update) {
    slots[slot] = scatter.at(j).slot;
    ++ slot;
  }
  gttoc(slots);

  // Apply updates to the upper triangle
  gttic(update);
  size_t nrInfoBlocks = this->info_.nBlocks();
  for(DenseIndex j2=0; j2<update.info_.nBlocks(); ++j2) {
    DenseIndex slot2 = (j2 == (DenseIndex)update.size()) ? nrInfoBlocks-1 : slots[j2];
    for(DenseIndex j1=0; j1<=j2; ++j1) {
      DenseIndex slot1 = (j1 == (DenseIndex)update.size()) ? nrInfoBlocks-1 : slots[j1];
      info_(slot1, slot2) += update.info_(j1, j2);
    }
  }
  gttoc(update);
}

/* ************************************************************************* */
void HessianFactor::updateATA(const JacobianFactor& update, const Scatter& scatter) {

  // This function updates 'combined' with the information in 'update'.
  // 'scatter' maps variables in the update factor to slots in the combined
  // factor.

  gttic(updateATA);

  if(update.rows() > 0)
  {
    // First build an array of slots
    gttic(slots);
    FastVector<DenseIndex> slots(update.size());
    DenseIndex slot = 0;
    BOOST_FOREACH(Key j, update) {
      slots[slot] = scatter.at(j).slot;
      ++ slot;
    }
    gttoc(slots);

    gttic(whiten);
    // Whiten the factor if it has a noise model
    boost::optional<JacobianFactor> _whitenedFactor;
    const JacobianFactor* whitenedFactor;
    if(update.get_model())
    {
      if(update.get_model()->isConstrained())
        throw invalid_argument("Cannot update HessianFactor from JacobianFactor with constrained noise model");

      _whitenedFactor = update.whiten();
      whitenedFactor = &(*_whitenedFactor);
    }
    else
    {
      whitenedFactor = &update;
    }
    gttoc(whiten);

    const VerticalBlockMatrix& updateBlocks = whitenedFactor->matrixObject();

    // Apply updates to the upper triangle
    gttic(update);
    DenseIndex nrInfoBlocks = this->info_.nBlocks(), nrUpdateBlocks = updateBlocks.nBlocks();
    for(DenseIndex j2 = 0; j2 < nrUpdateBlocks; ++j2) { // Horizontal block of Hessian
      DenseIndex slot2 = (j2 == (DenseIndex)update.size()) ? nrInfoBlocks-1 : slots[j2];
      assert(slot2 >= 0 && slot2 <= nrInfoBlocks);
      for(DenseIndex j1 = 0; j1 <= j2; ++j1) { // Vertical block of Hessian
        DenseIndex slot1 = (j1 == (DenseIndex)update.size()) ? nrInfoBlocks-1 : slots[j1];
        assert(slot1 >= 0 && slot1 < nrInfoBlocks);
        if(slot1 != slot2)
          info_(slot1, slot2).knownOffDiagonal() += updateBlocks(j1).transpose() * updateBlocks(j2);
        else
          info_(slot1, slot2).selfadjointView().rankUpdate(updateBlocks(j1).transpose());
      }
    }
    gttoc(update);
  }
}

/* ************************************************************************* */
GaussianFactor::shared_ptr HessianFactor::negate() const
{
  shared_ptr result = boost::make_shared<This>(*this);
  result->info_.full().triangularView() = -result->info_.full().triangularView().nestedExpression(); // Negate the information matrix of the result
  return result;
}

/* ************************************************************************* */
void HessianFactor::multiplyHessianAdd(double alpha, const VectorValues& x,
    VectorValues& yvalues) const {

  // Create a vector of temporary y values, corresponding to rows i
  vector<Vector> y;
  y.reserve(size());
  for (const_iterator it = begin(); it != end(); it++)
    y.push_back(zero(getDim(it)));

  // Accessing the VectorValues one by one is expensive
  // So we will loop over columns to access x only once per column
  // And fill the above temporary y values, to be added into yvalues after
  for (DenseIndex j = 0; j < (DenseIndex)size(); ++j) {
    // xj is the input vector
    Vector xj = x.at(keys_[j]);
    DenseIndex i = 0;
    for (; i < j; ++i)
      y[i] += info_(i, j).knownOffDiagonal() * xj;
    // blocks on the diagonal are only half
    y[i] += info_(j, j).selfadjointView() * xj;
    // for below diagonal, we take transpose block from upper triangular part
    for (i = j + 1; i < (DenseIndex)size(); ++i)
      y[i] += info_(i, j).knownOffDiagonal() * xj;
  }

  // copy to yvalues
  for(DenseIndex i = 0; i < (DenseIndex)size(); ++i) {
	bool didNotExist;
    VectorValues::iterator it;
    boost::tie(it, didNotExist) = yvalues.tryInsert(keys_[i], Vector());
    if (didNotExist)
      it->second = alpha * y[i]; // init
    else
      it->second += alpha * y[i]; // add
  }
}

/* ************************************************************************* */
void HessianFactor::multiplyHessianAdd(double alpha, const double* x,
    double* yvalues, vector<size_t> offsets) const {

  // Use eigen magic to access raw memory
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> DVector;
  typedef Eigen::Map<DVector> DMap;
  typedef Eigen::Map<const DVector> ConstDMap;

  // Create a vector of temporary y values, corresponding to rows i
  vector<Vector> y;
  y.reserve(size());
  for (const_iterator it = begin(); it != end(); it++)
    y.push_back(zero(getDim(it)));

  // Accessing the VectorValues one by one is expensive
  // So we will loop over columns to access x only once per column
  // And fill the above temporary y values, to be added into yvalues after
  for (DenseIndex j = 0; j < (DenseIndex) size(); ++j) {
    DenseIndex i = 0;
    for (; i < j; ++i)
      y[i] += info_(i, j).knownOffDiagonal()
          * ConstDMap(x + offsets[keys_[j]],
              offsets[keys_[j] + 1] - offsets[keys_[j]]);
    // blocks on the diagonal are only half
    y[i] += info_(j, j).selfadjointView()
        * ConstDMap(x + offsets[keys_[j]],
            offsets[keys_[j] + 1] - offsets[keys_[j]]);
    // for below diagonal, we take transpose block from upper triangular part
    for (i = j + 1; i < (DenseIndex) size(); ++i)
      y[i] += info_(i, j).knownOffDiagonal()
          * ConstDMap(x + offsets[keys_[j]],
              offsets[keys_[j] + 1] - offsets[keys_[j]]);
  }

  // copy to yvalues
  for (DenseIndex i = 0; i < (DenseIndex) size(); ++i)
    DMap(yvalues + offsets[keys_[i]], offsets[keys_[i] + 1] - offsets[keys_[i]]) +=
        alpha * y[i];
}


/* ************************************************************************* */
VectorValues HessianFactor::gradientAtZero() const {
  VectorValues g;
  size_t n = size();
  for (size_t j = 0; j < n; ++j)
    g.insert(keys_[j], -info_(j,n).knownOffDiagonal());
  return g;
}

/* ************************************************************************* */
// TODO: currently assumes all variables of the same size 9 and keys arranged from 0 to n
void HessianFactor::gradientAtZero(double* d) const {

  // Use eigen magic to access raw memory
  typedef Eigen::Matrix<double, 9, 1> DVector;
  typedef Eigen::Map<DVector> DMap;

  // Loop over all variables in the factor
    for (DenseIndex pos = 0; pos < (DenseIndex)size(); ++pos) {
      Key j = keys_[pos];
      // Get the diagonal block, and insert its diagonal
      DVector dj =  -info_(pos,size()).knownOffDiagonal();
      DMap(d + 9 * j) += dj;
    }
}

/* ************************************************************************* */
std::pair<boost::shared_ptr<GaussianConditional>, boost::shared_ptr<HessianFactor> >
EliminateCholesky(const GaussianFactorGraph& factors, const Ordering& keys)
{
  gttic(EliminateCholesky);

  // Build joint factor
  HessianFactor::shared_ptr jointFactor;
  try {
    jointFactor = boost::make_shared<HessianFactor>(factors, Scatter(factors, keys));
  } catch(std::invalid_argument&) {
    throw InvalidDenseElimination(
        "EliminateCholesky was called with a request to eliminate variables that are not\n"
        "involved in the provided factors.");
  }

  // Do dense elimination
  GaussianConditional::shared_ptr conditional;
  try {
    VerticalBlockMatrix Ab = jointFactor->info_.choleskyPartial(keys.size());
    conditional = boost::make_shared<GaussianConditional>(jointFactor->keys(), keys.size(), Ab);
    // Erase the eliminated keys in the remaining factor
    jointFactor->keys_.erase(jointFactor->begin(), jointFactor->begin() + keys.size());
  } catch(CholeskyFailed&) {
    throw IndeterminantLinearSystemException(keys.front());
  }

  // Return result
  return make_pair(conditional, jointFactor);
}

/* ************************************************************************* */
std::pair<boost::shared_ptr<GaussianConditional>, boost::shared_ptr<GaussianFactor> >
EliminatePreferCholesky(const GaussianFactorGraph& factors, const Ordering& keys)
{
  gttic(EliminatePreferCholesky);

  // If any JacobianFactors have constrained noise models, we have to convert
  // all factors to JacobianFactors.  Otherwise, we can convert all factors
  // to HessianFactors.  This is because QR can handle constrained noise
  // models but Cholesky cannot.
  if (hasConstraints(factors))
    return EliminateQR(factors, keys);
  else
    return EliminateCholesky(factors, keys);
}

} // gtsam
