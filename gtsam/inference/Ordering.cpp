/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.cpp
 * @author  Richard Roberts
 * @date    Sep 2, 2010
 */

#include <vector>
#include <limits>

#include <boost/format.hpp>

#include <gtsam/inference/Ordering.h>
#include <gtsam/3rdparty/CCOLAMD/Include/ccolamd.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  FastMap<Key, size_t> Ordering::invert() const
  {
    FastMap<Key, size_t> inverted;
    for(size_t pos = 0; pos < this->size(); ++pos)
      inverted.insert(make_pair((*this)[pos], pos));
    return inverted;
  }

  /* ************************************************************************* */
  Ordering Ordering::COLAMD(const VariableIndex& variableIndex)
  {
    // Call constrained version with all groups set to zero
    vector<int> dummy_groups(variableIndex.size(), 0);
    return Ordering::COLAMDConstrained(variableIndex, dummy_groups);
  }

  /* ************************************************************************* */
  Ordering Ordering::COLAMDConstrained(
    const VariableIndex& variableIndex, std::vector<int>& cmember)
  {
    gttic(Ordering_COLAMDConstrained);

    gttic(Prepare);
    size_t nEntries = variableIndex.nEntries(), nFactors = variableIndex.nFactors(), nVars = variableIndex.size();
    // Convert to compressed column major format colamd wants it in (== MATLAB format!)
    size_t Alen = ccolamd_recommended((int)nEntries, (int)nFactors, (int)nVars); /* colamd arg 3: size of the array A */
    vector<int> A = vector<int>(Alen); /* colamd arg 4: row indices of A, of size Alen */
    vector<int> p = vector<int>(nVars + 1); /* colamd arg 5: column pointers of A, of size n_col+1 */

    // Fill in input data for COLAMD
    p[0] = 0;
    int count = 0;
    vector<Key> keys(nVars); // Array to store the keys in the order we add them so we can retrieve them in permuted order
    size_t index = 0;
    BOOST_FOREACH(const VariableIndex::value_type key_factors, variableIndex) {
      // Arrange factor indices into COLAMD format
      const VariableIndex::Factors& column = key_factors.second;
      size_t lastFactorId = numeric_limits<size_t>::max();
      BOOST_FOREACH(size_t factorIndex, column) {
        if(lastFactorId != numeric_limits<size_t>::max())
          assert(factorIndex > lastFactorId);
        A[count++] = (int)factorIndex; // copy sparse column
      }
      p[index+1] = count; // column j (base 1) goes from A[j-1] to A[j]-1
      // Store key in array and increment index
      keys[index] = key_factors.first;
      ++ index;
    }

    assert((size_t)count == variableIndex.nEntries());

    //double* knobs = NULL; /* colamd arg 6: parameters (uses defaults if NULL) */
    double knobs[CCOLAMD_KNOBS];
    ccolamd_set_defaults(knobs);
    knobs[CCOLAMD_DENSE_ROW]=-1;
    knobs[CCOLAMD_DENSE_COL]=-1;

    int stats[CCOLAMD_STATS]; /* colamd arg 7: colamd output statistics and error codes */

    gttoc(Prepare);

    // call colamd, result will be in p
    /* returns (1) if successful, (0) otherwise*/
    if(nVars > 0) {
      gttic(ccolamd);
      int rv = ccolamd((int)nFactors, (int)nVars, (int)Alen, &A[0], &p[0], knobs, stats, &cmember[0]);
      if(rv != 1)
        throw runtime_error((boost::format("ccolamd failed with return value %1%")%rv).str());
    }

    //  ccolamd_report(stats);

    gttic(Fill_Ordering);
    // Convert elimination ordering in p to an ordering
    Ordering result;
    result.resize(nVars);
    for(size_t j = 0; j < nVars; ++j)
      result[j] = keys[p[j]];
    gttoc(Fill_Ordering);

    return result;
  }

  /* ************************************************************************* */
  Ordering Ordering::COLAMDConstrainedLast(
    const VariableIndex& variableIndex, const std::vector<Key>& constrainLast, bool forceOrder)
  {
    gttic(Ordering_COLAMDConstrainedLast);

    size_t n = variableIndex.size();
    std::vector<int> cmember(n, 0);

    // Build a mapping to look up sorted Key indices by Key
    FastMap<Key, size_t> keyIndices;
    size_t j = 0;
    BOOST_FOREACH(const VariableIndex::value_type key_factors, variableIndex)
      keyIndices.insert(keyIndices.end(), make_pair(key_factors.first, j++));

    // If at least some variables are not constrained to be last, constrain the
    // ones that should be constrained.
    int group = (constrainLast.size() != n ? 1 : 0);
    BOOST_FOREACH(Key key, constrainLast) {
      cmember[keyIndices.at(key)] = group;
      if(forceOrder)
        ++ group;
    }

    return Ordering::COLAMDConstrained(variableIndex, cmember);
  }

  /* ************************************************************************* */
  Ordering Ordering::COLAMDConstrainedFirst(
    const VariableIndex& variableIndex, const std::vector<Key>& constrainFirst, bool forceOrder)
  {
    gttic(Ordering_COLAMDConstrainedFirst);

    const int none = -1;
    size_t n = variableIndex.size();
    std::vector<int> cmember(n, none);

    // Build a mapping to look up sorted Key indices by Key
    FastMap<Key, size_t> keyIndices;
    size_t j = 0;
    BOOST_FOREACH(const VariableIndex::value_type key_factors, variableIndex)
      keyIndices.insert(keyIndices.end(), make_pair(key_factors.first, j++));

    // If at least some variables are not constrained to be last, constrain the
    // ones that should be constrained.
    int group = 0;
    BOOST_FOREACH(Key key, constrainFirst) {
      cmember[keyIndices.at(key)] = group;
      if(forceOrder)
        ++ group;
    }

    if(!forceOrder && !constrainFirst.empty())
      ++ group;
    BOOST_FOREACH(int& c, cmember)
      if(c == none)
        c = group;

    return Ordering::COLAMDConstrained(variableIndex, cmember);
  }

  /* ************************************************************************* */
  Ordering Ordering::COLAMDConstrained(const VariableIndex& variableIndex,
    const FastMap<Key, int>& groups)
  {
    gttic(Ordering_COLAMDConstrained);
    size_t n = variableIndex.size();
    std::vector<int> cmember(n, 0);

    // Build a mapping to look up sorted Key indices by Key
    FastMap<Key, size_t> keyIndices;
    size_t j = 0;
    BOOST_FOREACH(const VariableIndex::value_type key_factors, variableIndex)
      keyIndices.insert(keyIndices.end(), make_pair(key_factors.first, j++));

    // Assign groups
    typedef FastMap<Key, int>::value_type key_group;
    BOOST_FOREACH(const key_group& p, groups) {
      // FIXME: check that no groups are skipped
      cmember[keyIndices.at(p.first)] = p.second;
    }

    return Ordering::COLAMDConstrained(variableIndex, cmember);
  }

  /* ************************************************************************* */
  void Ordering::print(const std::string& str, const KeyFormatter& keyFormatter) const
  {
    cout << str;
    // Print ordering in index order
    // Print the ordering with varsPerLine ordering entries printed on each line,
    // for compactness.
    static const size_t varsPerLine = 10;
    bool endedOnNewline = false;
    for(size_t i = 0; i < size(); ++i) {
      if(i % varsPerLine == 0)
        cout << "Position " << i << ": ";
      if(i % varsPerLine != 0)
        cout << ", ";
      cout << keyFormatter(at(i));
      if(i % varsPerLine == varsPerLine - 1) {
        cout << "\n";
        endedOnNewline = true;
      } else {
        endedOnNewline = false;
      }
    }
    if(!endedOnNewline)
      cout << "\n";
    cout.flush();
  }

  /* ************************************************************************* */
  bool Ordering::equals(const Ordering& other, double tol) const
  {
    return (*this) == other;
  }

}
