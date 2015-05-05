/**
 * @file testFastContainers.cpp
 *
 * @brief Test for the Fast* containers that use boost pool allocators and interfaces
 *
 * @date Sep 24, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/set.hpp>

#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastVector.h>

using namespace boost::assign;
using namespace gtsam;

/* ************************************************************************* */
TEST( testFastContainers, KeySet ) {

  FastVector<size_t> init_vector;
  init_vector += 2, 3, 4, 5;

  FastSet<size_t> actSet(init_vector);
  FastSet<size_t> expSet; expSet += 2, 3, 4, 5;
  EXPECT(actSet == expSet);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
