#ifndef TESTS_RTK_ROVER_MANAGER_H
#define TESTS_RTK_ROVER_MANAGER_H

#include <AUnit.h>
using namespace aunit;

// Just to test the test file
test(correct) {
  int x = 1;
  assertEqual(x, 1);
}

test(incorrect) {
  int x = 2;
  assertNotEqual(x, 1);
}

#endif /*** TESTS_RTK_ROVER_MANAGER_H ***/