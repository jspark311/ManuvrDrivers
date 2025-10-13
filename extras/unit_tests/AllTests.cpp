/*
File:   AllTests.cpp
Author: J. Ian Lindsay
Date:   2021.09.25

This is the top-level testing program for CppPotpourri (or, C3P, for short).

NOTE: CryptoBurrito has its own test program. All of these tests must pass
  before testing CryptoBurrito.

Global unit-testing TODO list:
--------------------------------------------------------------------------------
TODO: Research testing frameworks for C++ again.

TODO: If you won't do that, this is at least a good place to start doing some
  dependency injection for GPIO. If we override the weak references in
  AbstractPlatform, we can fake pin behaviors from a separate thread.

TODO: About that... This program is presumably being run under linux, and so we
  have threads. Apart from mortality, there is no good reason that some directed
  concurrency testing of modules isn't already being done. This won't be an
  _exact_ simulation of ISR behavior, but it is close enough to catch almost
  everything that would happen in that context that C3P is concerned about.
*/

#include <cstdio>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

#include <fstream>
#include <iostream>

#include "CppPotpourri.h"
#include "StringBuilder.h"
#include "AsyncSequencer.h"
#include "PriorityQueue.h"
#include "C3PLinux.h"
#include "C3PRandom/C3PRandom.h"


/*******************************************************************************
* Test reporting functions that are intended to be called from unit tests...
*******************************************************************************/

int generate_random_text_buffer(StringBuilder* buf, const int RANDOM_BUF_LEN) {
  int ret = 0;
  if ((RANDOM_BUF_LEN > 0) && (nullptr != buf)) {
    uint8_t tmp_buf[RANDOM_BUF_LEN+1];
    random_fill(tmp_buf, (uint32_t) RANDOM_BUF_LEN);
    tmp_buf[RANDOM_BUF_LEN] = 0;
    for (int i = 0; i < RANDOM_BUF_LEN; i++) {
      tmp_buf[i] = (0x30 + (tmp_buf[i] % 0x4E));
    }
    ret = RANDOM_BUF_LEN;
    buf->concat(tmp_buf, RANDOM_BUF_LEN);
  }
  return ret;
}


uint64_t generate_random_uint64() {
  uint64_t ret = 0;
  random_fill((uint8_t*) &ret, (uint32_t) sizeof(uint64_t));
  return ret;
}


int64_t generate_random_int64() {
  int64_t ret = 0;
  random_fill((uint8_t*) &ret, (uint32_t) sizeof(int64_t));
  return ret;
}


bool flip_coin() {
  return (0 != (1 & randomUInt32()));
}


float generate_random_float() {
  // True entropy: 28-bit
  return (FLT_EPSILON * (int32_t) (0x87FFFFFF & randomUInt32()));
}

double generate_random_double() {
  // True entropy: 58-bit
  return (DBL_EPSILON * (int64_t) (0x87FFFFFFFFFFFFFF & generate_random_uint64()));
}

Vector3<float> generate_random_vect3f() {
  Vector3<float> ret(generate_random_float(), generate_random_float(), generate_random_float());
  return ret;
}



void dump_c3pvalue(C3PValue* a) {
  if (a) {
    StringBuilder log;
    a->printDebug(&log);
    printf("%s\n", (char*) log.string());
  }
  else {
    printf("dump_c3pvalue() was passed a nullptr.\n");
  }
}


void dump_kvp(KeyValuePair* a) {
  if (a) {
    StringBuilder log;
    a->printDebug(&log);
    printf("%s\n", (char*) log.string());
  }
  else {
    printf("dump_kvp() was passed a nullptr.\n");
  }
}

void dump_timeseries(TimeSeriesBase* a) {
  if (a) {
    StringBuilder log;
    a->printSeries(&log);
    printf("%s\n", (char*) log.string());
  }
  else {
    printf("dump_timeseries() was passed a nullptr.\n");
  }
}

void dump_strbldr(StringBuilder* a) {
  if (a) {
    StringBuilder log;
    a->printDebug(&log);
    printf("%s\n", (char*) log.string());
  }
  else {
    printf("dump_strbldr() was passed a nullptr.\n");
  }
}



/*******************************************************************************
* Something terrible.
* Textual inclusion of CPP files until a testing framework is writen or adopted.
*******************************************************************************/
#include "TestModules/PlatformAssurances.cpp"


/*******************************************************************************
* Aggregation functions that call pieces from each unit test source file.
* Brittle, ugly, hard to understand. Recommend me a test framework...
* I want something that can do dependency injection with a bit more grace than
*   the dabbling I've done for the platform.
*******************************************************************************/

/**
* Prints the sizes of various types. Informational only. No test.
*/
void printTypeSizes() {
  printf("===< Type sizes >=======================================\n");
}


/*******************************************************************************
* Top-level tests are managed using AsyncSequencer.
* The dependency graph will allow us to order tests in a bottom-up manner, with
*   more sophisticated pieces being run only if base support passes.
* NOTE: The flag value ordering is not important.
*******************************************************************************/
#define CHKLST_CI_PLATFORM_TESTS        0x00000001
#define CHKLST_CI_RPI_GPIO_TESTS        0x00000002
#define CHKLST_SX1503_TESTS             0x00000004
#define CHKLST_PCA9539_TESTS            0x00000008

#define CHKLST_ADG2128_TESTS            0x00000010
#define CHKLST_BME280_TESTS             0x00000020
#define CHKLST_DS1881_TESTS             0x00000040
#define CHKLST_MCP4728_TESTS            0x00000080
#define CHKLST_MCP356x_TESTS            0x00000100

#define CHKLST_MANUVR_PMU_TESTS         0x40000000
#define CHKLST_VIAM_SONUS_TESTS         0x80000000



/*
* We're going to do a bit of clutter-control...
* Tier-0: The platform itself.
* Tier-1: The CI box relies on these drivers to test other drivers.
* Tier-2: Tests of base-level drivers that have no other drivers as deps.
* Tier-3: Tests of composite drivers.
*
* Complex, high-level tests are encouraged to cite one of these tiers as a
*   dependency for brevity. This will save testing complexity by not
*   requiring strict dep-knowledge for a given high-level capability (which
*   probably relies on StringBuilder, and at least one other thing covered
*   by CHKLST_ALL_TIER_1_TESTS).
*/
#define CHKLST_ALL_TIER_0_TESTS (CHKLST_CI_PLATFORM_TESTS)

#define CHKLST_ALL_TIER_1_TESTS ( \
  CHKLST_CI_RPI_GPIO_TESTS | CHKLST_SX1503_TESTS | CHKLST_PCA9539_TESTS)

#define CHKLST_ALL_TIER_2_TESTS ( \
  CHKLST_BME280_TESTS | CHKLST_ADG2128_TESTS | CHKLST_MCP4728_TESTS | \
  CHKLST_DS1881_TESTS)

// These are tests of composite drivers.
#define CHKLST_ALL_TIER_3_TESTS ( \
  CHKLST_VIAM_SONUS_TESTS)

/*
#define CHKLST_ALL_TESTS ( \
  CHKLST_ALL_TIER_0_TESTS | CHKLST_ALL_TIER_1_TESTS | \
  CHKLST_ALL_TIER_2_TESTS | CHKLST_ALL_TIER_3_TESTS)
*/

#define CHKLST_ALL_TESTS (CHKLST_CI_PLATFORM_TESTS)

/*
* Top level test definitions.
*
* NOTE: These tests are listed in their dependency order for clarity only. Their
*   ordering in this list is arbitrary with respect to the outcome. All tests
*   with sated dependencies will be given a chance to run, even if the test that
*   just ran failed.
*/
const StepSequenceList TOP_LEVEL_TEST_LIST[] = {
  // The tests of the test program's implementation of AbstractPlatform. Nothing
  //   else will make any sense if this fails. It is ultimately a dependency for
  //   everything being tested, in one way or another.
  // TODO: Verification of correct operation of any dependency injection
  //   features should fall into this block as well.
  { .FLAG         = CHKLST_CI_PLATFORM_TESTS,
    .LABEL        = "Test program ontology",
    .DEP_MASK     = (0),   // Bottom Turtle
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == platform_assurance_test_main()) ? 1:-1);  }
  },

  //////////////////////////////////////////////////////////////////////////////
  // Now, we can begin TIER-1 tests. We have lots of GPIO pins to deal with, and
  //   a few drivers to help us deal with it.
/*
  // We use the SX1503 and Linux sysfs GPIO.
  { .FLAG         = CHKLST_CI_RPI_GPIO_TESTS,
    .LABEL        = "CI native GPIO",
    .DEP_MASK     = (CHKLST_ALL_TIER_0_TESTS),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == ci_gpio_main()) ? 1:-1);  }
  },

  // The SX1503 is used to handle pins that are not explicitly under the control
  //   of their respective drivers.
  { .FLAG         = CHKLST_SX1503_TESTS,
    .LABEL        = "SX1508",
    .DEP_MASK     = (CHKLST_ALL_TIER_0_TESTS),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == ci_sx1503_main()) ? 1:-1);  }
  },
*/

  //////////////////////////////////////////////////////////////////////////////
  // Now moving into TIER-2, where we can take GPIO as a given.


  //////////////////////////////////////////////////////////////////////////////
  // Here begins TIER-3.
  // Some drivers are composites of other drivers.

};

AsyncSequencer checklist_drvr_tests(TOP_LEVEL_TEST_LIST, (sizeof(TOP_LEVEL_TEST_LIST) / sizeof(TOP_LEVEL_TEST_LIST[0])));


/*******************************************************************************
* The top-level main function.                                                 *
*******************************************************************************/
int main(int argc, char *argv[]) {
  srand(time(NULL));
  printTypeSizes();

  checklist_drvr_tests.requestSteps(CHKLST_ALL_TESTS);
  while (!checklist_drvr_tests.request_completed() && (0 == checklist_drvr_tests.failed_steps(false))) {
    checklist_drvr_tests.poll();
  }
  int exit_value = (checklist_drvr_tests.request_fulfilled() ? 0 : 1);

  StringBuilder report_output;
  checklist_drvr_tests.printDebug(&report_output, "Final test report");
  printf("%s\n", (char*) report_output.string());

  exit(exit_value);
}
