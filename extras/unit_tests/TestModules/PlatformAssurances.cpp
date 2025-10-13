/*
File:   PlatformAssurances.cpp
Author: J. Ian Lindsay
Date:   2023.09.13


Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This program tests the implementation of the platform as it is used to execute
  all of our unit tests. This is the proper place for verifying the operation
  of things like pinRead()/pinSet(), RNG, millis()/micros(), as well as any
  dependency-injection strategies that are employed therein.

In the future, we may also run tests on the platform contract itself, or
  implementation of same that is provided by C3P itself.

TODO: John_00_Fleming_-_JOOF_Radio_045
*/

/*******************************************************************************
* Platform assurance testing
*******************************************************************************/

/*
* Testing RNG implementations is a whole deep topic in itself. But for the sake
*   of this test program, we only care that the values are sufficiently
*   different between calls to generate unique test cases.
* We aren't doing any serious cryptography in this program, nor is this a test
*   of CryptoBurrito. CryptoBurrito will do its own RNG testing to a standard
*   appropriate to its purpose.
*/
int platform_rng_tests() {
  int ret = -1;
  printf("Verifying acceptable RNG operation... \n");
  printf("\tVerifying RNG is not a stub that returns 0... ");
  uint32_t val = randomUInt32();
  if (0 == val) {    // It _could_ happen with odds of 1-in-(2^32).
    val = randomUInt32();
  }
  if (0 != val) {    // But twice is 2^32 times as suspect... Fail.
    printf("Pass.\n\tVerifying RNG returns different values on subsequent calls... ");
    const uint32_t RNG_TEST_VAL_0 = randomUInt32();
    const uint32_t RNG_TEST_VAL_1 = randomUInt32();
    const uint32_t RNG_TEST_VAL_2 = randomUInt32();
    if ((RNG_TEST_VAL_0 != RNG_TEST_VAL_1) & (RNG_TEST_VAL_0 != RNG_TEST_VAL_2) & (RNG_TEST_VAL_2 != RNG_TEST_VAL_1)) {
      printf("Pass.\n\tVerifying RNG returns full-spectrum values, and isn't a blind count... ");
      // Regarding RNG_SPECTRA_TEST_MAX_ITERATIONS:
      // The odds of this test returning false-failures goes up as
      //   RNG_SPECTRA_TEST_MAX_ITERATIONS becomes smaller. If we can't fill
      //   32-bits within a few hundred cycles, the RNG is too gimped to serve
      //   our purposes.
      // TODO: Calculate odds that a single coin flip comes up tails 1000 times
      //   in a row, multiplied by 32 coins.
      const int RNG_SPECTRA_TEST_MAX_ITERATIONS = 1000;
      int bailout_count = 0;
      uint32_t whiteness_test_value = randomUInt32();
      while ((RNG_SPECTRA_TEST_MAX_ITERATIONS > bailout_count++) & (0xFFFFFFFF != whiteness_test_value)) {
        whiteness_test_value |= randomUInt32();
      }
      if (0xFFFFFFFF == whiteness_test_value) {
        printf("Passed in %d iterations.\n\tRNG appears sufficient for tests to be valid.\n", bailout_count);
        ret = 0;
      }
      else printf("Fail. RNG output is insufficiently random. Test value ended at 0x%08x after %d iterations.\n", whiteness_test_value, bailout_count);
    }
  }
  else printf("Fail. RNG gave 0 twice-in-a-row. There is a 1-in-(2^64 that this is a false-failure.\n");
  return ret;
}


/*
* Test that the pRNG class is working correctly.
*/
int platform_rng_api_tests() {
  int ret = -1;
  printf("Verifying RNG API via pRNG... \n");
  // We'll need a test matrix...
  const uint32_t FUZZ_VAL_COUNT = (18 + (randomUInt32() % 12));
  const uint64_t SEED_VALUE_0 = generate_random_uint64();
  const uint64_t SEED_VALUE_1 = generate_random_uint64();
  uint32_t RNG_TEST_VALUE[6][FUZZ_VAL_COUNT];
  C3P_pRNG rng0;            // [0] and [1] should differ from each other, since it is left up to system time.
  C3P_pRNG rng1;            //
  C3P_pRNG rng2;            // [2] and [3] should match each other, but shouldn't match anything else.
  C3P_pRNG rng3;            //
  C3P_pRNG rng4;            // [4] and [5] should match each other, but shouldn't match anything else.
  C3P_pRNG rng5;            //
  rng0.init();
  rng1.init();
  rng2.init(SEED_VALUE_0);
  rng3.init(SEED_VALUE_0);
  rng4.init(SEED_VALUE_1);
  rng5.init(SEED_VALUE_1);

  // Collect the test data...
  for (uint32_t i = 0; i < FUZZ_VAL_COUNT; i++) {
    RNG_TEST_VALUE[0][i] = rng0.randomUInt32();
    RNG_TEST_VALUE[1][i] = rng1.randomUInt32();
    RNG_TEST_VALUE[2][i] = rng2.randomUInt32();
    RNG_TEST_VALUE[3][i] = rng3.randomUInt32();
    RNG_TEST_VALUE[4][i] = rng4.randomUInt32();
    RNG_TEST_VALUE[5][i] = rng5.randomUInt32();
  }

  printf("\tVerifying Two pRNGs seeded by timers are different... ");
  bool test_passed = true;
  for (uint32_t i = 0; i < FUZZ_VAL_COUNT; i++) {
    if (RNG_TEST_VALUE[0][i] == RNG_TEST_VALUE[1][i]) {  test_passed = false;  break;  }
  }
  if (test_passed) {
    printf("Pass.\n\tVerifying pRNG returns different values on subsequent calls... ");
    for (uint32_t i = 0; i < (FUZZ_VAL_COUNT-1); i++) {
      if (RNG_TEST_VALUE[0][i] == RNG_TEST_VALUE[0][i+1]) {  test_passed = false;  break;  }
      if (RNG_TEST_VALUE[1][i] == RNG_TEST_VALUE[1][i+1]) {  test_passed = false;  break;  }
      if (RNG_TEST_VALUE[2][i] == RNG_TEST_VALUE[2][i+1]) {  test_passed = false;  break;  }
      if (RNG_TEST_VALUE[3][i] == RNG_TEST_VALUE[3][i+1]) {  test_passed = false;  break;  }
      if (RNG_TEST_VALUE[4][i] == RNG_TEST_VALUE[4][i+1]) {  test_passed = false;  break;  }
      if (RNG_TEST_VALUE[5][i] == RNG_TEST_VALUE[5][i+1]) {  test_passed = false;  break;  }
    }
    if (test_passed) {
      printf("Pass.\n\tVerifying pRNG returns the same (distinct sequence for the same seed value... ");
      for (uint32_t i = 0; i < FUZZ_VAL_COUNT; i++) {
        if (RNG_TEST_VALUE[2][i] != RNG_TEST_VALUE[3][i]) {  test_passed = false;  break;  }
        if (RNG_TEST_VALUE[4][i] != RNG_TEST_VALUE[5][i]) {  test_passed = false;  break;  }
      }
      if (test_passed) {
        printf("Pass\n\n");
        ret = 0;
      }
    }
  }


  if (0 != ret) {
    printf("Fail.\n");
  }
  return ret;
}



/*
* C3P system time is given by millis() and micros().
* This function only tests the validity of the system time functions in terms of
*   self-reference, and if the test environment gives an implementation of both
*   functions that is within the contractual bounds of AbstractPlatform. It does
*   NOT make any attempt to cross-check them against any notions of real time.
* The reason for this is two-part:
*   1) The contract for AbstractPlatform doesn't specify drift or jitter limits,
*      counter wrap behavior, bit-width of time values, or grounding (if any) to
*      a specific anchor in real time. If your class needs those things, you
*      should not be using system time. Use a proper RTC instead.
*   2) For the purposes of testing the behaviors of time-sensitive classes, this
*      test program might manipulate the system time values directly to simulate
*      various conditions that might be a problem for a given class.
*
* TODO: Write the bridge fxns to make explicit timer return values possible.
*
* The two functions should return values which evolve at the same rate. That
*   is, (millis() / 1000) should always be equal to the return from micros(),
*   wrap-range exempted. Ideally, this would be a natural consequence of both
*   functions drawing from the same timing source. But that may not be the case
*   for any given platform.
*
* Assumptions made that allow this test to pass:
*   1) Execution proceeds at such a rate that microseconds can be seen to pass
*      at a resolution better than ALLOWABLE_SLOP_IN_MICROS. You may think that
*      this isn't a tall order in 2023, but this isn't the bare metal. Unknown
*      kernel overhead, a multi-threading OS and possible virtualization are all
*      capable of converting this assumption into a test failure. In real
*      applications, this assumption is not required, and code under test needs
*      to be capable of dealing with the fact that calling micros() might take
*      longer than a single microsecond.
*   2) Despite testing the wrap-controlled mark functions, this test disregards
*      the possibility of timer wrap. It is one of the first tests to run, and
*      the test program rebases the true system time to give the illusion of
*      running on a microcontroller with a freshly-zeroed timer register.
*/
int platform_system_time_tests() {
  printf("Verifying acceptable operation of millis() and micros()... \n");
  printf("\tVerifying that micros() evolves, and evolves in the right direction... ");
  // These values should be fairly conservative to allow testing to pass in a
  //   variety of environments that are terrible for real-time handling at the
  //   microsecond scale. If testing anything tighter than this is required, the
  //   tests will need to leverage the test program's timer value-injector to
  //   directly specify values for micros() to return.
  const long SYSTIME_EVOLUTION_MAX_ITERATIONS = 10000;
  const long ALLOWABLE_SLOP_IN_MICROS         = 500;

  // SPIN_UNTIL_MICROS should have some entropy. But we need to run this test
  //   for a minimum of two milliseconds for our later tests of millis() to be
  //   meaningful.
  const unsigned long SPIN_UNTIL_MICROS = ((1000 * (2 + (randomUInt32() % 15))) + micros()) + ALLOWABLE_SLOP_IN_MICROS;
  int bailout_count = 0;
  const unsigned long TEST_START_MICROS = micros();
  const unsigned long TEST_START_MILLIS = millis();
  unsigned long micros_return_0 = TEST_START_MICROS;
  unsigned long micros_return_1 = TEST_START_MICROS;

  while (((SPIN_UNTIL_MICROS - ALLOWABLE_SLOP_IN_MICROS) > micros_return_1) & (SYSTIME_EVOLUTION_MAX_ITERATIONS > bailout_count++)) {
    if (micros_return_0 != micros_return_1) {
      // If the micro state evolved, make sure it was both sequential and in
      //   the ascending direction.
      // Also make sure that our timer-mark wrappers do the advertised thing...
      const unsigned long MICROS_SINCE_CHANGE       = micros_since(TEST_START_MICROS);
      const unsigned long MICROS_UNTIL_RETURN       = micros_until(SPIN_UNTIL_MICROS);
      const unsigned long MICROS_SINCE_TEST_START   = (micros_return_1 - TEST_START_MICROS);
      const unsigned long MICROS_UNTIL_TEST_ENDS    = (SPIN_UNTIL_MICROS - micros_return_1);

      if (micros_return_1 < micros_return_0) {
        printf("Fail. Timer is not ascending, and it is too early for wrap to be the reason.\n");
        return -1;
      }
      if (ALLOWABLE_SLOP_IN_MICROS < strict_abs_delta((uint32_t) MICROS_SINCE_CHANGE, (uint32_t) MICROS_SINCE_TEST_START)) {
        printf("Fail. (%lu = micros_since(%lu)) disagrees with our own notions of elapsed time (%lu = %lu - %lu).\n", MICROS_SINCE_CHANGE, TEST_START_MICROS, MICROS_SINCE_TEST_START, micros_return_1, TEST_START_MICROS);
        return -1;
      }
      if (ALLOWABLE_SLOP_IN_MICROS < strict_abs_delta((uint32_t) MICROS_UNTIL_RETURN, (uint32_t) MICROS_UNTIL_TEST_ENDS)) {
        printf("Fail. (%lu = micros_until(%lu)) disagrees with our own notions of elapsed time (%lu = %lu - %lu).\n", MICROS_UNTIL_RETURN, SPIN_UNTIL_MICROS, MICROS_UNTIL_TEST_ENDS, SPIN_UNTIL_MICROS, micros_return_1);
        return -1;
      }
      micros_return_0 = micros_return_1;
      bailout_count = 0;  // State evolved.
    }
    else {
      micros_return_1 = micros();
    }
  }
  if (SYSTIME_EVOLUTION_MAX_ITERATIONS <= bailout_count) {
    printf("Fail. Timer is not evolving.\n");
    return -1;
  }
  printf("Pass. Execution rate was %d loops-per-us.\n\tVerifying that micros() and millis() evolve at the same rate... ", bailout_count);
  unsigned long timer_rationalizer_us = micros();
  unsigned long timer_rationalizer_ms = millis();

  //uint32_t tr_loops = 0;
  //while ((timer_rationalizer_us != (timer_rationalizer_ms / 1000)) & (tr_loops++ < 1000)) {
  //  // If the timer values can't be rationalized after 1000 tries, give up.
  //  timer_rationalizer_us = micros();
  //  timer_rationalizer_ms = millis();
  //}
  //if (tr_loops >= 1000) {
  //  printf("Fail. millis() and micros() disagreed to often for their respective costs to be the cause. (%lu / %lu)\n", timer_rationalizer_ms, timer_rationalizer_us);
  //  return -1;
  //}
  const unsigned long TEST_STOP_MICROS    = timer_rationalizer_us;
  const unsigned long TEST_STOP_MILLIS    = timer_rationalizer_ms;
  const long MICROS_SPENT        = (TEST_STOP_MICROS - TEST_START_MICROS);
  const long MILLIS_SPENT        = (TEST_STOP_MILLIS - TEST_START_MILLIS);
  const long MS_SPENT_VIA_MICROS = (MICROS_SPENT / 1000);
  int ret = -1;
  if (MICROS_SPENT > 0) {
    if (MILLIS_SPENT > 0) {
      // Make sure the ratio matches the outcome.
      // NOTE: We tolerate the truncation of integer division. systime isn't a float.
      if (MILLIS_SPENT == (MS_SPENT_VIA_MICROS)) {
        printf("Pass.\n\tmillis() and micros() appear to be adequate for testing.\n");
        ret = 0;
      }
      else printf("Fail. It appears that a different number of ms and us have passed (%ld versus %ld).\n", MILLIS_SPENT, MICROS_SPENT);
    }
    else printf("Fail. MILLIS_SPENT came out negative (%ld) with test beginning at mark (%lu).\n", MILLIS_SPENT, TEST_START_MILLIS);
  }
  else printf("Fail. MICROS_SPENT came out negative (%ld) with test beginning at mark (%lu).\n", MICROS_SPENT, TEST_START_MICROS);

  return ret;
}


/*
* C3P has an optional abstract thread model. This test ensures that it actually
*   works in the test program.
*/
int platform_threading_tests() {
  int ret = -1;
  // TODO: This
  return ret;
}


/*
* This test program should have provided the implementations of all of the GPIO
*   functions demanded by AbstractPlatform.h. In doing so, it also supplied a
*   set of emulated pins, some of which are permanently cross-connected for the
*   sake of testing.
*/
int platform_gpio_tests() {
  int ret = -1;
  // TODO: This
  return ret;
}



void print_types_platform() {
  printf("\tAbstractPlatform         %u\t%u\n", sizeof(AbstractPlatform), alignof(AbstractPlatform));
  printf("\tC3P_pRNG                 %u\t%u\n", sizeof(C3P_pRNG),         alignof(C3P_pRNG));
}



/*******************************************************************************
* Test plan
*******************************************************************************/
#define CHKLST_PLATFORM_RNG_WORKS     0x00000001  //
#define CHKLST_PLATFORM_TIMER_CHECK   0x00000002  //
#define CHKLST_PLATFORM_THREAD_CHECK  0x00000004  //
#define CHKLST_PLATFORM_GPIO_API      0x40000000  //
#define CHKLST_PLATFORM_RNG_API       0x80000000  //

#define CHKLST_PLATFORM_TESTS_ALL ( \
  CHKLST_PLATFORM_RNG_WORKS | CHKLST_PLATFORM_TIMER_CHECK | CHKLST_PLATFORM_RNG_API)

const StepSequenceList TOP_LEVEL_PLATFORM_TEST_LIST[] = {
  { .FLAG         = CHKLST_PLATFORM_RNG_WORKS,
    .LABEL        = "Test environment RNG operation",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == platform_rng_tests()) ? 1:-1);  }
  },
  { .FLAG         = CHKLST_PLATFORM_TIMER_CHECK,
    .LABEL        = "Test environment timer operation",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == platform_system_time_tests()) ? 1:-1);  }
  },
  { .FLAG         = CHKLST_PLATFORM_THREAD_CHECK,
    .LABEL        = "Test environment thread operation",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == platform_threading_tests()) ? 1:-1);  }
  },
  { .FLAG         = CHKLST_PLATFORM_GPIO_API,
    .LABEL        = "GPIO API",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == platform_gpio_tests()) ? 1:-1);  }
  },
  { .FLAG         = CHKLST_PLATFORM_RNG_API,
    .LABEL        = "RNG API",
    .DEP_MASK     = (CHKLST_PLATFORM_RNG_WORKS | CHKLST_PLATFORM_TIMER_CHECK),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((0 == platform_rng_api_tests()) ? 1:-1);  }
  },
};

AsyncSequencer platform_test_plan(TOP_LEVEL_PLATFORM_TEST_LIST, (sizeof(TOP_LEVEL_PLATFORM_TEST_LIST) / sizeof(TOP_LEVEL_PLATFORM_TEST_LIST[0])));



/*******************************************************************************
* The main function
*******************************************************************************/

int platform_assurance_test_main() {
  const char* const MODULE_NAME = "AbstractPlatform";
  printf("===< %s >=======================================\n", MODULE_NAME);

  platform_test_plan.requestSteps(CHKLST_PLATFORM_TESTS_ALL);
  while (!platform_test_plan.request_completed() && (0 == platform_test_plan.failed_steps(false))) {
    platform_test_plan.poll();
  }
  int ret = (platform_test_plan.request_fulfilled() ? 0 : 1);

  StringBuilder report_output;
  platform_test_plan.printDebug(&report_output, "AbstractPlatform test report");
  printf("%s\n", (char*) report_output.string());

  return ret;
}
