#pragma once
// Minimal shared test harness used by all stereo-camera unit tests.
#include <cstdio>
#include <cmath>

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(expr) do { \
    if (expr) { printf("  PASS  %s\n", #expr); ++s_pass; } \
    else      { printf("  FAIL  %s  (line %d)\n", #expr, __LINE__); ++s_fail; } \
} while(0)

#define CHECK_NEAR(a, b, tol) \
    CHECK(std::fabs(static_cast<double>(a) - static_cast<double>(b)) < (tol))

static inline void section(const char* name) { printf("\n── %s\n", name); }

static inline int test_report(const char* suite) {
    printf("\n[%s]  %d passed  /  %d failed\n", suite, s_pass, s_fail);
    return s_fail > 0 ? 1 : 0;
}
