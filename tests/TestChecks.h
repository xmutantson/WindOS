#ifndef WINDOS_TEST_CHECKS_H
#define WINDOS_TEST_CHECKS_H
#include <cstdio>
#include <cstdlib>
// Fail in the console without abort()/Windows crash dialogs; never disabled by NDEBUG.
#define CHECK(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #condition); \
  std::exit(EXIT_FAILURE); \
} } while (0)
#endif
