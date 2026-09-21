// Debug logging gate. Compile with -DMC_DEBUG to enable; stripped otherwise.
#ifndef MC_DEBUG_H
#define MC_DEBUG_H

#ifdef MC_DEBUG
#include <cstdio>
#define MC_DLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define MC_DLOG(...)
#endif

#endif  // MC_DEBUG_H
