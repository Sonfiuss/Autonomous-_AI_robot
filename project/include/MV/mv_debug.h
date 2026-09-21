// Debug logging gate. Compile with -DMV_DEBUG to enable; stripped otherwise.
#ifndef MV_DEBUG_H
#define MV_DEBUG_H

#ifdef MV_DEBUG
#include <cstdio>
#define MV_DLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define MV_DLOG(...)
#endif

#endif  // MV_DEBUG_H
