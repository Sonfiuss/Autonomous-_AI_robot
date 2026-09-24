// Debug logging gate. Compile with -DLINK_DEBUG to enable; stripped otherwise.
// Mirrors RM_DLOG / MC_DLOG so every module in project/ behaves the same way.
#ifndef LINK_DEBUG_H
#define LINK_DEBUG_H

#ifdef LINK_DEBUG
#include <cstdio>
#define LINK_DLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define LINK_DLOG(...)
#endif

#endif  // LINK_DEBUG_H
