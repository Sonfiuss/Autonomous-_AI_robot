// Debug logging gate. Compile with -DFW_DEBUG to enable; stripped otherwise.
// Mirrors RM_DLOG / MC_DLOG / LINK_DLOG. On ESP-IDF, stderr reaches the console.
#ifndef FW_DEBUG_H
#define FW_DEBUG_H

#ifdef FW_DEBUG
#include <cstdio>
#define FW_DLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define FW_DLOG(...)
#endif

#endif  // FW_DEBUG_H
