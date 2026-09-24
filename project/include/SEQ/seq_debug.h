// Debug logging gate. Compile with -DSEQ_DEBUG to enable; stripped otherwise.
// Mirrors RM_DLOG / MC_DLOG / LINK_DLOG so every module in project/ behaves the
// same way.
#ifndef SEQ_DEBUG_H
#define SEQ_DEBUG_H

#ifdef SEQ_DEBUG
#include <cstdio>
#define SEQ_DLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define SEQ_DLOG(...)
#endif

#endif  // SEQ_DEBUG_H
