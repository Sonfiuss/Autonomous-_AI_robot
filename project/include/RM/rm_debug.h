// Debug logging gate. Compile with -DRM_DEBUG to enable; stripped otherwise.
// On ESP32 (Arduino / ESP-IDF) fprintf(stderr) reaches the USB serial console.
#ifndef RM_DEBUG_H
#define RM_DEBUG_H

#ifdef RM_DEBUG
#include <cstdio>
#define RM_DLOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define RM_DLOG(...)
#endif

#endif  // RM_DEBUG_H
