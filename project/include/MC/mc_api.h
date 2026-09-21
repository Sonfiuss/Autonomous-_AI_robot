// MC C API: the only header a foreign caller (Python ctypes, C, firmware) needs.
// Plain C structs, no exceptions, no allocation: the caller owns every buffer.
// Takes the primitive list MV produced plus a commanded speed, and returns the
// angular speed of each wheel for every control tick.
//
// Typical use:
//   McRequest req = {prims, n, theta0, 0, 0, 0};   // zeros -> compiled defaults
//   McResult  res = {steps, cap, 0, ...};
//   int status = mc_run(&req, &res);               // MC_OK -> res.steps filled
#ifndef MC_API_H
#define MC_API_H

#if defined(_WIN32) && defined(MC_BUILD_SHARED)
#define MC_API __declspec(dllexport)
#else
#define MC_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Same numbering as MvPrimitive.type in MV/mv_api.h. */
typedef struct { int type; float a; float b; } McPrimitive;

/* One control tick. `u, v, r` is the body velocity the wheels actually produce;
   `x, y, theta` is the pose reached at the END of the tick, integrated through
   rm::Odometry, so a caller can draw the motion without integrating anything. */
typedef struct {
    float       t;             /* s since the first tick */
    float       u, v, r;       /* m/s, m/s, rad/s */
    float       w[3];          /* wheel angular speed, rad/s, signed */
    float       hz[3];         /* pulse rate per wheel, always >= 0 */
    signed char dir[3];        /* +1 forward, -1 reverse */
    float       x, y, theta;   /* world pose after this tick, m / rad */
} McStep;

enum McStatus {
    MC_OK             = 0,
    MC_ERR_ARGS       = 1,  /* null pointer, negative count, capacity <= 0 */
    MC_ERR_PRIMITIVE  = 2,  /* primitive type outside 0..3 */
    MC_ERR_KINEMATICS = 3,  /* wheel layout in constants.h is singular */
    MC_ERR_BUFFER     = 4   /* the trajectory needs more steps than step_cap */
};

typedef struct {
    const McPrimitive* prims;
    int                n_prims;
    float              start_theta;   /* rad, heading at the first tick (MOVE legs need it) */
    float              cruise_speed;  /* m/s  for FORWARD/MOVE; <= 0 -> compiled default */
    float              yaw_rate;      /* rad/s for ROTATE;      <= 0 -> compiled default */
    float              dt;            /* s control tick;        <= 0 -> compiled default */
} McRequest;

typedef struct {
    McStep* steps;  int step_cap; int step_len;
    float   duration_s;                      /* step_len * dt */
    float   end_x, end_y, end_theta;         /* pose reached, integrated through rm::Odometry */
} McResult;

/* Expands the primitive list into per-tick wheel speeds. Returns an McStatus;
   outputs are valid only on MC_OK. */
MC_API int mc_run(const McRequest* req, McResult* res);

/* Feasible chassis speed and acceleration along a body-frame direction
   (u, v, r; magnitude ignored). `requested` caps the returned speed.
   Writes *v_max and *accel. Returns MC_OK or MC_ERR_ARGS. */
MC_API int mc_max_speed(float u, float v, float r, float requested, float* v_max, float* accel);

MC_API const char* mc_status_text(int status);
MC_API int mc_version(void);

#ifdef __cplusplus
}
#endif

#endif  /* MC_API_H */
