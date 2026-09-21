// MV C API: the only header a foreign caller (Python ctypes, C, firmware)
// needs. Plain C structs, no exceptions, no allocation: the caller owns every
// buffer. Coordinates are world frame, metres, radians (CCW from +x).
//
// Typical use:
//   MvRequest req = {...};  MvResult res = {...caller buffers...};
//   int status = mv_plan(&req, &res);   // MV_OK -> res.path / waypoints / prims filled
#ifndef MV_API_H
#define MV_API_H

#if defined(_WIN32) && defined(MV_BUILD_SHARED)
#define MV_API __declspec(dllexport)
#else
#define MV_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float x; float y; } MvPoint;
typedef struct { float x; float y; float theta; } MvPose;
typedef struct { int type; float a; float b; } MvPrimitive;  /* type: 0 ROTATE, 1 FORWARD, 2 MOVE, 3 STOP */

enum MvStatus {
    MV_OK                = 0,
    MV_ERR_ARGS          = 1,  /* null pointer, polygon with < 3 vertices, capacity <= 0 */
    MV_ERR_ROOM_TOO_BIG  = 2,  /* room exceeds the compiled grid size */
    MV_ERR_START_BLOCKED = 3,  /* no free cell within the snap radius of start */
    MV_ERR_GOAL_BLOCKED  = 4,  /* no free cell within the snap radius of goal */
    MV_ERR_NO_PATH       = 5,  /* start and goal lie in different free regions */
    MV_ERR_BUFFER        = 6   /* an output buffer is too small */
};

typedef struct {
    float          room_w;        /* m, along +x */
    float          room_l;        /* m, along +y */
    float          robot_radius;  /* m, hull radius; SAFETY_MARGIN_M is added internally */
    const MvPoint* vertices;      /* all polygon vertices back to back */
    const int*     poly_sizes;    /* vertex count per polygon (>= 3 each) */
    int            n_polys;
    MvPose         start;
    MvPose         goal;
    int            holonomic;     /* 0: ROTATE/FORWARD, 1: MOVE dx dy */
} MvRequest;

typedef struct {
    MvPoint*     path;      int path_cap; int path_len;  /* start, cell centres..., goal */
    MvPoint*     waypoints; int wp_cap;   int wp_len;    /* smoothed corners incl. start and goal */
    MvPrimitive* prims;     int prim_cap; int prim_len;
    float        length_m;                                /* waypoint polyline length */
    MvPoint      snapped_start;                           /* free cell centre actually searched from */
    MvPoint      snapped_goal;
} MvResult;

/* Plans start -> goal. Returns an MvStatus; outputs are valid only on MV_OK. */
MV_API int mv_plan(const MvRequest* req, MvResult* res);

/* Rasterises the request's room + polygons (inflated) into out[rows * cols],
   row-major, 1 = blocked, row 0 at y = 0. cap = capacity of out in bytes.
   Sets *cols, *rows, *res_m. Returns MV_OK, MV_ERR_ARGS, MV_ERR_ROOM_TOO_BIG or MV_ERR_BUFFER. */
MV_API int mv_grid(const MvRequest* req, unsigned char* out, int cap, int* cols, int* rows, float* res_m);

MV_API const char* mv_status_text(int status);
MV_API int mv_version(void);

#ifdef __cplusplus
}
#endif

#endif  /* MV_API_H */
