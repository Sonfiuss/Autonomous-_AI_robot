// mc_cli: drives the MC C API from a text script on stdin, so a trajectory can
// be inspected without Python.
//
//   theta  <rad>                 heading at the first tick (default 0)
//   speed  <m/s> <rad/s>         cruise speed and yaw rate (0 = compiled default)
//   dt     <s>                   control tick (0 = compiled default)
//   rotate <rad>                 append a ROTATE primitive
//   forward <m>                  append a FORWARD primitive
//   move   <dx> <dy>             append a MOVE primitive (world frame)
//   stop                         append a STOP primitive
//   limit  <u> <v> <r>           print the feasible speed/accel for a direction
//   run                          expand and print the trajectory table
//   every  <n>                   print only every n-th row (default 1)
#include <cstdio>
#include <cstring>

#include "MC/mc_api.h"

namespace {

constexpr int MAX_PRIMS = 256;
constexpr int MAX_STEPS = 4096;
constexpr int LINE_MAX  = 256;

McPrimitive g_prims[MAX_PRIMS];
McStep      g_steps[MAX_STEPS];

void printTrajectory(const McRequest& req, int every) {
    McResult res{};
    res.steps    = g_steps;
    res.step_cap = MAX_STEPS;
    const int status = mc_run(&req, &res);
    std::printf("status %d %s\n", status, mc_status_text(status));
    if (status != MC_OK) return;
    std::printf("steps %d  duration %.3f s  end %.4f %.4f %.4f\n",
                res.step_len, res.duration_s, res.end_x, res.end_y, res.end_theta);
    std::printf("%8s %8s %8s %8s %9s %9s %9s %8s %8s %8s\n",
                "t", "u", "v", "r", "w1", "w2", "w3", "hz1", "hz2", "hz3");
    for (int i = 0; i < res.step_len; ++i) {
        if (every > 1 && i % every != 0 && i != res.step_len - 1) continue;
        const McStep& s = g_steps[i];
        std::printf("%8.3f %8.4f %8.4f %8.4f %9.4f %9.4f %9.4f %8.0f %8.0f %8.0f\n",
                    s.t, s.u, s.v, s.r, s.w[0], s.w[1], s.w[2],
                    s.hz[0] * s.dir[0], s.hz[1] * s.dir[1], s.hz[2] * s.dir[2]);
    }
}

}  // namespace

int main() {
    McRequest req{};
    req.prims = g_prims;
    int every = 1;
    char word[LINE_MAX];
    while (std::scanf("%255s", word) == 1) {
        if (std::strcmp(word, "theta") == 0) {
            if (std::scanf("%f", &req.start_theta) != 1) break;
        } else if (std::strcmp(word, "speed") == 0) {
            if (std::scanf("%f %f", &req.cruise_speed, &req.yaw_rate) != 2) break;
        } else if (std::strcmp(word, "dt") == 0) {
            if (std::scanf("%f", &req.dt) != 1) break;
        } else if (std::strcmp(word, "every") == 0) {
            if (std::scanf("%d", &every) != 1 || every < 1) break;
        } else if (std::strcmp(word, "rotate") == 0 || std::strcmp(word, "forward") == 0) {
            const int type = std::strcmp(word, "rotate") == 0 ? 0 : 1;
            if (req.n_prims >= MAX_PRIMS) break;
            float value = 0.0f;
            if (std::scanf("%f", &value) != 1) break;
            g_prims[req.n_prims].type = type;
            g_prims[req.n_prims].a    = value;
            g_prims[req.n_prims].b    = 0.0f;
            ++req.n_prims;
        } else if (std::strcmp(word, "move") == 0) {
            if (req.n_prims >= MAX_PRIMS) break;
            float dx = 0.0f, dy = 0.0f;
            if (std::scanf("%f %f", &dx, &dy) != 2) break;
            g_prims[req.n_prims].type = 2;
            g_prims[req.n_prims].a    = dx;
            g_prims[req.n_prims].b    = dy;
            ++req.n_prims;
        } else if (std::strcmp(word, "stop") == 0) {
            if (req.n_prims >= MAX_PRIMS) break;
            g_prims[req.n_prims].type = 3;
            g_prims[req.n_prims].a    = 0.0f;
            g_prims[req.n_prims].b    = 0.0f;
            ++req.n_prims;
        } else if (std::strcmp(word, "limit") == 0) {
            float u = 0.0f, v = 0.0f, r = 0.0f, vMax = 0.0f, accel = 0.0f;
            if (std::scanf("%f %f %f", &u, &v, &r) != 3) break;
            mc_max_speed(u, v, r, 0.0f, &vMax, &accel);
            std::printf("limit (%.2f %.2f %.2f): v_max %.4f accel %.4f\n", u, v, r, vMax, accel);
        } else if (std::strcmp(word, "run") == 0) {
            printTrajectory(req, every);
        } else {
            std::fprintf(stderr, "unknown command: %s\n", word);
            return 1;
        }
    }
    return 0;
}
