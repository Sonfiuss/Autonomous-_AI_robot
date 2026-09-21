#include "MC/mc_api.h"

#include "MC/executor.h"
#include "MC/mc_debug.h"
#include "MC/speed_limit.h"
#include "RM/odometry.h"

namespace {

constexpr int MC_VERSION = 2;   // 2: McStep carries the per-tick pose

// One executor instance for the process, like MV's planner: the API is not
// re-entrant (documented in mc_api.h / README).
mc::Executor   g_executor;
mc::Primitive  g_primitives[mc::cfg::MAX_PRIMITIVES];
rm::Odometry   g_odometry;
rm::OmniKinematics g_kinematics;

bool validRequest(const McRequest* req) {
    if (req == nullptr || req->n_prims < 0) return false;
    if (req->n_prims > 0 && req->prims == nullptr) return false;
    return req->n_prims <= mc::cfg::MAX_PRIMITIVES;
}

bool validResult(const McResult* res) {
    return res != nullptr && res->steps != nullptr && res->step_cap > 0;
}

// Copies the caller's primitives into typed storage, rejecting unknown types.
bool copyPrimitives(const McRequest* req) {
    for (int i = 0; i < req->n_prims; ++i) {
        const int type = req->prims[i].type;
        if (type < static_cast<int>(mc::PrimitiveType::ROTATE) ||
            type > static_cast<int>(mc::PrimitiveType::STOP)) {
            MC_DLOG("mc_run: primitive %d has type %d\n", i, type);
            return false;
        }
        g_primitives[i].type = static_cast<mc::PrimitiveType>(type);
        g_primitives[i].a    = req->prims[i].a;
        g_primitives[i].b    = req->prims[i].b;
    }
    return true;
}

}  // namespace

extern "C" {

int mc_run(const McRequest* req, McResult* res) {
    if (!validResult(res)) return MC_ERR_ARGS;
    res->step_len   = 0;
    res->duration_s = 0.0f;
    res->end_x = res->end_y = res->end_theta = 0.0f;
    if (!validRequest(req)) return MC_ERR_ARGS;
    if (!copyPrimitives(req)) return MC_ERR_PRIMITIVE;

    const float dt = req->dt > 0.0f ? req->dt : mc::cfg::TICK_S;
    mc::MotionLimits limits;
    if (req->cruise_speed > 0.0f) limits.cruiseSpeed = req->cruise_speed;
    if (req->yaw_rate > 0.0f)     limits.yawRate     = req->yaw_rate;

    if (!g_executor.load(g_primitives, req->n_prims, limits, req->start_theta)) {
        return g_kinematics.valid() ? MC_ERR_ARGS : MC_ERR_KINEMATICS;
    }

    rm::Pose start;
    start.theta = req->start_theta;
    g_odometry.reset(start);

    mc::MotionStep step;
    int count = 0;
    while (g_executor.step(dt, &step)) {
        if (count >= res->step_cap) {
            MC_DLOG("mc_run: trajectory exceeds capacity %d\n", res->step_cap);
            return MC_ERR_BUFFER;
        }
        McStep& out = res->steps[count];
        out.t = step.t;
        out.u = step.body.u;
        out.v = step.body.v;
        out.r = step.body.r;
        float wheelAngles[rm::cfg::NUM_WHEELS];
        for (int i = 0; i < rm::cfg::NUM_WHEELS; ++i) {
            out.w[i]   = step.wheels.w[i];
            out.hz[i]  = step.steps[i].freqHz;
            out.dir[i] = step.steps[i].forward ? 1 : -1;
            wheelAngles[i] = step.wheels.w[i] * dt;
        }
        g_odometry.updateWheelAngles(wheelAngles, dt);
        out.x     = g_odometry.pose().x;
        out.y     = g_odometry.pose().y;
        out.theta = g_odometry.pose().theta;
        ++count;
    }

    res->step_len   = count;
    res->duration_s = static_cast<float>(count) * dt;
    res->end_x      = g_odometry.pose().x;
    res->end_y      = g_odometry.pose().y;
    res->end_theta  = g_odometry.pose().theta;
    MC_DLOG("mc_run: %d steps, %.3f s, end (%.3f %.3f %.3f)\n",
            count, res->duration_s, res->end_x, res->end_y, res->end_theta);
    return MC_OK;
}

int mc_max_speed(float u, float v, float r, float requested, float* v_max, float* accel) {
    if (v_max == nullptr || accel == nullptr) return MC_ERR_ARGS;
    rm::BodyVel direction;
    direction.u = u;
    direction.v = v;
    direction.r = r;
    // A zero request means "as fast as the wheels allow".
    const float ask = requested > 0.0f ? requested : rm::cfg::MAX_WHEEL_OMEGA_RAD_S;
    const mc::AxisLimits limits = mc::limitsFor(g_kinematics, direction, ask);
    *v_max = limits.vMax;
    *accel = limits.accel;
    return MC_OK;
}

const char* mc_status_text(int status) {
    switch (status) {
        case MC_OK:             return "ok";
        case MC_ERR_ARGS:       return "bad arguments";
        case MC_ERR_PRIMITIVE:  return "unknown primitive type";
        case MC_ERR_KINEMATICS: return "wheel layout is singular";
        case MC_ERR_BUFFER:     return "trajectory does not fit in the buffer";
        default:                return "unknown status";
    }
}

int mc_version(void) { return MC_VERSION; }

}  // extern "C"
