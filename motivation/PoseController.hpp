#pragma once
#include <cmath>

/**
 * Proportional pose controller cho omni 3-wheel robot.
 *
 * Input : current odometry (x, y, theta_rad) + target (x, y)
 * Output: velocity commands (vx, vy, omega) trong robot frame
 *
 * Vòng lặp ngoài gọi compute() ở tần số cố định (ví dụ 20 Hz).
 */
class PoseController {
public:
    struct Pose   { float x, y, theta; };  // theta radians
    struct CmdVel { float vx, vy, omega; bool reached; };

    struct Params {
        float kp_lin    = 1.2f;   // gain tuyến tính
        float kp_ang    = 2.0f;   // gain góc
        float max_vel   = 1.5f;   // m/s tối đa
        float max_omega = 2.0f;   // rad/s tối đa
        float goal_tol  = 0.05f;  // ngưỡng đến đích (m)
        float ang_tol   = 0.05f;  // ngưỡng góc (rad)
    };

    explicit PoseController(const Params& p = {}) : m_p(p) {}

    void setParams(const Params& p) { m_p = p; }

    /**
     * Tính velocity command từ pose hiện tại → target.
     * Gọi ở mỗi chu kỳ điều khiển.
     */
    CmdVel compute(const Pose& current, const Pose& target) const {
        float dx = target.x - current.x;
        float dy = target.y - current.y;
        float dist = std::sqrt(dx*dx + dy*dy);

        if (dist < m_p.goal_tol) return {0.f, 0.f, 0.f, true};

        // Chuyển error sang robot frame (xoay theo theta hiện tại)
        float ct = std::cos(current.theta);
        float st = std::sin(current.theta);
        float ex =  dx*ct + dy*st;   // trục tiến (forward)
        float ey = -dx*st + dy*ct;   // trục ngang (strafe)

        float vx = m_p.kp_lin * ex;
        float vy = m_p.kp_lin * ey;

        // Điều chỉnh hướng mũi robot về phía mục tiêu
        float target_theta = std::atan2(dy, dx);
        float dtheta = normalizeAngle(target_theta - current.theta);
        float omega  = m_p.kp_ang * dtheta;

        // Giới hạn tốc độ tuyến tính
        float vmag = std::sqrt(vx*vx + vy*vy);
        if (vmag > m_p.max_vel) {
            float scale = m_p.max_vel / vmag;
            vx *= scale;
            vy *= scale;
        }

        // Giới hạn tốc độ góc
        omega = clamp(omega, -m_p.max_omega, m_p.max_omega);

        return {vx, vy, omega, false};
    }

private:
    Params m_p;

    static float normalizeAngle(float a) {
        while (a >  (float)M_PI) a -= 2.f*(float)M_PI;
        while (a < -(float)M_PI) a += 2.f*(float)M_PI;
        return a;
    }
    static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};
