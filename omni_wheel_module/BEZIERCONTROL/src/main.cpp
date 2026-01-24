// Real-time control loop for a 3-wheel omni robot.
// Input sources supported: keyboard (Windows example), joystick/sensor/AI placeholders.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

constexpr double V_MAX = 1.0;       	// m/s linear speed limit
constexpr double OMEGA_MAX = 1.0;   	// rad/s angular speed limit (safer than hardware max 50)
constexpr double STOP_VEL = 0.001;  	// m/s threshold to treat as zero
constexpr double STOP_OMEGA = 0.1047; 	// rad/s (~6 degrees)

enum class InputType { Keyboard, Joystick, Sensor, AI };
enum class Frame { Mobile, World };

struct InputConfig {
	InputType type{InputType::Keyboard};
	Frame frame{Frame::Mobile};
	double update_hz{50.0};
	double deadzone{0.08}; // only used for joystick-like inputs
};

struct RawInput {
	double x{0.0};     // normalized -1..1 lateral (robot right) or world X
	double y{0.0};     // normalized -1..1 forward (robot front) or world Y
	double omega{0.0}; // normalized -1..1 yaw
	bool active{false};
};

struct Velocity {
	double vx{0.0};
	double vy{0.0};
	double omega{0.0};
};

std::atomic<bool> keep_running{true};

double clamp(double v, double lo, double hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

double apply_deadzone(double v, double dz) {
	if (std::abs(v) < dz) return 0.0;
	const double sign = (v >= 0.0) ? 1.0 : -1.0;
	return sign * (std::abs(v) - dz) / (1.0 - dz);
}

std::pair<double, double> rotate(double vx, double vy, double angle_rad) {
	const double c = std::cos(angle_rad);
	const double s = std::sin(angle_rad);
	return {vx * c - vy * s, vx * s + vy * c};
}

void sleep_until_next(const std::chrono::steady_clock::time_point& start,
					  const std::chrono::duration<double>& period) {
#ifdef _WIN32
	const auto target = start + period;
	const auto now = std::chrono::steady_clock::now();
	if (target > now) {
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(target - now).count();
		if (ms > 0) {
			::Sleep(static_cast<DWORD>(ms));
		}
	}
#else
	std::this_thread::sleep_until(start + period);
#endif
}

#ifdef _WIN32
bool key_down(int virtual_key) {
	return (GetAsyncKeyState(virtual_key) & 0x8000) != 0;
}

bool poll_keyboard(RawInput& out) {
	bool any = false;

	// WASD (giữ lại) và phím mũi tên: lên/xuống/trái/phải/lùi.
	if (key_down('W') || key_down(VK_UP))    { out.y += 1.0; any = true; }
	if (key_down('S') || key_down(VK_DOWN))  { out.y -= 1.0; any = true; }
	if (key_down('D') || key_down(VK_RIGHT)) { out.x += 1.0; any = true; }
	if (key_down('A') || key_down(VK_LEFT))  { out.x -= 1.0; any = true; }

	// Xoay: phím + / - (cả bàn phím chính và numpad) và Q/E giữ nguyên.
	if (key_down('E') || key_down(VK_OEM_PLUS) || key_down(VK_ADD))    { out.omega += 1.0; any = true; }
	if (key_down('Q') || key_down(VK_OEM_MINUS) || key_down(VK_SUBTRACT)) { out.omega -= 1.0; any = true; }

	if (!any) return false;

	// Normalize to [-1, 1] if multiple keys combine.
	out.x = clamp(out.x, -1.0, 1.0);
	out.y = clamp(out.y, -1.0, 1.0);
	out.omega = clamp(out.omega, -1.0, 1.0);
	out.active = true;
	return true;
}
#else
bool poll_keyboard(RawInput&) { return false; }
#endif

bool poll_joystick(const InputConfig& cfg, RawInput& out) {
	(void)cfg; (void)out;
	return false; // TODO: Wire up actual joystick axes here. Expect raw axes in [-1, 1].
}

bool poll_sensor(RawInput& out) {
	(void)out;
	return false; // TODO: Replace with sensor-driven command generation.
}

bool poll_ai(RawInput& out) {
	(void)out;
	return false; // TODO: Replace with AI planner output.
}

bool fetch_input(const InputConfig& cfg, RawInput& out) {
	switch (cfg.type) {
		case InputType::Keyboard:
			return poll_keyboard(out);
		case InputType::Joystick:
			return poll_joystick(cfg, out);
		case InputType::Sensor:
			return poll_sensor(out);
		case InputType::AI:
			return poll_ai(out);
		default:
			return false;
	}
}

Velocity map_to_velocity(const RawInput& in, const InputConfig& cfg) {
	Velocity v{};
	double x = in.x;
	double y = in.y;
	double w = in.omega;

	if (cfg.type == InputType::Joystick) {
		x = apply_deadzone(x, cfg.deadzone);
		y = apply_deadzone(y, cfg.deadzone);
		w = apply_deadzone(w, cfg.deadzone);
	}

	v.vx = clamp(x, -1.0, 1.0) * V_MAX;
	v.vy = clamp(y, -1.0, 1.0) * V_MAX;
	v.omega = clamp(w, -1.0, 1.0) * OMEGA_MAX;
	return v;
}

void send_to_robot(const Velocity& v_cmd) {
	// TODO: Replace with bus/driver call. For now, print.
	std::cout << "cmd vx=" << v_cmd.vx
			  << " vy=" << v_cmd.vy
			  << " omega=" << v_cmd.omega << "\n";
}

void handle_signal(int) {
	keep_running = false;
}

}  // namespace

int main() {
	std::signal(SIGINT, handle_signal);

	InputConfig cfg;
	cfg.type = InputType::Keyboard; // set to Keyboard/Joystick/Sensor/AI
	cfg.frame = Frame::Mobile;      // set to Frame::World if input is world-frame
	cfg.update_hz = 50.0;           // control loop frequency
	cfg.deadzone = 0.08;            // only matters for joystick

	const auto period = std::chrono::duration<double>(1.0 / cfg.update_hz);
	double robot_heading = 0.0; // replace with IMU/odometry when available

	while (keep_running) {
		const auto start = std::chrono::steady_clock::now();

		RawInput raw{};
		Velocity cmd{};
		if (fetch_input(cfg, raw) && raw.active) {
			cmd = map_to_velocity(raw, cfg);
		}

		if (cfg.frame == Frame::World) {
			auto rot = rotate(cmd.vx, cmd.vy, -robot_heading);
			cmd.vx = rot.first;
			cmd.vy = rot.second;
		}

		if (std::hypot(cmd.vx, cmd.vy) < STOP_VEL && std::abs(cmd.omega) < STOP_OMEGA) {
			cmd = {};
		}

		send_to_robot(cmd);


		sleep_until_next(start, period);
	}

	return 0;
}
