#include "CommandParser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <vector>

namespace kinematics {

// ── helpers ──────────────────────────────────────────────────────────────────

static std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return s;
}

static std::vector<std::string> tokenize(const std::string& s) {
    std::vector<std::string> tokens;
    std::istringstream ss(s);
    std::string tok;
    while (ss >> tok) {
        // strip trailing unit suffixes: cm, deg, °
        for (const char* suf : {"cm", "deg", "°"}) {
            size_t pos = tok.rfind(suf);
            if (pos != std::string::npos && pos == tok.size() - std::strlen(suf))
                tok.erase(pos);
        }
        if (!tok.empty()) tokens.push_back(toLower(tok));
    }
    return tokens;
}

// Find the first numeric token at or after index i; return true and advance i.
static bool nextFloat(const std::vector<std::string>& toks, size_t& i, float& val) {
    while (i < toks.size()) {
        try {
            size_t pos = 0;
            val = std::stof(toks[i], &pos);
            if (pos > 0) { ++i; return true; }
        } catch (...) {}
        ++i;
    }
    return false;
}

// ── direction helpers ─────────────────────────────────────────────────────────

// Returns dx, dy unit vectors for a direction token like "forward", "left",
// "forward-left", etc.  Returns false for unknown directions.
static bool resolveDirection(const std::string& dir,
                             float& dx, float& dy) {
    dx = dy = 0.f;

    if (dir == "forward")            { dx =  1.f; return true; }
    if (dir == "back"   ||
        dir == "backward")           { dx = -1.f; return true; }
    if (dir == "left")               { dy =  1.f; return true; }
    if (dir == "right")              { dy = -1.f; return true; }

    // diagonal: "forward-left", "back-right", etc.
    const float d = 1.f / std::sqrt(2.f);
    if (dir == "forward-left"  || dir == "left-forward")  { dx= d; dy= d; return true; }
    if (dir == "forward-right" || dir == "right-forward") { dx= d; dy=-d; return true; }
    if (dir == "back-left"     || dir == "left-back")     { dx=-d; dy= d; return true; }
    if (dir == "back-right"    || dir == "right-back")    { dx=-d; dy=-d; return true; }

    return false;
}

// ── parsers per verb ──────────────────────────────────────────────────────────

// "move <direction> <dist>"
static bool parseMove(const std::vector<std::string>& toks, Command& cmd,
                      std::string& error) {
    if (toks.size() < 3) {
        error = "move: expected 'move <direction> <distance>'"; return false;
    }
    float dx, dy;
    if (!resolveDirection(toks[1], dx, dy)) {
        error = "move: unknown direction '" + toks[1] + "'"; return false;
    }
    size_t idx = 2;
    float dist;
    if (!nextFloat(toks, idx, dist)) {
        error = "move: missing distance value"; return false;
    }
    cmd.dx_cm = dx * dist;
    cmd.dy_cm = dy * dist;
    return true;
}

// "spin [left|right] <angle>"  or  "self-round [left|right] <angle>"
static bool parseSpin(const std::vector<std::string>& toks, Command& cmd,
                      std::string& error) {
    float sign = 1.f;  // default CCW
    size_t idx = 1;

    if (idx < toks.size() && (toks[idx] == "left" || toks[idx] == "right")) {
        sign = (toks[idx] == "right") ? -1.f : 1.f;
        ++idx;
    }

    float angle;
    if (!nextFloat(toks, idx, angle)) {
        error = "spin: missing angle value"; return false;
    }
    cmd.rotate_deg = sign * angle;
    return true;
}

// "arc <direction> <dist> turn <angle>"
static bool parseArc(const std::vector<std::string>& toks, Command& cmd,
                     std::string& error) {
    if (toks.size() < 4) {
        error = "arc: expected 'arc <direction> <dist> turn <angle>'"; return false;
    }
    float dx, dy;
    if (!resolveDirection(toks[1], dx, dy)) {
        error = "arc: unknown direction '" + toks[1] + "'"; return false;
    }
    size_t idx = 2;
    float dist;
    if (!nextFloat(toks, idx, dist)) {
        error = "arc: missing distance value"; return false;
    }

    // look for "turn" keyword then angle
    bool found_turn = false;
    for (size_t i = idx; i < toks.size(); ++i) {
        if (toks[i] == "turn") {
            ++i;
            float angle;
            if (!nextFloat(toks, i, angle)) {
                error = "arc: missing angle after 'turn'"; return false;
            }
            cmd.rotate_deg = angle;
            found_turn = true;
            break;
        }
    }
    if (!found_turn) {
        error = "arc: expected 'turn <angle>'"; return false;
    }

    cmd.dx_cm = dx * dist;
    cmd.dy_cm = dy * dist;
    return true;
}

// ── public entry point ────────────────────────────────────────────────────────

bool parseCommand(const std::string& input, Command& out, std::string& error) {
    out = Command{};
    auto toks = tokenize(input);
    if (toks.empty()) { error = "empty command"; return false; }

    const std::string& verb = toks[0];

    if (verb == "move")
        return parseMove(toks, out, error);

    if (verb == "spin" || verb == "self-round" || verb == "selfround")
        return parseSpin(toks, out, error);

    if (verb == "arc")
        return parseArc(toks, out, error);

    error = "unknown verb '" + verb + "' — use: move, spin, self-round, arc";
    return false;
}

} // namespace kinematics
