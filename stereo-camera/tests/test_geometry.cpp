/**
 * test_geometry — encodes implement_plan_stereo.md §8 table (G1–G6).
 * fx=fy=600, cx=320, cy=240, r=0 unless noted. Units: mm.
 */
#include "test_util.hpp"
#include "../core/Geometry.hpp"
#include <vector>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using geom::Vec3;
using geom::Mat3;

static Config cfg() {
    Config c;            // defaults already fx=fy=600, cx=320, cy=240, r=0
    return c;
}

static Vec3 world(float u, float v, float Z, float pan, float tilt, const Config& c) {
    Mat3 R = geom::rotation(pan, tilt, c);
    return geom::pixelToWorld(u, v, Z, R, c);
}

static void G1_forward() {
    section("G1 centre pixel, Z=1000, θ=0 φ=0 -> (1000,0,0)");
    Config c = cfg();
    Vec3 p = world(c.cx, c.cy, 1000.f, 0.f, 0.f, c);
    CHECK_NEAR(p[0], 1000.f, 1e-2f);
    CHECK_NEAR(p[1],    0.f, 1e-2f);
    CHECK_NEAR(p[2],    0.f, 1e-2f);
}

static void G2_pan_right() {
    section("G2 centre pixel, Z=1000, θ=90 φ=0 -> (0,1000,0)");
    Config c = cfg();
    Vec3 p = world(c.cx, c.cy, 1000.f, 90.f, 0.f, c);
    CHECK_NEAR(p[0],    0.f, 1e-2f);
    CHECK_NEAR(p[1], 1000.f, 1e-2f);
    CHECK_NEAR(p[2],    0.f, 1e-2f);
}

static void G3_tilt_up() {
    section("G3 centre pixel, Z=1000, θ=0 φ=90 -> (0,0,1000)");
    Config c = cfg();
    Vec3 p = world(c.cx, c.cy, 1000.f, 0.f, 90.f, c);
    CHECK_NEAR(p[0],    0.f, 1e-2f);
    CHECK_NEAR(p[1],    0.f, 1e-2f);
    CHECK_NEAR(p[2], 1000.f, 1e-2f);
}

static void G4_offcentre_ray() {
    section("G4 pixel (440,240), Z=1000, θ=0 φ=0 -> (1000,200,0)");
    Config c = cfg();
    Vec3 p = world(440.f, 240.f, 1000.f, 0.f, 0.f, c);
    CHECK_NEAR(p[0], 1000.f, 1e-2f);
    CHECK_NEAR(p[1],  200.f, 1e-2f);   // (440-320)/600*1000 = 200, mapped to +Y
    CHECK_NEAR(p[2],    0.f, 1e-2f);
}

// Fit a plane to points, return RMS distance (mm).
static float planarityRMS(const std::vector<Vec3>& pts) {
    // centroid
    Vec3 cen{0,0,0};
    for (auto& p : pts) { cen[0]+=p[0]; cen[1]+=p[1]; cen[2]+=p[2]; }
    float n = (float)pts.size();
    cen[0]/=n; cen[1]/=n; cen[2]/=n;
    // covariance
    double cov[3][3] = {{0}};
    for (auto& p : pts) {
        double dx=p[0]-cen[0], dy=p[1]-cen[1], dz=p[2]-cen[2];
        double d[3]={dx,dy,dz};
        for (int i=0;i<3;++i) for(int j=0;j<3;++j) cov[i][j]+=d[i]*d[j];
    }
    // smallest-eigenvector via inverse power iteration (good enough for test)
    // Use a simple approach: try the 3 axis normals + iterate.
    // For these synthetic planes the normal is near a coordinate axis, so we
    // measure spread along the smallest-variance axis directly.
    double var[3] = {cov[0][0], cov[1][1], cov[2][2]};
    int axis = 0; if (var[1]<var[axis]) axis=1; if (var[2]<var[axis]) axis=2;
    double ss = var[axis] / n;
    return (float)std::sqrt(ss);
}

static void G5_flat_wall() {
    section("G5 fronto-parallel plane X=2000, noise-free -> planarity RMS < 1 mm");
    Config c = cfg();
    Mat3 R = geom::rotation(0.f, 0.f, c);
    std::vector<Vec3> pts;
    for (int v = 0; v < 480; v += 20)
        for (int u = 0; u < 640; u += 20)
            pts.push_back(geom::pixelToWorld((float)u, (float)v, 2000.f, R, c));
    float rms = planarityRMS(pts);
    printf("  G5 planarity RMS = %.5f mm\n", rms);
    CHECK(rms < 1.0f);
    // every X should equal 2000 for a fronto-parallel (constant-Z) patch
    bool all_x = true;
    for (auto& p : pts) if (std::fabs(p[0]-2000.f) > 1e-1f) all_x = false;
    CHECK(all_x);
}

static void G6_wall_across_pans() {
    section("G6 same world wall X=2000 across θ∈{-44,0,44} -> merged RMS < 2 mm");
    Config c = cfg();
    const float D = 2000.f;
    std::vector<Vec3> merged;
    for (float pan : {-44.f, 0.f, 44.f}) {
        Mat3 R = geom::rotation(pan, 0.f, c);
        const float th = pan * (float)(M_PI/180.0);
        const float ct = std::cos(th), st = std::sin(th);
        for (int v = 200; v < 280; v += 10)
            for (int u = 200; u < 440; u += 10) {
                float a = ((float)u - c.cx) / c.fx;   // normalised x
                // depth so that resulting world X == D  (see task notes)
                float Z = D / (ct - a * st);
                if (Z <= 0) continue;
                Vec3 p = geom::pixelToWorld((float)u, (float)v, Z, R, c);
                merged.push_back(p);
            }
    }
    // all merged points must lie on the world plane X = D
    float maxerr = 0.f;
    for (auto& p : merged) maxerr = std::max(maxerr, std::fabs(p[0] - D));
    printf("  G6 max |X-2000| = %.5f mm over %zu pts\n", maxerr, merged.size());
    CHECK(maxerr < 2.0f);
}

int main() {
    printf("=== test_geometry ===\n");
    G1_forward();
    G2_pan_right();
    G3_tilt_up();
    G4_offcentre_ray();
    G5_flat_wall();
    G6_wall_across_pans();
    return test_report("test_geometry");
}
