#include "nav_core/syscoord.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

int g_total = 0;
int g_failures = 0;

#define CHECK(expr)                                                           \
    do {                                                                      \
        ++g_total;                                                            \
        if (!(expr)) {                                                        \
            ++g_failures;                                                     \
            std::cerr << "FAIL [" << __LINE__ << "]: " << #expr << "\n";    \
        }                                                                     \
    } while (0)

bool near(double lhs, double rhs, double eps = 1e-8) {
    return std::abs(lhs - rhs) <= eps;
}

template <typename Lhs, typename Rhs>
bool near_matrix(const Eigen::MatrixBase<Lhs>& lhs,
                 const Eigen::MatrixBase<Rhs>& rhs,
                 double eps = 1e-8) {
    if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
        return false;
    }

    for (Eigen::Index r = 0; r < lhs.rows(); ++r) {
        for (Eigen::Index c = 0; c < lhs.cols(); ++c) {
            if (!near(lhs(r, c), rhs(r, c), eps)) {
                return false;
            }
        }
    }
    return true;
}

void test_earth_model() {
    const Eigen::Vector2d radii = nav_core::rm_rn(0.0);
    CHECK(near(radii.y(), nav_core::kWGS84Ra, 1e-6));

    const Eigen::Vector3d equator_sea_level(0.0, 0.0, 0.0);
    CHECK(near(nav_core::gravity(equator_sea_level), 9.7803267715, 1e-10));
}

void test_position_roundtrip() {
    const Eigen::Vector3d blh(30.0 * nav_core::kD2R, 114.0 * nav_core::kD2R, 42.0);
    const Eigen::Vector3d xyz = nav_core::blh2xyz(blh);
    const Eigen::Vector3d recovered_blh = nav_core::xyz2blh(xyz);

    CHECK(near_matrix(recovered_blh, blh, 1e-7));

    const Eigen::Vector3d ned(12.0, -3.0, 1.5);
    const Eigen::Vector3d point_xyz = nav_core::ned2xyz(blh, ned);
    CHECK(near_matrix(nav_core::xyz2ned(blh, point_xyz), ned, 1e-7));
}

void test_attitude_conversions() {
    const Eigen::Vector3d euler(0.1, -0.2, 0.3);
    const Eigen::Matrix3d dcm = nav_core::euler2dcm(euler);

    CHECK(near_matrix(dcm.transpose() * dcm, Eigen::Matrix3d::Identity(), 1e-8));
    CHECK(near_matrix(nav_core::dcm2euler(dcm), euler, 1e-8));

    const Eigen::Quaterniond q = nav_core::euler2quat(euler);
    CHECK(near_matrix(nav_core::quat2dcm(q), dcm, 1e-8));

    const Eigen::Vector3d rotvec(0.01, -0.02, 0.03);
    const Eigen::Quaterniond q_rotvec = nav_core::rotvec2quat(rotvec);
    CHECK(near_matrix(nav_core::quat2rotvec(q_rotvec), rotvec, 1e-8));
}

void test_bridges_and_az_el() {
    const Eigen::Vector3d ned(1.0, 2.0, -3.0);
    CHECK(near_matrix(nav_core::enu2ned(nav_core::ned2enu(ned)), ned));

    const Eigen::Vector3d frd(1.0, -2.0, 3.0);
    CHECK(near_matrix(nav_core::rfu2frd(nav_core::frd2rfu(frd)), frd));

    const Eigen::Vector3d blh(0.0, 0.0, 0.0);
    const Eigen::Vector3d receiver = nav_core::blh2xyz(blh);
    const Eigen::Vector3d satellite = nav_core::enu2xyz(blh, Eigen::Vector3d(1000.0, 0.0, 1000.0));

    double az = 0.0;
    double el = 0.0;
    nav_core::getAzEl(blh, receiver, satellite, az, el);
    CHECK(near(az, nav_core::kD2R * 90.0, 1e-8));
    CHECK(near(el, nav_core::kD2R * 45.0, 1e-8));
}

} // namespace

int main() {
    test_earth_model();
    test_position_roundtrip();
    test_attitude_conversions();
    test_bridges_and_az_el();

    std::cout << "syscoord: " << (g_total - g_failures) << " / " << g_total << " passed\n";
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
