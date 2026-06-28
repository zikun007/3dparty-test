/**
 * @file syscoord.hpp
 * @author kun (2025202140016@whu.edu.cn)
 * @brief 统一坐标系统与姿态转换库
 * @details
 * 1. 内部核心算法统一且严格遵循：航天航空标准 (NED导航系 + FRD载体系)。
 * 2. 严格遵循 Hamilton 四元数规范。
 * 3. 尾部提供零开销的桥接接口，用于输出到机器人标准 (ENU导航系 + RFU载体系)。
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace nav_core {

// ============================================================================
// 常量定义 (WGS84)
// ============================================================================
inline constexpr double kD2R = M_PI / 180.0;
inline constexpr double kR2D = 180.0 / M_PI;

inline constexpr double kC = 299792458.0;
inline constexpr double kWe = 7.2921151467E-5;
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// clang-format off
inline constexpr double kWGS84F   = 0.0033528106647474805; // 扁率
inline constexpr double kWGS84Ra  = 6378137.0;             // 长半轴 a
inline constexpr double kWGS84Rb  = 6356752.3142451793;    // 短半轴 b
inline constexpr double kWGS84Gm0 = 398600441800000.0;     // 引力常数
inline constexpr double kWGS84E1  = 0.0066943799901413156; // 第一偏心率平方
// clang-format on

// ============================================================================
// 基础地球物理模型 (Internal Core)
// ============================================================================

inline Eigen::Vector2d rm_rn(double lat) {
    double sinlat = std::sin(lat);
    double tmp = 1.0 - kWGS84E1 * sinlat * sinlat;
    double sqrttmp = std::sqrt(tmp);
    return {kWGS84Ra * (1.0 - kWGS84E1) / (sqrttmp * tmp), kWGS84Ra / sqrttmp};
}

inline double gravity(const Eigen::Vector3d& blh) {
    double sin2 = std::sin(blh.x());
    sin2 *= sin2;
    return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2)
           + blh.z() * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh.z() * blh.z();
}

// ============================================================================
// 导航运动学 (Navigation Kinematics - NED)
// ============================================================================

inline Eigen::Vector3d Wie_ned(double lat) { return {kWe * std::cos(lat), 0.0, -kWe * std::sin(lat)}; }

inline Eigen::Vector3d Wen_ned(const Eigen::Vector2d& rmn, const Eigen::Vector3d& blh, const Eigen::Vector3d& v_ned) {
    return {v_ned.y() / (rmn.y() + blh.z()), -v_ned.x() / (rmn.x() + blh.z()),
            -v_ned.y() * std::tan(blh.x()) / (rmn.y() + blh.z())};
}

// ============================================================================
// 绝对位置与旋转矩阵转换 (ECEF <-> BLH <-> NED)
// ============================================================================

inline Eigen::Vector3d blh2xyz(const Eigen::Vector3d& blh) {
    double coslat = std::cos(blh.x()), sinlat = std::sin(blh.x());
    double coslon = std::cos(blh.y()), sinlon = std::sin(blh.y());
    double r_n = rm_rn(blh.x()).y();
    double rnh = r_n + blh.z();
    return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - r_n * kWGS84E1) * sinlat};
}

inline Eigen::Vector3d xyz2blh(const Eigen::Vector3d& xyz) {
    double p = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    if (p < 1e-6) {
        return {(xyz.z() > 0 ? M_PI / 2 : -M_PI / 2), 0.0, std::abs(xyz.z()) - kWGS84Rb};
    }

    double lat = std::atan(xyz.z() / (p * (1.0 - kWGS84E1)));
    double lon = 2.0 * std::atan2(xyz.y(), xyz.x() + p);
    double h = 0.0, h2 = 0.0, r_n = 0.0;

    do {
        h2 = h;
        r_n = rm_rn(lat).y();
        h = p / std::cos(lat) - r_n;
        lat = std::atan(xyz.z() / (p * (1.0 - kWGS84E1 * r_n / (r_n + h))));
    } while (std::abs(h - h2) > 1.0e-4);

    return {lat, lon, h};
}

inline Eigen::Matrix3d Rned2xyz(const Eigen::Vector3d& blh) {
    double sl = std::sin(blh.x()), cl = std::cos(blh.x());
    double sL = std::sin(blh.y()), cL = std::cos(blh.y());
    Eigen::Matrix3d dcm;
    dcm << -sl * cL, -sL, -cl * cL, -sl * sL, cL, -cl * sL, cl, 0.0, -sl;
    return dcm;
}

// ============================================================================
// 局部坐标点平移与转换 (Local Cartesian Points - NED)
// ============================================================================

inline Eigen::Vector3d ned2xyz(const Eigen::Vector3d& origin_blh, const Eigen::Vector3d& pt_ned) {
    return blh2xyz(origin_blh) + Rned2xyz(origin_blh) * pt_ned;
}

inline Eigen::Vector3d xyz2ned(const Eigen::Vector3d& origin_blh, const Eigen::Vector3d& pt_ecef) {
    return Rned2xyz(origin_blh).transpose() * (pt_ecef - blh2xyz(origin_blh));
}

// ============================================================================
// 姿态表示转换 (Attitude Conversions) - 内部统一为 FRD/NED, 四元数采用 Hamilton 约定
// ============================================================================

inline Eigen::Quaterniond dcm2quat(const Eigen::Matrix3d& dcm) { return Eigen::Quaterniond(dcm).normalized(); }
inline Eigen::Matrix3d quat2dcm(const Eigen::Quaterniond& q) { return q.toRotationMatrix(); }

/**
 * @brief 旋转矩阵转欧拉角 (FRD 载体 -> NED 导航系)
 * @details ZYX 顺序，输出 [Roll, Pitch, Yaw]，取值范围 Roll[-π, π], Pitch[-π/2, π/2], Yaw[-π, π]
 */
inline Eigen::Vector3d dcm2euler(const Eigen::Matrix3d& dcm) {
    Eigen::Vector3d euler;
    
    // Pitch 提取始终是稳定的
    euler.y() = std::atan(-dcm(2, 0) / std::sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2))); 

    if (dcm(2, 0) <= -0.999) { 
        // 1. 正向万向节死锁 (Pitch 接近 π/2)
        euler.x() = std::atan2(dcm(2, 1), dcm(2, 2)); // Roll
        // 修复：atan2 提取出的是 (Yaw - Roll)，必须加回 Roll 才能得到真正的 Yaw
        euler.z() = std::atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1))) + euler.x(); 
        
    } else if (dcm(2, 0) >= 0.999) { 
        // 2. 负向万向节死锁 (Pitch 接近 -π/2)
        euler.x() = std::atan2(dcm(2, 1), dcm(2, 2)); // Roll
        // 修复：atan2 提取出的是 (Yaw + Roll - π)，补偿 π 之后还要减去 Roll
        euler.z() = std::atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1))) + M_PI - euler.x(); 
        
    } else { 
        // 3. 正常情况
        euler.x() = std::atan2(dcm(2, 1), dcm(2, 2)); // Roll
        euler.z() = std::atan2(dcm(1, 0), dcm(0, 0)); // Yaw
    }

    // 约束 Yaw 在 [-π, π] 范围内 (因为上面万向节死锁加减 Roll 后可能越界)
    while (euler.z() > M_PI)  euler.z() -= 2.0 * M_PI;
    while (euler.z() < -M_PI) euler.z() += 2.0 * M_PI;

    return euler;
}

inline Eigen::Vector3d quat2euler(const Eigen::Quaterniond& q) { return dcm2euler(q.toRotationMatrix()); }

inline Eigen::Matrix3d euler2dcm(const Eigen::Vector3d& euler) {
    return Eigen::Matrix3d(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
}

inline Eigen::Quaterniond euler2quat(const Eigen::Vector3d& euler) { return Eigen::Quaterniond(euler2dcm(euler)); }

inline Eigen::Vector3d quat2rotvec(const Eigen::Quaterniond& q) {
    Eigen::AngleAxisd axisd(q);
    return axisd.angle() * axisd.axis();
}

inline Eigen::Quaterniond rotvec2quat(const Eigen::Vector3d& rotvec) {
    double angle = rotvec.norm();
    if (angle < 1e-8) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotvec.normalized()));
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d mat;
    mat << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return mat;
}

inline Eigen::Matrix4d quat_left(const Eigen::Quaterniond& q) {
    Eigen::Matrix4d ans;
    ans(0, 0) = q.w();
    ans.block<1, 3>(0, 1) = -q.vec().transpose();
    ans.block<3, 1>(1, 0) = q.vec();
    ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skew(q.vec());
    return ans;
}

inline Eigen::Matrix4d quat_right(const Eigen::Quaterniond& q) {
    Eigen::Matrix4d ans;
    ans(0, 0) = q.w();
    ans.block<1, 3>(0, 1) = -q.vec().transpose();
    ans.block<3, 1>(1, 0) = q.vec();
    ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() - skew(q.vec());
    return ans;
}

// ============================================================================
// 对外发布桥接层 (ROS Interfaces - ENU/RFU Bridge)
// ============================================================================

/**
 * @brief 置换矩阵 P (用于 NED<->ENU 以及 FRD<->RFU 的无损变换)
 * @details X_new = Y_old, Y_new = X_old, Z_new = -Z_old. P = P^T = P^-1.
 */
inline static Eigen::Matrix3d bridge_p_matrix() {
    Eigen::Matrix3d P;
    P << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
    return P;
}

/**
 * @brief 向量系转换 (如将速度、加速度从 NED 转 ENU，或将角速度从 FRD 转 RFU)
 */
inline Eigen::Vector3d ned2enu(const Eigen::Vector3d& v_ned) { return bridge_p_matrix() * v_ned; }
inline Eigen::Vector3d enu2ned(const Eigen::Vector3d& v_enu) { return bridge_p_matrix() * v_enu; }
inline Eigen::Vector3d frd2rfu(const Eigen::Vector3d& v_frd) { return bridge_p_matrix() * v_frd; }
inline Eigen::Vector3d rfu2frd(const Eigen::Vector3d& v_rfu) { return bridge_p_matrix() * v_rfu; }

/**
 * @brief ECEF <-> ENU 坐标转换桥接
 */
inline Eigen::Matrix3d Renu2xyz(const Eigen::Vector3d& blh) {
    return Rned2xyz(blh) * bridge_p_matrix(); // C_enu^ecef = C_ned^ecef * P
}
inline Eigen::Vector3d enu2xyz(const Eigen::Vector3d& origin_blh, const Eigen::Vector3d& pt_enu) {
    return ned2xyz(origin_blh, enu2ned(pt_enu));
}
inline Eigen::Vector3d xyz2enu(const Eigen::Vector3d& origin_blh, const Eigen::Vector3d& pt_ecef) {
    return ned2enu(xyz2ned(origin_blh, pt_ecef));
}

/**
 * @brief 姿态相似变换 (R_rfu^enu = P * R_frd^ned * P)
 * @details 绝佳的数学性质使得旋转矩阵在两个体系间无缝映射，且计算极快。
 */
inline Eigen::Matrix3d dcm_frd2ned_to_rfu2enu(const Eigen::Matrix3d& dcm_frd_ned) {
    Eigen::Matrix3d P = bridge_p_matrix();
    return P * dcm_frd_ned * P;
}

inline Eigen::Quaterniond quat_frd2ned_to_rfu2enu(const Eigen::Quaterniond& q_frd_ned) {
    return Eigen::Quaterniond(dcm_frd2ned_to_rfu2enu(q_frd_ned.toRotationMatrix())).normalized();
}

inline void getAzEl(const Eigen::Vector3d& blh, const Eigen::Vector3d& rcv_pos, const Eigen::Vector3d& sat_pos, double& az, double& el){
    Eigen::Vector3d dr = sat_pos - rcv_pos;
    Eigen::Matrix3d Rxyz2enu = Renu2xyz(blh).transpose();

    Eigen::Vector3d enu = Rxyz2enu * dr;
    double e = enu.x(), n = enu.y(), u = enu.z();
    double hor_dist = std::sqrt(e * e + n * n);

    el = std::atan(u/hor_dist);
    az = std::atan2(e, n);
    if (az < 0.0) az += 2.0 * M_PI;
}


} // namespace nav_core
