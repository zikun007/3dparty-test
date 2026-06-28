/**
 * @file test_eigen3.cpp
 * @brief Board-oriented Eigen smoke test for GNSS/INS tight integration.
 *
 * Covered paths:
 *   Core fixed-size matrix/vector operations, LU inverse/solve, LLT Cholesky,
 *   covariance propagation/update algebra, quaternion attitude operations, and
 *   Eigen aligned STL containers.
 *
 * This test intentionally avoids QR, LDLT, COD, and Eigen::isApprox(), because
 * those APIs are removed from this embedded profile after i.MX93 GCC 13.3
 * repeatedly failed on those template paths.
 */

#include <Eigen/Eigen>
#include <Eigen/StdDeque>
#include <Eigen/StdList>
#include <Eigen/StdVector>

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <list>
#include <vector>

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

constexpr float kPi = 3.14159265358979323846f;

bool near_float(float lhs, float rhs, float eps = 1e-5f) {
    return std::abs(lhs - rhs) <= eps;
}

template <typename Lhs, typename Rhs>
bool coeffs_near(const Eigen::MatrixBase<Lhs>& lhs,
                 const Eigen::MatrixBase<Rhs>& rhs,
                 float eps = 1e-5f) {
    if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
        return false;
    }

    for (Eigen::Index r = 0; r < lhs.rows(); ++r) {
        for (Eigen::Index c = 0; c < lhs.cols(); ++c) {
            if (!near_float(lhs(r, c), rhs(r, c), eps)) {
                return false;
            }
        }
    }
    return true;
}

template <typename MatrixType>
bool matrix_is_finite(const Eigen::MatrixBase<MatrixType>& matrix) {
    for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
        for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
            if (!std::isfinite(matrix(r, c))) {
                return false;
            }
        }
    }
    return true;
}

template <typename MatrixType>
bool diagonal_is_positive(const Eigen::MatrixBase<MatrixType>& matrix) {
    const Eigen::Index n = matrix.rows() < matrix.cols() ? matrix.rows() : matrix.cols();
    for (Eigen::Index i = 0; i < n; ++i) {
        if (matrix(i, i) <= 0.0f) {
            return false;
        }
    }
    return true;
}

Eigen::Matrix3f skew(const Eigen::Vector3f& v) {
    Eigen::Matrix3f s;
    s << 0.0f, -v.z(), v.y(),
         v.z(), 0.0f, -v.x(),
         -v.y(), v.x(), 0.0f;
    return s;
}

void test_core_matrix_ops() {
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    CHECK(coeffs_near(I * Eigen::Vector3f(1.0f, 2.0f, 3.0f),
                      Eigen::Vector3f(1.0f, 2.0f, 3.0f)));

    Eigen::Matrix3f A;
    A << 1.0f, 2.0f, 3.0f,
         0.0f, 1.0f, 4.0f,
         5.0f, 6.0f, 0.0f;
    Eigen::Matrix3f expected_sum;
    expected_sum << 2.0f, 2.0f, 3.0f,
                    0.0f, 2.0f, 4.0f,
                    5.0f, 6.0f, 1.0f;
    CHECK(coeffs_near(A + I, expected_sum));

    CHECK(coeffs_near(A * Eigen::Vector3f(1.0f, 0.0f, -1.0f),
                      Eigen::Vector3f(-2.0f, -4.0f, 5.0f)));
    CHECK(near_float(A.transpose()(1, 0), A(0, 1)));

    Eigen::Matrix2f expected_block;
    expected_block << 1.0f, 2.0f,
                      0.0f, 1.0f;
    CHECK(coeffs_near(A.block<2, 2>(0, 0), expected_block));

    float raw[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    Eigen::Map<Eigen::Matrix<float, 2, 3>> mapped(raw);
    CHECK(mapped(0, 0) == 1.0f && mapped(1, 2) == 6.0f);

    Eigen::Matrix3f upper = Eigen::Matrix3f::Ones();
    upper.triangularView<Eigen::StrictlyLower>().setZero();
    CHECK(upper(2, 0) == 0.0f && upper(0, 2) == 1.0f);

    A.diagonal().setConstant(2.0f);
    CHECK(A(0, 0) == 2.0f && A(2, 2) == 2.0f);

    Eigen::VectorXf dyn = Eigen::VectorXf::LinSpaced(6, 0.0f, 5.0f);
    CHECK(coeffs_near(dyn.segment(2, 3), Eigen::Vector3f(2.0f, 3.0f, 4.0f)));

    Eigen::Matrix2f M;
    M << 4.0f, 7.0f,
         2.0f, 6.0f;
    CHECK(coeffs_near(M * M.inverse(), Eigen::Matrix2f::Identity(), 1e-4f));
}

void test_ins_covariance_and_gnss_update() {
    using Mat15 = Eigen::Matrix<float, 15, 15>;
    using Vec15 = Eigen::Matrix<float, 15, 1>;
    using Mat4x15 = Eigen::Matrix<float, 4, 15>;
    using Mat4 = Eigen::Matrix4f;
    using Vec4 = Eigen::Vector4f;

    const float dt = 0.1f;
    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;

    Vec15 x = Vec15::Zero();
    x.segment<3>(0) = Eigen::Vector3f(10.0f, 20.0f, 30.0f);
    x.segment<3>(3) = Eigen::Vector3f(1.0f, -2.0f, 0.5f);
    Vec15 x_pred = F * x;
    CHECK(coeffs_near(x_pred.segment<3>(0), Eigen::Vector3f(10.1f, 19.8f, 30.05f), 1e-5f));
    CHECK(coeffs_near(x_pred.segment<3>(3), x.segment<3>(3)));

    Mat15 P = Mat15::Identity() * 0.1f;
    P.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 4.0f;
    P.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.2f;
    Mat15 Q = Mat15::Identity() * 0.01f;
    Mat15 P_pred = F * P * F.transpose() + Q;
    CHECK(coeffs_near(P_pred, P_pred.transpose(), 1e-5f));
    CHECK(diagonal_is_positive(P_pred));

    Mat15 product = Mat15::Zero();
    product.noalias() = F * P;
    CHECK(coeffs_near(product, F * P));

    Mat4x15 H = Mat4x15::Zero();
    H.block<1, 3>(0, 0) = -Eigen::Vector3f(1.0f, 0.0f, 0.0f).transpose();
    H.block<1, 3>(1, 0) = -Eigen::Vector3f(0.0f, 1.0f, 0.0f).transpose();
    H.block<1, 3>(2, 0) = -Eigen::Vector3f(0.0f, 0.0f, 1.0f).transpose();
    H.block<1, 3>(3, 0) = -Eigen::Vector3f(0.57735026f, 0.57735026f, 0.57735026f).transpose();
    H.col(9).setOnes();

    Vec15 dx = Vec15::Zero();
    dx.segment<3>(0) = Eigen::Vector3f(1.0f, 2.0f, -1.0f);
    dx(9) = 0.5f;
    Vec4 innovation = H * dx;
    CHECK(coeffs_near(innovation, Vec4(-0.5f, -1.5f, 1.5f, -0.6547005f), 1e-5f));

    Mat4 R = Mat4::Identity() * 2.0f;
    Mat4 S = H * P_pred * H.transpose() + R;
    CHECK(coeffs_near(S * S.inverse(), Mat4::Identity(), 1e-3f));

    Eigen::Matrix<float, 15, 4> K = P_pred * H.transpose() * S.inverse();
    CHECK(K.rows() == 15 && K.cols() == 4 && matrix_is_finite(K));

    Mat15 I15 = Mat15::Identity();
    Mat15 P_update = (I15 - K * H) * P_pred * (I15 - K * H).transpose() + K * R * K.transpose();
    CHECK(coeffs_near(P_update, P_update.transpose(), 1e-4f));
    CHECK(diagonal_is_positive(P_update));
}

void test_lu_and_cholesky() {
    Eigen::Matrix3f A;
    A << 1.0f, 2.0f, 3.0f,
         0.0f, 1.0f, 4.0f,
         5.0f, 6.0f, 0.0f;
    Eigen::Vector3f b(1.0f, 2.0f, 3.0f);

    Eigen::PartialPivLU<Eigen::Matrix3f> lu(A);
    CHECK(coeffs_near(A * lu.solve(b), b, 1e-4f));
    CHECK(coeffs_near(A * lu.inverse(), Eigen::Matrix3f::Identity(), 1e-4f));
    CHECK(std::abs(A.determinant() - 1.0f) < 1e-4f);

    Eigen::Matrix3f P;
    P << 4.0f, 1.0f, 0.0f,
         1.0f, 3.0f, 1.0f,
         0.0f, 1.0f, 2.0f;
    Eigen::LLT<Eigen::Matrix3f> llt(P);
    CHECK(llt.info() == Eigen::Success);

    Eigen::Matrix3f L = llt.matrixL();
    CHECK(coeffs_near(L * L.transpose(), P, 1e-4f));
    CHECK(coeffs_near(P * llt.solve(b), b, 1e-4f));
    CHECK(P.selfadjointView<Eigen::Lower>().llt().info() == Eigen::Success);
}

void test_quaternion_and_attitude_ops() {
    Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
    CHECK(near_float(identity.w(), 1.0f) && near_float(identity.vec().norm(), 0.0f));

    Eigen::Quaternionf q_yaw(Eigen::AngleAxisf(kPi * 0.5f, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f y_axis = q_yaw * Eigen::Vector3f::UnitX();
    CHECK(coeffs_near(y_axis, Eigen::Vector3f::UnitY(), 1e-4f));

    CHECK(coeffs_near(q_yaw.conjugate() * y_axis, Eigen::Vector3f::UnitX(), 1e-4f));

    Eigen::Quaternionf q_pitch(Eigen::AngleAxisf(kPi / 6.0f, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf q_body_nav = (q_yaw * q_pitch).normalized();
    CHECK(near_float(q_body_nav.norm(), 1.0f, 1e-5f));

    Eigen::Matrix3f R = q_body_nav.toRotationMatrix();
    CHECK(coeffs_near(R.transpose() * R, Eigen::Matrix3f::Identity(), 1e-4f));

    Eigen::Vector3f lever_arm_body(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f lever_arm_nav = q_yaw * lever_arm_body;
    CHECK(coeffs_near(lever_arm_nav, Eigen::Vector3f::UnitY(), 1e-4f));

    Eigen::Quaternionf q_from = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY());
    CHECK(coeffs_near(q_from * Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 1e-4f));

    Eigen::Quaternionf q_mid = Eigen::Quaternionf::Identity().slerp(0.5f, q_yaw);
    CHECK(std::abs(Eigen::AngleAxisf(q_mid).angle() - kPi * 0.25f) < 1e-4f);

    Eigen::Vector3f dtheta(0.0f, 0.0f, 0.01f);
    Eigen::Quaternionf dq(Eigen::AngleAxisf(dtheta.norm(), dtheta.normalized()));
    CHECK(coeffs_near(dq * Eigen::Vector3f::UnitX(), Eigen::Vector3f(0.99995f, 0.00999983f, 0.0f), 1e-4f));
}

void test_cross_products_and_alignment() {
    Eigen::Vector3f a(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f b(-4.0f, 5.0f, -6.0f);
    CHECK(coeffs_near(skew(a) * b, a.cross(b)));
    CHECK(coeffs_near(skew(a).transpose(), -skew(a)));

    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
    vec.push_back(Eigen::Vector4f(1.0f, 2.0f, 3.0f, 4.0f));
    CHECK(coeffs_near(vec.front(), Eigen::Vector4f(1.0f, 2.0f, 3.0f, 4.0f)));

    std::deque<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f>> dq;
    dq.push_back(Eigen::Matrix2f::Identity());
    CHECK(coeffs_near(dq.front(), Eigen::Matrix2f::Identity()));

    std::list<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lst;
    lst.push_back(Eigen::Vector3f::Zero());
    CHECK(coeffs_near(lst.front(), Eigen::Vector3f::Zero()));
}

} // namespace

int main() {
    test_core_matrix_ops();
    test_ins_covariance_and_gnss_update();
    test_lu_and_cholesky();
    test_quaternion_and_attitude_ops();
    test_cross_products_and_alignment();

    std::cout << "eigen3: " << (g_total - g_failures) << " / " << g_total << " passed\n";
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
