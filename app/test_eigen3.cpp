/**
 * @file test_eigen3.cpp
 * @brief Smoke test for the SRKF embedded Eigen3 profile.
 *
 * Covered modules:
 *   Core, LU, LLT Cholesky, Geometry, StdDeque/StdList/StdVector.
 *
 * This board smoke test intentionally avoids QR, LDLT, COD, and Eigen::isApprox()
 * because the native i.MX93 GCC 13.3 toolchain has shown ICEs on those template
 * paths. The production library still keeps the relevant headers.
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

static int g_failures = 0;
static int g_total = 0;

#define VERIFY(expr)                                                          \
    do {                                                                      \
        ++g_total;                                                            \
        if (!(expr)) {                                                        \
            ++g_failures;                                                     \
            std::cerr << "  FAIL [" << __LINE__ << "]: " << #expr << "\n"; \
        }                                                                     \
    } while (0)

#define SECTION(title) std::cout << "[TEST] " << (title) << "\n"

static constexpr float kPi = 3.14159265358979323846f;

template <typename Lhs, typename Rhs>
bool coeffs_near(const Eigen::MatrixBase<Lhs>& lhs,
                 const Eigen::MatrixBase<Rhs>& rhs,
                 float eps = 1e-5f) {
    if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
        return false;
    }

    for (Eigen::Index r = 0; r < lhs.rows(); ++r) {
        for (Eigen::Index c = 0; c < lhs.cols(); ++c) {
            if (std::abs(lhs(r, c) - rhs(r, c)) > eps) {
                return false;
            }
        }
    }
    return true;
}

template <typename QuaternionType>
bool quaternion_near(const QuaternionType& lhs,
                     const QuaternionType& rhs,
                     float eps = 1e-5f) {
    return std::abs(lhs.w() - rhs.w()) <= eps &&
           std::abs(lhs.x() - rhs.x()) <= eps &&
           std::abs(lhs.y() - rhs.y()) <= eps &&
           std::abs(lhs.z() - rhs.z()) <= eps;
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

static void test_core_basics() {
    SECTION("Core: matrices, vectors, blocks, maps");

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    VERIFY(I(0, 0) == 1.0f && I(2, 2) == 1.0f);

    Eigen::Vector3f x(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f y(0.0f, 1.0f, 0.0f);
    VERIFY(x.dot(y) == 0.0f);
    VERIFY(coeffs_near(x.cross(y), Eigen::Vector3f::UnitZ()));

    Eigen::Matrix4f M = Eigen::Matrix4f::Random();
    Eigen::Matrix2f corner = M.block<2, 2>(0, 0);
    VERIFY(corner(1, 1) == M(1, 1));

    float raw[] = {1, 2, 3, 4, 5, 6};
    Eigen::Map<Eigen::Matrix<float, 2, 3>> mapped(raw);
    VERIFY(mapped(0, 0) == 1.0f && mapped(1, 2) == 6.0f);

    Eigen::Matrix3f A = Eigen::Matrix3f::Random();
    Eigen::Matrix3f B = Eigen::Matrix3f::Random();
    Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
    C.noalias() = A * B;
    Eigen::Matrix3f Cref = A * B;
    VERIFY(coeffs_near(C, Cref));

    Eigen::Matrix3f U = Eigen::Matrix3f::Ones();
    U.triangularView<Eigen::StrictlyLower>().setZero();
    VERIFY(U(2, 0) == 0.0f && U(0, 2) == 1.0f);

    A.diagonal().setConstant(2.0f);
    VERIFY(A(0, 0) == 2.0f && A(2, 2) == 2.0f);

    Eigen::Matrix3f Acopy = A;
    A.transposeInPlace();
    Eigen::Matrix3f AcopyT = Acopy.transpose();
    VERIFY(coeffs_near(A, AcopyT));

    Eigen::VectorXf v = Eigen::VectorXf::LinSpaced(6, 0.0f, 5.0f);
    VERIFY(coeffs_near(v.segment(2, 3), Eigen::Vector3f(2.0f, 3.0f, 4.0f)));

    Eigen::MatrixXf dyn(2, 3);
    dyn.setConstant(2.0f);
    VERIFY(dyn.rows() == 2 && dyn.cols() == 3 && dyn.sum() == 12.0f);
}

static void test_geometry_navigation() {
    SECTION("Geometry: quaternion and transforms");

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    VERIFY(quaternion_near(q, Eigen::Quaternionf(1, 0, 0, 0)));

    Eigen::Quaternionf qz(Eigen::AngleAxisf(kPi * 0.5f, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f rotated_x = qz * Eigen::Vector3f::UnitX();
    VERIFY(coeffs_near(rotated_x, Eigen::Vector3f::UnitY(), 1e-4f));

    Eigen::Quaternionf q_from = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX());
    Eigen::Vector3f opposite_x = -Eigen::Vector3f::UnitX();
    Eigen::Vector3f rotated_to_opposite = q_from * Eigen::Vector3f::UnitX();
    VERIFY(coeffs_near(rotated_to_opposite, opposite_x, 1e-4f));

    Eigen::Quaternionf qmid = Eigen::Quaternionf::Identity().slerp(0.5f, qz);
    VERIFY(std::abs(Eigen::AngleAxisf(qmid).angle() - kPi * 0.25f) < 1e-4f);

    Eigen::Quaternionf qprod = qz * qz.conjugate();
    VERIFY(quaternion_near(qprod, Eigen::Quaternionf::Identity(), 1e-4f));

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translate(Eigen::Vector3f(1, 2, 3));
    T.rotate(qz);
    Eigen::Vector3f p = T * Eigen::Vector3f::UnitX();
    VERIFY(coeffs_near(p, Eigen::Vector3f(1, 3, 3), 1e-4f));
    Eigen::Vector3f p_back = T.inverse() * p;
    VERIFY(coeffs_near(p_back, Eigen::Vector3f::UnitX(), 1e-4f));

    Eigen::Matrix3f R = (Eigen::AngleAxisf(kPi / 6.0f, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(kPi / 5.0f, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(kPi / 4.0f, Eigen::Vector3f::UnitZ()))
                            .toRotationMatrix();
    Eigen::Matrix3f RtR = R.transpose() * R;
    VERIFY(coeffs_near(RtR, Eigen::Matrix3f::Identity(), 1e-4f));

    Eigen::Rotation2Df r2(kPi * 0.5f);
    Eigen::Vector2f rotated_2d = r2 * Eigen::Vector2f::UnitX();
    VERIFY(coeffs_near(rotated_2d, Eigen::Vector2f::UnitY(), 1e-4f));

    Eigen::Transform<float, 3, Eigen::Affine> scaled =
        Eigen::Translation3f(1, 2, 3) * Eigen::UniformScaling<float>(2.0f);
    Eigen::Vector3f scaled_point = scaled * Eigen::Vector3f::Ones();
    VERIFY(coeffs_near(scaled_point, Eigen::Vector3f(3, 4, 5)));
}

static void test_lu() {
    SECTION("LU: inverse, determinant, solves");

    Eigen::Matrix3f A;
    A << 1, 2, 3,
         0, 1, 4,
         5, 6, 0;
    Eigen::Vector3f b(1, 2, 3);

    Eigen::PartialPivLU<Eigen::Matrix3f> lu(A);
    Eigen::Vector3f solved_b = A * lu.solve(b);
    VERIFY(coeffs_near(solved_b, b, 1e-4f));
    Eigen::Matrix3f inverse_check = A * lu.inverse();
    VERIFY(coeffs_near(inverse_check, Eigen::Matrix3f::Identity(), 1e-4f));
    VERIFY(std::abs(A.determinant() - 1.0f) < 1e-4f);

    Eigen::FullPivLU<Eigen::Matrix3f> full(A);
    VERIFY(full.rank() == 3 && full.isInvertible());

    Eigen::Matrix3f singular;
    singular << 1, 2, 3,
                2, 4, 6,
                0, 0, 0;
    VERIFY(Eigen::FullPivLU<Eigen::Matrix3f>(singular).rank() == 1);
}

static void test_cholesky() {
    SECTION("Cholesky: covariance square roots");

    Eigen::Matrix3f P;
    P << 4, 1, 0,
         1, 3, 1,
         0, 1, 2;
    Eigen::Vector3f b(1, 2, 3);

    Eigen::LLT<Eigen::Matrix3f> llt(P);
    VERIFY(llt.info() == Eigen::Success);
    Eigen::Matrix3f L = llt.matrixL();
    Eigen::Matrix3f P_reconstructed = L * L.transpose();
    VERIFY(coeffs_near(P_reconstructed, P, 1e-4f));
    Eigen::Vector3f llt_solved_b = P * llt.solve(b);
    VERIFY(coeffs_near(llt_solved_b, b, 1e-4f));

    VERIFY(P.selfadjointView<Eigen::Lower>().llt().info() == Eigen::Success);
}

static void test_stl_support_and_edges() {
    SECTION("STL support and edge cases");

    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
    vec.push_back(Eigen::Vector4f(1, 2, 3, 4));
    VERIFY(coeffs_near(vec.front(), Eigen::Vector4f(1, 2, 3, 4)));

    std::deque<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f>> dq;
    dq.push_back(Eigen::Matrix2f::Identity());
    VERIFY(coeffs_near(dq.front(), Eigen::Matrix2f::Identity()));

    std::list<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lst;
    lst.push_back(Eigen::Vector3f::Zero());
    VERIFY(coeffs_near(lst.front(), Eigen::Vector3f::Zero()));

    Eigen::MatrixXf empty(0, 0);
    VERIFY(empty.size() == 0);

    Eigen::Matrix<float, 15, 15> F = Eigen::Matrix<float, 15, 15>::Identity();
    Eigen::Matrix<float, 15, 1> state = Eigen::Matrix<float, 15, 1>::Ones();
    Eigen::Matrix<float, 15, 1> propagated_state = F * state;
    VERIFY(coeffs_near(propagated_state, state));

    Eigen::Matrix3f P = Eigen::Matrix3f::Identity();
    P.noalias() += Eigen::Vector3f(1, 2, 3) * Eigen::Vector3f(1, 2, 3).transpose();
    Eigen::Matrix3f Pt = P.transpose();
    VERIFY(coeffs_near(P, Pt));
}

int main() {
    std::cout << "===========================================\n";
    std::cout << "  Eigen3 SRKF Embedded Profile Test\n";
    std::cout << "===========================================\n\n";

    test_core_basics();
    test_geometry_navigation();
    test_lu();
    test_cholesky();
    test_stl_support_and_edges();

    std::cout << "\n===========================================\n";
    std::cout << "  Results: " << (g_total - g_failures) << " / " << g_total
              << " passed";
    if (g_failures > 0) {
        std::cout << "  (" << g_failures << " FAILURES)";
    }
    std::cout << "\n===========================================\n";

    return g_failures > 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
