/**
 * @file test_eigen3.cpp
 * @brief Smoke test for the SRKF embedded Eigen3 profile.
 *
 * Covered modules:
 *   Core, LU, Cholesky, QR, Householder/Jacobi dependencies, Geometry,
 *   StdDeque/StdList/StdVector.
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

static void test_core_basics() {
    SECTION("Core: matrices, vectors, blocks, maps");

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    VERIFY(I(0, 0) == 1.0f && I(2, 2) == 1.0f);

    Eigen::Vector3f x(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f y(0.0f, 1.0f, 0.0f);
    VERIFY(x.dot(y) == 0.0f);
    VERIFY(x.cross(y).isApprox(Eigen::Vector3f::UnitZ()));

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
    VERIFY(C.isApprox(A * B));

    Eigen::Matrix3f U = Eigen::Matrix3f::Ones();
    U.triangularView<Eigen::StrictlyLower>().setZero();
    VERIFY(U(2, 0) == 0.0f && U(0, 2) == 1.0f);

    A.diagonal().setConstant(2.0f);
    VERIFY(A(0, 0) == 2.0f && A(2, 2) == 2.0f);

    Eigen::Matrix3f Acopy = A;
    A.transposeInPlace();
    VERIFY(A.isApprox(Acopy.transpose()));

    Eigen::VectorXf v = Eigen::VectorXf::LinSpaced(6, 0.0f, 5.0f);
    VERIFY(v.segment(2, 3).isApprox(Eigen::Vector3f(2.0f, 3.0f, 4.0f)));

    Eigen::MatrixXf dyn(2, 3);
    dyn.setConstant(2.0f);
    VERIFY(dyn.rows() == 2 && dyn.cols() == 3 && dyn.sum() == 12.0f);
}

static void test_geometry_navigation() {
    SECTION("Geometry: quaternion and transforms");

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    VERIFY(q.isApprox(Eigen::Quaternionf(1, 0, 0, 0)));

    Eigen::Quaternionf qz(Eigen::AngleAxisf(kPi * 0.5f, Eigen::Vector3f::UnitZ()));
    VERIFY((qz * Eigen::Vector3f::UnitX()).isApprox(Eigen::Vector3f::UnitY(), 1e-4f));

    Eigen::Quaternionf q_from = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitX());
    VERIFY((q_from * Eigen::Vector3f::UnitX()).isApprox(-Eigen::Vector3f::UnitX(), 1e-4f));

    Eigen::Quaternionf qmid = Eigen::Quaternionf::Identity().slerp(0.5f, qz);
    VERIFY(std::abs(Eigen::AngleAxisf(qmid).angle() - kPi * 0.25f) < 1e-4f);

    Eigen::Quaternionf qprod = qz * qz.conjugate();
    VERIFY(qprod.isApprox(Eigen::Quaternionf::Identity(), 1e-4f));

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translate(Eigen::Vector3f(1, 2, 3));
    T.rotate(qz);
    Eigen::Vector3f p = T * Eigen::Vector3f::UnitX();
    VERIFY(p.isApprox(Eigen::Vector3f(1, 3, 3), 1e-4f));
    VERIFY((T.inverse() * p).isApprox(Eigen::Vector3f::UnitX(), 1e-4f));

    Eigen::Matrix3f R = (Eigen::AngleAxisf(kPi / 6.0f, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(kPi / 5.0f, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(kPi / 4.0f, Eigen::Vector3f::UnitZ()))
                            .toRotationMatrix();
    VERIFY(R.isUnitary(1e-4f));

    Eigen::Rotation2Df r2(kPi * 0.5f);
    VERIFY((r2 * Eigen::Vector2f::UnitX()).isApprox(Eigen::Vector2f::UnitY(), 1e-4f));

    Eigen::Transform<float, 3, Eigen::Affine> scaled =
        Eigen::Translation3f(1, 2, 3) * Eigen::UniformScaling<float>(2.0f);
    VERIFY((scaled * Eigen::Vector3f::Ones()).isApprox(Eigen::Vector3f(3, 4, 5)));
}

static void test_lu() {
    SECTION("LU: inverse, determinant, solves");

    Eigen::Matrix3f A;
    A << 1, 2, 3,
         0, 1, 4,
         5, 6, 0;
    Eigen::Vector3f b(1, 2, 3);

    Eigen::PartialPivLU<Eigen::Matrix3f> lu(A);
    VERIFY((A * lu.solve(b)).isApprox(b, 1e-4f));
    VERIFY((A * lu.inverse()).isApprox(Eigen::Matrix3f::Identity(), 1e-4f));
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
    VERIFY((L * L.transpose()).isApprox(P, 1e-4f));
    VERIFY((P * llt.solve(b)).isApprox(b, 1e-4f));

    Eigen::LDLT<Eigen::Matrix3f> ldlt(P);
    VERIFY(ldlt.info() == Eigen::Success);
    VERIFY(ldlt.isPositive());
    VERIFY((P * ldlt.solve(b)).isApprox(b, 1e-4f));

    VERIFY(P.selfadjointView<Eigen::Lower>().llt().info() == Eigen::Success);
}

static void test_qr_and_srkf_update() {
    SECTION("QR: square-root Kalman filter primitives");

    Eigen::Matrix<float, 5, 3> H;
    H << 1, 0, 0,
         0, 1, 0,
         0, 0, 1,
         1, 1, 0,
         0, 1, 1;

    Eigen::HouseholderQR<Eigen::Matrix<float, 5, 3>> hqr(H);
    Eigen::Matrix<float, 5, 3> Q = hqr.householderQ() * Eigen::Matrix<float, 5, 3>::Identity();
    Eigen::Matrix3f R = hqr.matrixQR().topRows<3>().triangularView<Eigen::Upper>();
    VERIFY((Q.transpose() * Q).isApprox(Eigen::Matrix3f::Identity(), 1e-4f));
    VERIFY((Q * R).isApprox(H, 1e-4f));

    Eigen::Matrix<float, 5, 1> z;
    z << 1, 2, 3, 4, 5;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<float, 5, 3>> cpqr(H);
    Eigen::Vector3f x = cpqr.solve(z);
    VERIFY((H.transpose() * (H * x - z)).norm() < 1e-4f);
    VERIFY(cpqr.rank() == 3);

    Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
    S(0, 0) = 0.8f;
    S(1, 1) = 0.6f;
    S(2, 2) = 0.4f;

    Eigen::Matrix3f Qsqrt = Eigen::Matrix3f::Zero();
    Qsqrt.diagonal() << 0.1f, 0.2f, 0.3f;

    Eigen::Matrix<float, 6, 3> stacked;
    stacked << S,
               Qsqrt;
    Eigen::HouseholderQR<Eigen::Matrix<float, 6, 3>> sr_qr(stacked);
    Eigen::Matrix3f S_pred = sr_qr.matrixQR().topRows<3>().triangularView<Eigen::Upper>();
    VERIFY(S_pred.rows() == 3 && S_pred.cols() == 3);
    const Eigen::Matrix3f predicted_cov = (S_pred.transpose() * S_pred).eval();
    const Eigen::Matrix3f reference_cov = (stacked.transpose() * stacked).eval();
    VERIFY(predicted_cov.isApprox(reference_cov, 1e-4f));
    VERIFY(S_pred.array().isFinite().all());
    VERIFY(std::abs(S_pred(2, 0)) < 1e-6f && std::abs(S_pred(2, 1)) < 1e-6f);
}

static void test_stl_support_and_edges() {
    SECTION("STL support and edge cases");

    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
    vec.push_back(Eigen::Vector4f(1, 2, 3, 4));
    VERIFY(vec.front().isApprox(Eigen::Vector4f(1, 2, 3, 4)));

    std::deque<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f>> dq;
    dq.push_back(Eigen::Matrix2f::Identity());
    VERIFY(dq.front().isApprox(Eigen::Matrix2f::Identity()));

    std::list<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lst;
    lst.push_back(Eigen::Vector3f::Zero());
    VERIFY(lst.front().isZero());

    Eigen::MatrixXf empty(0, 0);
    VERIFY(empty.size() == 0);

    Eigen::Matrix<float, 15, 15> F = Eigen::Matrix<float, 15, 15>::Identity();
    Eigen::Matrix<float, 15, 1> state = Eigen::Matrix<float, 15, 1>::Ones();
    VERIFY((F * state).isApprox(state));

    Eigen::Matrix3f P = Eigen::Matrix3f::Identity();
    P.noalias() += Eigen::Vector3f(1, 2, 3) * Eigen::Vector3f(1, 2, 3).transpose();
    VERIFY(P.isApprox(P.transpose()));
}

int main() {
    std::cout << "===========================================\n";
    std::cout << "  Eigen3 SRKF Embedded Profile Test\n";
    std::cout << "===========================================\n\n";

    test_core_basics();
    test_geometry_navigation();
    test_lu();
    test_cholesky();
    test_qr_and_srkf_update();
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
