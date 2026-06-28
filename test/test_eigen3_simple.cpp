#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdlib>
#include <iostream>

static int g_total = 0;
static int g_failures = 0;

#define CHECK(expr)                                                           \
    do {                                                                      \
        ++g_total;                                                            \
        if (!(expr)) {                                                        \
            ++g_failures;                                                     \
            std::cerr << "FAIL [" << __LINE__ << "]: " << #expr << "\n";     \
        }                                                                     \
    } while (0)

static bool near_float(float a, float b, float eps = 1e-5f) {
    return std::abs(a - b) <= eps;
}

template <typename Lhs, typename Rhs>
static bool near_matrix(const Eigen::MatrixBase<Lhs>& lhs,
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

static void test_matrix_ops() {
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    CHECK(near_matrix(I, Eigen::Matrix3f::Identity()));

    Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
    CHECK(near_matrix(I * v, v));

    Eigen::Matrix3f A;
    A << 1.0f, 2.0f, 3.0f,
         0.0f, 1.0f, 4.0f,
         5.0f, 6.0f, 0.0f;
    Eigen::Matrix3f expected_sum;
    expected_sum << 2.0f, 2.0f, 3.0f,
                    0.0f, 2.0f, 4.0f,
                    5.0f, 6.0f, 1.0f;
    CHECK(near_matrix(A + I, expected_sum));

    Eigen::Vector3f Av = A * Eigen::Vector3f(1.0f, 0.0f, -1.0f);
    CHECK(near_matrix(Av, Eigen::Vector3f(-2.0f, -4.0f, 5.0f)));

    CHECK(near_float(A.transpose()(1, 0), A(0, 1)));

    Eigen::Matrix2f block = A.block<2, 2>(0, 0);
    Eigen::Matrix2f expected_block;
    expected_block << 1.0f, 2.0f,
                      0.0f, 1.0f;
    CHECK(near_matrix(block, expected_block));

    Eigen::Matrix2f M;
    M << 4.0f, 7.0f,
         2.0f, 6.0f;
    Eigen::Matrix2f M_inv = M.inverse();
    CHECK(near_matrix(M * M_inv, Eigen::Matrix2f::Identity(), 1e-4f));
}

static void test_quaternion_ops() {
    const float pi = 3.14159265358979323846f;

    Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
    CHECK(near_float(identity.w(), 1.0f));

    Eigen::Quaternionf qz(Eigen::AngleAxisf(pi * 0.5f, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f rotated = qz * Eigen::Vector3f::UnitX();
    CHECK(near_matrix(rotated, Eigen::Vector3f::UnitY(), 1e-4f));

    Eigen::Quaternionf q_inv = qz.conjugate();
    Eigen::Vector3f restored = q_inv * rotated;
    CHECK(near_matrix(restored, Eigen::Vector3f::UnitX(), 1e-4f));

    Eigen::Quaternionf q_from = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY());
    CHECK(near_matrix(q_from * Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), 1e-4f));
}

int main() {
    test_matrix_ops();
    test_quaternion_ops();

    std::cout << "simple: " << (g_total - g_failures) << " / "
              << g_total << " passed\n";
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
