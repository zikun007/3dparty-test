/**
 * @file test_eigen3.cpp
 * @brief Comprehensive test for the trimmed Eigen3 (Dense-only) library.
 *
 * Tests all retained modules:
 *   Core, Geometry, LU, Cholesky, QR, SVD, Eigenvalues,
 *   Householder, Jacobi, StdDeque/StdList/StdVector
 *
 * Each test section uses static_assert or runtime assertions (isApprox / VERIFY).
 * A non-zero exit code signals failure.
 */

#include <Eigen/Eigen>
#include <Eigen/StdDeque>
#include <Eigen/StdList>
#include <Eigen/StdVector>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <list>
#include <vector>

// ---------------------------------------------------------------------------
// Tiny test harness
// ---------------------------------------------------------------------------
static int  g_failures = 0;
static int  g_total    = 0;

#define VERIFY(expr)                                                          \
    do {                                                                      \
        ++g_total;                                                            \
        if (!(expr)) {                                                        \
            ++g_failures;                                                     \
            std::cerr << "  FAIL [" << __LINE__ << "]: " << #expr << "\n";    \
        }                                                                     \
    } while (0)

#define SECTION(title)  std::cout << "[TEST] " << (title) << "\n"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
template <typename T>
bool is_approx(const T& a, const T& b, double eps = 1e-4) {
    return a.isApprox(b, eps);
}

// ===========================================================================
// 1. Core — basic matrix / vector / array operations
// ===========================================================================
static void test_core_basics() {
    SECTION("Core: basic types and construction");

    // Fixed-size
    Eigen::Matrix3f m33 = Eigen::Matrix3f::Identity();
    VERIFY(m33(0,0) == 1.0f && m33(0,1) == 0.0f && m33(2,2) == 1.0f);

    Eigen::Matrix4d m44 = Eigen::Matrix4d::Zero();
    VERIFY(m44.norm() == 0.0);

    Eigen::Matrix2i m2i;
    m2i << 1, 2, 3, 4;
    VERIFY(m2i(0,0) == 1 && m2i(1,1) == 4);

    // Dynamic-size
    Eigen::MatrixXd dyn(5, 7);
    dyn.setRandom();
    VERIFY(dyn.rows() == 5 && dyn.cols() == 7);

    // Vector
    Eigen::Vector3f v3(1.0f, 2.0f, 3.0f);
    VERIFY(v3.x() == 1.0f && v3.y() == 2.0f && v3.z() == 3.0f);

    Eigen::VectorXd vdyn(10);
    vdyn.setZero();
    VERIFY(vdyn.squaredNorm() == 0.0);

    // Array
    Eigen::ArrayXXf arr(3, 3);
    arr.setConstant(2.5f);
    VERIFY(std::abs(arr(0,0) - 2.5f) < 1e-6f);
}

static void test_core_arithmetic() {
    SECTION("Core: arithmetic and expression templates");

    Eigen::Matrix3f A;
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 10;
    Eigen::Matrix3f B;
    B << 0, 1, 2,
         3, 4, 5,
         6, 7, 8;

    // Addition / subtraction
    Eigen::Matrix3f C = A + B;
    VERIFY(C(0,0) == 1.0f && C(0,1) == 3.0f);
    Eigen::Matrix3f D = A - B;
    VERIFY(D(0,0) == 1.0f && D(2,2) == 2.0f);

    // Scalar ops
    Eigen::Matrix3f S = A * 2.0f;
    VERIFY(S(0,0) == 2.0f && S(2,2) == 20.0f);
    S = 2.0f * A;
    VERIFY(S(0,0) == 2.0f && S(2,2) == 20.0f);

    // Matrix product
    Eigen::Matrix3f P = A * B;
    // P(0,0) = 1*0+2*3+3*6 = 24
    VERIFY(std::abs(P(0,0) - 24.0f) < 1e-6f);
    // P(2,2) = 7*2+8*5+10*8 = 134
    VERIFY(std::abs(P(2,2) - 134.0f) < 1e-6f);

    // Matrix-vector
    Eigen::Vector3f v(1, 2, 3);
    Eigen::Vector3f w = A * v;
    // w(0) = 1*1+2*2+3*3 = 14
    VERIFY(std::abs(w(0) - 14.0f) < 1e-6f);

    // Transpose
    Eigen::Matrix3f At = A.transpose();
    VERIFY(At(0,1) == 4.0f && At(1,0) == 2.0f);

    // Dot / cross
    Eigen::Vector3f a(1, 0, 0), b(0, 1, 0);
    VERIFY(a.dot(b) == 0.0f);
    Eigen::Vector3f c = a.cross(b);
    VERIFY(c.isApprox(Eigen::Vector3f(0, 0, 1)));
}

static void test_core_block_ops() {
    SECTION("Core: block, segment, and view operations");

    Eigen::Matrix4f M = Eigen::Matrix4f::Random();

    // Block
    Eigen::Matrix2f blk = M.block<2,2>(0, 0);
    VERIFY(blk(0,0) == M(0,0) && blk(1,1) == M(1,1));

    // Dynamic block
    Eigen::MatrixXf blk_dyn = M.block(0, 0, 2, 3);
    VERIFY(blk_dyn.rows() == 2 && blk_dyn.cols() == 3);

    // Row / col
    Eigen::Vector4f row0 = M.row(0);
    VERIFY(row0(0) == M(0,0) && row0(3) == M(0,3));
    Eigen::Vector4f col2 = M.col(2);
    VERIFY(col2(0) == M(0,2) && col2(3) == M(3,2));

    // Segment (vector)
    Eigen::VectorXf vbig = Eigen::VectorXf::Random(10);
    Eigen::VectorXf seg3 = vbig.segment(2, 3);
    VERIFY(seg3.size() == 3 && seg3(0) == vbig(2));

    // head / tail
    Eigen::VectorXf head = vbig.head(4);
    VERIFY(head.size() == 4 && head(0) == vbig(0));
    Eigen::VectorXf tail = vbig.tail(3);
    VERIFY(tail.size() == 3 && tail(2) == vbig(9));

    // Map
    float raw[] = {1, 2, 3, 4, 5, 6};
    Eigen::Map<Eigen::Matrix<float, 2, 3>> mapped(raw);
    VERIFY(mapped(0,0) == 1 && mapped(1,2) == 6);

    Eigen::Map<Eigen::VectorXf> vmap(raw, 6);
    VERIFY(vmap(0) == 1 && vmap(5) == 6);

    // Ref — accepts both fixed and dynamic
    auto accept_ref = [](Eigen::Ref<const Eigen::MatrixXf> r) -> float {
        return r(0, 0);
    };
    VERIFY(accept_ref(M) == M(0,0));
    VERIFY(accept_ref(blk_dyn) == blk_dyn(0,0));
}

static void test_core_advanced() {
    SECTION("Core: reductions, visitors, and utilities");

    Eigen::Matrix3f R = Eigen::Matrix3f::Random();

    // Reductions
    float mn = R.minCoeff();
    float mx = R.maxCoeff();
    VERIFY(mn <= mx);

    Eigen::Vector3f::Index minRow, minCol;
    R.minCoeff(&minRow, &minCol);
    VERIFY(R(minRow, minCol) == mn);

    // Norms
    float nrm2 = R.norm();           // Frobenius
    float nrm1 = R.lpNorm<1>();
    float nrmInf = R.lpNorm<Eigen::Infinity>();
    VERIFY(nrm2 >= 0 && nrm1 >= 0 && nrmInf >= 0);

    // Boolean reductions
    Eigen::Matrix3f P = Eigen::Matrix3f::Ones();
    VERIFY(P.all());
    VERIFY(!(P - P).any());
    VERIFY((P.array() > 0.5f).all());

    // Diagonal
    Eigen::Vector3f diag = R.diagonal();
    VERIFY(diag(0) == R(0,0) && diag(1) == R(1,1));
    R.diagonal().setConstant(7.0f);
    VERIFY(R(0,0) == 7.0f && R(2,2) == 7.0f);

    // Triangular view
    Eigen::Matrix3f U = Eigen::Matrix3f::Ones();
    U.triangularView<Eigen::StrictlyUpper>().setZero();
    VERIFY(U(1,0) == 1.0f && U(0,1) == 0.0f);

    // SelfAdjointView
    Eigen::Matrix3f S = Eigen::Matrix3f::Random();
    S = S * S.transpose();  // make symmetric
    Eigen::SelfAdjointView<Eigen::Matrix3f, Eigen::Lower> symView(S);
    VERIFY(symView.rows() == 3);

    // Random (deterministic via srand)
    Eigen::Matrix3f r1 = Eigen::Matrix3f::Random();
    VERIFY(r1.rows() == 3 && r1.cols() == 3);
}

static void test_core_comma_init() {
    SECTION("Core: comma initializer");

    Eigen::Matrix3f m;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    VERIFY(m(0,0) == 1 && m(1,1) == 5 && m(2,2) == 9);

    Eigen::Vector4f v;
    v << 10, 20, 30, 40;
    VERIFY(v(0) == 10 && v(3) == 40);
}

static void test_core_arithmetic_sequence() {
    SECTION("Core: ArithmeticSequence and indexing");

    Eigen::VectorXf v = Eigen::VectorXf::LinSpaced(10, 0, 9);
    VERIFY(v(0) == 0.0f && v(9) == 9.0f);

    // Eigen::seq (if available)
    #if EIGEN_HAS_CXX11
    Eigen::VectorXf sub = v(Eigen::seq(2, 5));
    VERIFY(sub.size() == 4 && sub(0) == 2.0f && sub(3) == 5.0f);
    #endif

    // Eigen::all (works with fixed-size + all, or with seqN)
    Eigen::VectorXf allv = v(Eigen::seqN(0, 10));
    VERIFY(allv.size() == 10);
}

// ===========================================================================
// 2. Geometry
// ===========================================================================
static void test_geometry_quaternion() {
    SECTION("Geometry: Quaternion");

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    VERIFY(q.isApprox(Eigen::Quaternionf(1, 0, 0, 0)));

    // From angle-axis
    Eigen::Quaternionf q2(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f v(1, 0, 0);
    Eigen::Vector3f r = q2 * v;
    VERIFY(r.isApprox(Eigen::Vector3f(0, 1, 0), 1e-4f));

    // From two vectors
    Eigen::Quaternionf q_from = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY());
    Eigen::Vector3f r2 = q_from * Eigen::Vector3f::UnitX();
    VERIFY(r2.normalized().isApprox(Eigen::Vector3f::UnitY(), 1e-4f));

    // Slerp
    Eigen::Quaternionf qa(Eigen::AngleAxisf(0.0f,          Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf qb(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf qmid = qa.slerp(0.5f, qb);
    // angle should be ~45 degrees about Z
    Eigen::AngleAxisf aa(qmid);
    VERIFY(std::abs(aa.angle() - M_PI / 4) < 1e-3f);

    // Conjugate / inverse
    Eigen::Quaternionf qrand = Eigen::Quaternionf::UnitRandom();
    Eigen::Quaternionf qprod = qrand * qrand.conjugate();
    VERIFY(qprod.isApprox(Eigen::Quaternionf::Identity(), 1e-4f));

    // Rotation of vector
    Eigen::Quaternionf qrot(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    Eigen::Vector3f vx(0, 1, 0);
    Eigen::Vector3f rx = qrot * vx;
    VERIFY(rx.isApprox(Eigen::Vector3f(0, -1, 0), 1e-4f));
}

static void test_geometry_transform() {
    SECTION("Geometry: Transform (Isometry / Affine)");

    // Isometry3f
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translate(Eigen::Vector3f(1, 2, 3));
    T.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()));

    Eigen::Vector3f pt(1, 0, 0);
    Eigen::Vector3f pt_t = T * pt;
    // rotated (0,1,0) then translated (1,2,3) => (1,3,3)
    VERIFY(pt_t.isApprox(Eigen::Vector3f(1, 3, 3), 1e-4f));

    // Inverse
    Eigen::Vector3f pt_back = T.inverse() * pt_t;
    VERIFY(pt_back.isApprox(pt, 1e-4f));

    // Affine3f
    Eigen::Affine3f A = Eigen::Affine3f::Identity();
    A.linear() = Eigen::Matrix3f::Identity() * 2.0f;
    A.translation() = Eigen::Vector3f(1, 0, 0);
    VERIFY((A * Eigen::Vector3f(1, 1, 1)).isApprox(Eigen::Vector3f(3, 2, 2), 1e-4f));

    // Projective (4x4 homogeneous matrix) — use homogeneous coords explicitly
    Eigen::Matrix4f PH = Eigen::Matrix4f::Identity();
    PH(0, 3) = 5.0f;
    Eigen::Vector4f pt_h(0, 0, 0, 1);
    Eigen::Vector4f result_h = PH * pt_h;
    VERIFY(std::abs(result_h(0) / result_h(3) - 5.0f) < 1e-4f);
}

static void test_geometry_other() {
    SECTION("Geometry: AngleAxis, Rotation2D, Translation, Scaling");

    // AngleAxis
    Eigen::AngleAxisf aa(M_PI / 3, Eigen::Vector3f(1, 1, 1).normalized());
    Eigen::Matrix3f R = aa.toRotationMatrix();
    VERIFY(R.isUnitary(1e-4f));
    VERIFY(std::abs(R.determinant() - 1.0f) < 1e-4f);

    // Rotation2D
    Eigen::Rotation2Df r2d(M_PI / 2);
    Eigen::Vector2f v2(1, 0);
    Eigen::Vector2f r2 = r2d * v2;
    VERIFY(r2.isApprox(Eigen::Vector2f(0, 1), 1e-4f));

    // Translation
    Eigen::Translation<float, 3> tr(1, 2, 3);
    VERIFY((tr * Eigen::Vector3f(0, 0, 0)).isApprox(Eigen::Vector3f(1, 2, 3)));

    // Scaling (UniformScaling) + non-uniform via DiagonalMatrix
    Eigen::UniformScaling<float> us(2.0f);
    VERIFY((us * Eigen::Vector3f(1, 1, 1)).isApprox(Eigen::Vector3f(2, 2, 2)));

    // Non-uniform scaling via DiagonalMatrix
    Eigen::DiagonalMatrix<float, 3> ds(2, 3, 4);
    VERIFY((ds * Eigen::Vector3f(1, 1, 1)).isApprox(Eigen::Vector3f(2, 3, 4)));

    // Combined: Translation * Rotation * UniformScaling
    // scale p=(1,0,0) by 2 → (2,0,0), rotate by π around Z → (-2,0,0), translate by (1,2,3) → (-1,2,3)
    Eigen::Transform<float, 3, Eigen::Affine> T =
        Eigen::Translation3f(1, 2, 3)
        * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ())
        * Eigen::UniformScaling<float>(2.0f);
    Eigen::Vector3f p(1, 0, 0);
    VERIFY((T * p).isApprox(Eigen::Vector3f(-1, 2, 3), 1e-4f));
}

static void test_geometry_aligned_box() {
    SECTION("Geometry: AlignedBox and Hyperplane");

    // AlignedBox
    Eigen::AlignedBox3f box;
    box.extend(Eigen::Vector3f(0, 0, 0));
    box.extend(Eigen::Vector3f(1, 2, 3));
    VERIFY(box.contains(Eigen::Vector3f(0.5f, 1.0f, 1.5f)));
    VERIFY(!box.contains(Eigen::Vector3f(-1, 0, 0)));
    VERIFY(box.sizes().isApprox(Eigen::Vector3f(1, 2, 3)));
    VERIFY(box.center().isApprox(Eigen::Vector3f(0.5f, 1.0f, 1.5f)));
    VERIFY(!box.isEmpty());
    VERIFY(box.volume() > 0);

    // Hyperplane
    Eigen::Hyperplane<float, 3> plane(Eigen::Vector3f::UnitZ(), Eigen::Vector3f(0, 0, 5));
    VERIFY(std::abs(plane.signedDistance(Eigen::Vector3f(10, 20, 5))) < 1e-6f);
    VERIFY(plane.signedDistance(Eigen::Vector3f(0, 0, 10)) > 0);

    // ParametrizedLine
    Eigen::ParametrizedLine<float, 3> line(
        Eigen::Vector3f(0, 0, 0), Eigen::Vector3f::UnitX());
    VERIFY(line.pointAt(5.0f).isApprox(Eigen::Vector3f(5, 0, 0)));
}

static void test_geometry_euler() {
    SECTION("Geometry: EulerAngles");

    // Build a rotation matrix
    Eigen::AngleAxisf rx(M_PI / 6, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf ry(0.0f,       Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rz(M_PI / 4, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R = (rz * ry * rx).toRotationMatrix();

    // eulerAngles(2,1,0) extracts ZYX: R = Rz(θ0) * Ry(θ1) * Rx(θ2)
    // Returns: (θ0, θ1, θ2) = (z_angle, y_angle, x_angle)
    Eigen::Vector3f euler_out = R.eulerAngles(2, 1, 0);
    // Reconstruct rotation matrix from extracted angles and compare
    Eigen::Matrix3f R_out = (Eigen::AngleAxisf(euler_out(0), Eigen::Vector3f::UnitZ())
                          * Eigen::AngleAxisf(euler_out(1), Eigen::Vector3f::UnitY())
                          * Eigen::AngleAxisf(euler_out(2), Eigen::Vector3f::UnitX()))
                             .toRotationMatrix();
    VERIFY(R_out.isApprox(R, 1e-4f));
}

static void test_geometry_umeyama() {
    SECTION("Geometry: Umeyama");

    // Create source points
    Eigen::Matrix3Xf src(3, 5);
    src << 0, 1, 2, 3, 4,
           0, 0, 0, 0, 0,
           0, 0, 0, 0, 0;

    // Rotate + translate to get dst
    Eigen::Matrix3f Rtrue = Eigen::AngleAxisf(M_PI / 4, Eigen::Vector3f::UnitZ())
                                .toRotationMatrix();
    Eigen::Vector3f ttrue(10, 20, 30);
    Eigen::Matrix3Xf dst = (Rtrue * src).colwise() + ttrue;

    Eigen::Matrix4f T = Eigen::umeyama(src, dst);
    // T maps src→dst in homogeneous coordinates
    // Verify: for each pair, T * src_h ≈ dst_h
    for (int i = 0; i < src.cols(); ++i) {
        Eigen::Vector4f src_h;
        src_h << src.col(i), 1.0f;
        Eigen::Vector4f dst_h = T * src_h;
        Eigen::Vector3f dst_pred = dst_h.head<3>() / dst_h(3);
        VERIFY(dst_pred.isApprox(dst.col(i), 5e-3f));
    }
}

// ===========================================================================
// 3. LU — decomposition, inverse, determinant
// ===========================================================================
static void test_lu() {
    SECTION("LU: PartialPivLU, FullPivLU, inverse, determinant");

    Eigen::Matrix3f A;
    A << 1, 2, 3,
         0, 1, 4,
         5, 6, 0;

    // PartialPivLU
    Eigen::PartialPivLU<Eigen::Matrix3f> lu(A);
    VERIFY(lu.matrixLU().rows() == 3);
    Eigen::Matrix3f A_inv = lu.inverse();
    VERIFY((A * A_inv).isApprox(Eigen::Matrix3f::Identity(), 1e-4f));

    // Solve
    Eigen::Vector3f b(1, 2, 3);
    Eigen::Vector3f x = lu.solve(b);
    VERIFY((A * x).isApprox(b, 1e-4f));

    // Determinant
    float det = A.determinant();
    // det([1 2 3; 0 1 4; 5 6 0]) = 1*(1*0-4*6) - 2*(0*0-4*5) + 3*(0*6-1*5)
    //  = -24 + 40 - 15 = 1
    VERIFY(std::abs(det - 1.0f) < 1e-4f);

    // .inverse() convenience
    Eigen::Matrix3f Ai = A.inverse();
    VERIFY((Ai * A).isApprox(Eigen::Matrix3f::Identity(), 1e-4f));

    // FullPivLU
    Eigen::FullPivLU<Eigen::Matrix3f> flu(A);
    VERIFY(flu.rank() == 3);
    VERIFY(flu.isInvertible());
    VERIFY(flu.dimensionOfKernel() == 0);

    // Rank-deficient matrix
    Eigen::Matrix3f singular;
    singular << 1, 2, 3,
                2, 4, 6,
                0, 0, 0;
    Eigen::FullPivLU<Eigen::Matrix3f> flu_s(singular);
    VERIFY(flu_s.rank() == 1);
    VERIFY(!flu_s.isInvertible());
    VERIFY(flu_s.dimensionOfKernel() == 2);
    VERIFY(std::abs(flu_s.determinant()) < 1e-10f);

    // image / kernel
    Eigen::MatrixXf ker = flu_s.kernel();
    VERIFY(ker.rows() == 3 && ker.cols() == 2);
}

// ===========================================================================
// 4. Cholesky
// ===========================================================================
static void test_cholesky() {
    SECTION("Cholesky: LLT, LDLT");

    // SPD matrix
    Eigen::Matrix3f A;
    A << 4, 12, -16,
         12, 37, -43,
         -16, -43, 98;
    // A is SPD (diagonally dominant variant)

    // LLT
    Eigen::LLT<Eigen::Matrix3f> llt(A);
    VERIFY(llt.info() == Eigen::Success);
    Eigen::Matrix3f L = llt.matrixL();
    VERIFY((L * L.transpose()).isApprox(A, 1e-4f));

    // Solve via LLT
    Eigen::Vector3f b(1, 2, 3);
    Eigen::Vector3f x = llt.solve(b);
    VERIFY((A * x).isApprox(b, 1e-4f));

    // LDLT
    Eigen::LDLT<Eigen::Matrix3f> ldlt(A);
    VERIFY(ldlt.info() == Eigen::Success);
    VERIFY(ldlt.isPositive());
    Eigen::Vector3f x2 = ldlt.solve(b);
    VERIFY((A * x2).isApprox(b, 1e-4f));

    // Non-positive-definite
    Eigen::Matrix3f B;
    B << 1, 2, 3,
         0, 1, 4,
         5, 6, 0;
    Eigen::LLT<Eigen::Matrix3f> llt_bad(B);
    // May or may not succeed depending on numerics, but shouldn't crash
    (void)llt_bad;
}

// ===========================================================================
// 5. QR
// ===========================================================================
static void test_qr() {
    SECTION("QR: HouseholderQR, ColPivHouseholderQR, FullPivHouseholderQR");

    Eigen::MatrixXf A(5, 3);
    A.setRandom();

    // HouseholderQR
    Eigen::HouseholderQR<Eigen::MatrixXf> hqr(A);
    // Extract thin Q explicitly
    Eigen::MatrixXf thinQ = hqr.householderQ() * Eigen::MatrixXf::Identity(5, 3);
    Eigen::MatrixXf R = hqr.matrixQR().topRows(3).triangularView<Eigen::Upper>();
    // thinQ should have orthogonal columns: Q^T Q = I_3
    VERIFY((thinQ.transpose() * thinQ).isApprox(Eigen::MatrixXf::Identity(3, 3), 1e-4f));
    VERIFY((thinQ * R).isApprox(A, 1e-4f));

    // Solve (least squares)
    Eigen::VectorXf b(5);
    b.setRandom();
    Eigen::VectorXf x = hqr.solve(b);
    // Check residual
    VERIFY((A.transpose() * (A * x - b)).norm() < 1e-3f);

    // ColPivHouseholderQR
    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> cpqr(A);
    VERIFY(cpqr.rank() >= 1);
    Eigen::VectorXf x2 = cpqr.solve(b);
    VERIFY(x2.size() == 3);

    // FullPivHouseholderQR
    Eigen::FullPivHouseholderQR<Eigen::MatrixXf> fpqr(A);
    VERIFY(fpqr.rank() >= 1);
    VERIFY(fpqr.isInjective());  // 5x3 should be full column rank
    Eigen::VectorXf x3 = fpqr.solve(b);
    VERIFY(x3.size() == 3);

    // CompleteOrthogonalDecomposition
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXf> cod(A);
    Eigen::VectorXf x4 = cod.solve(b);
    VERIFY(x4.size() == 3);

    // Under-determined system (more cols than rows)
    Eigen::MatrixXf wide(3, 6);
    wide.setRandom();
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXf> cod_w(wide);
    VERIFY(cod_w.rank() <= 3);
}

// ===========================================================================
// 6. SVD
// ===========================================================================
static void test_svd() {
    SECTION("SVD: JacobiSVD, BDCSVD");

    Eigen::MatrixXf A(4, 3);
    A.setRandom();

    // JacobiSVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXf s = svd.singularValues();
    VERIFY(s.size() == 3);
    VERIFY(s(0) >= s(1) && s(1) >= s(2));
    // All singular values non-negative
    for (int i = 0; i < s.size(); ++i) VERIFY(s(i) >= 0);

    // Reconstruction
    Eigen::MatrixXf Arec = svd.matrixU() * s.asDiagonal() * svd.matrixV().transpose();
    VERIFY(Arec.isApprox(A, 1e-4f));

    // Solve via SVD
    Eigen::VectorXf b(4);
    b.setRandom();
    Eigen::VectorXf x = svd.solve(b);
    VERIFY(x.size() == 3);

    // BDCSVD (bidiagonal divide-and-conquer, often faster for large matrices)
    Eigen::BDCSVD<Eigen::MatrixXf> bdc(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf Arec2 = bdc.matrixU() * bdc.singularValues().asDiagonal()
                            * bdc.matrixV().transpose();
    VERIFY(Arec2.isApprox(A, 1e-4f));

    // Rank (set small singular values to zero)
    svd.setThreshold(1e-6f);
    int rank = svd.rank();
    VERIFY(rank >= 1);

    // Square matrix SVD (request U and V explicitly)
    Eigen::Matrix3f Asq = Eigen::Matrix3f::Random();
    Eigen::JacobiSVD<Eigen::Matrix3f> svd_sq(Asq,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd_sq.matrixU();
    VERIFY(U.isUnitary(1e-4f));
}

// ===========================================================================
// 7. Eigenvalues
// ===========================================================================
static void test_eigenvalues() {
    SECTION("Eigenvalues: SelfAdjointEigenSolver, EigenSolver, ComplexEigenSolver");

    // --- SelfAdjointEigenSolver ---
    Eigen::Matrix3f Asym = Eigen::Matrix3f::Random();
    Asym = Asym * Asym.transpose();  // make SPD

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(Asym);
    VERIFY(es.info() == Eigen::Success);
    // Eigenvalues sorted ascending
    Eigen::Vector3f ev = es.eigenvalues();
    VERIFY(ev(0) <= ev(1) && ev(1) <= ev(2));
    VERIFY(ev(0) > 0);  // SPD => all positive

    // Diagonalization: A = V * D * V^T
    Eigen::Matrix3f V = es.eigenvectors();
    Eigen::Matrix3f D = ev.asDiagonal();
    VERIFY((V * D * V.transpose()).isApprox(Asym, 1e-4f));
    VERIFY(V.isUnitary(1e-4f));

    // Compute sqrt(A) via eigenvalue decomposition: A_sqrt = V * sqrt(D) * V^T
    Eigen::Matrix3f Asqrt = es.operatorSqrt();
    Eigen::Matrix3f A_recon = Asqrt * Asqrt;
    VERIFY(A_recon.isApprox(Asym, 1e-4f));

    // --- EigenSolver (general, possibly complex) ---
    Eigen::Matrix3f Agen = Eigen::Matrix3f::Random();
    Eigen::EigenSolver<Eigen::Matrix3f> eig(Agen);
    VERIFY(eig.info() == Eigen::Success);
    auto eigenvalues = eig.eigenvalues();
    VERIFY(eigenvalues.size() == 3);

    // Reconstruct: check that |A v - λ v| ~ 0 for each eigenpair
    auto eVecs = eig.eigenvectors();
    for (int i = 0; i < 3; ++i) {
        if (std::abs(eigenvalues(i).imag()) < 1e-10f) {
            float lambda = eigenvalues(i).real();
            Eigen::Vector3cf v = eVecs.col(i);
            Eigen::Vector3cf Av = Agen.cast<std::complex<float>>() * v;
            VERIFY((Av - lambda * v).norm() < 1e-3f);
        }
    }

    // --- ComplexEigenSolver ---
    Eigen::ComplexEigenSolver<Eigen::Matrix3f> ces(Agen);
    VERIFY(ces.info() == Eigen::Success);
    VERIFY(ces.eigenvalues().size() == 3);

    // --- GeneralizedSelfAdjointEigenSolver ---
    Eigen::Matrix3f B = Eigen::Matrix3f::Identity();
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix3f> ges(Asym, B);
    VERIFY(ges.info() == Eigen::Success);
    VERIFY(ges.eigenvalues().size() == 3);
}

static void test_eigenvalues_decompositions() {
    SECTION("Eigenvalues: Tridiagonalization, Hessenberg, RealSchur");

    Eigen::Matrix3f A = Eigen::Matrix3f::Random();

    // Tridiagonalization (A should be symmetric for best results)
    Eigen::Matrix3f Asym = Eigen::Matrix3f::Random();
    Asym = Asym * Asym.transpose();  // make symmetric
    Eigen::Tridiagonalization<Eigen::Matrix3f> tri(Asym);
    Eigen::Matrix3f T = tri.matrixT();
    Eigen::Matrix3f Q = tri.matrixQ();
    VERIFY(Q.isUnitary(1e-4f));
    VERIFY((Q * T * Q.transpose()).isApprox(Asym, 1e-4f));

    // HessenbergDecomposition
    Eigen::HessenbergDecomposition<Eigen::Matrix3f> hess(A);
    Eigen::Matrix3f H = hess.matrixH();
    // H should be upper Hessenberg: H(i,j)==0 for i>j+1
    VERIFY(std::abs(H(2,0)) < 1e-10f);
    Eigen::Matrix3f Qh = hess.matrixQ();
    VERIFY((Qh * H * Qh.transpose()).isApprox(A, 1e-4f));

    // RealSchur
    Eigen::RealSchur<Eigen::Matrix3f> schur(A);
    Eigen::Matrix3f U = schur.matrixU();
    // schur.matrixT() gives quasi-triangular Schur form
    VERIFY(U.isUnitary(1e-4f));

    // RealQZ
    Eigen::Matrix3f B = Eigen::Matrix3f::Identity();
    B(0,0) = 2;
    Eigen::RealQZ<Eigen::Matrix3f> qz(A, B);
    VERIFY(qz.info() == Eigen::Success);
}

// ===========================================================================
// 8. Householder
// ===========================================================================
static void test_householder() {
    SECTION("Householder");

    Eigen::Vector4f v = Eigen::Vector4f::Random();
    Eigen::HouseholderQR<Eigen::Matrix4f> hqr(Eigen::Matrix4f::Identity());
    // HouseholderSequence from QR
    auto hseq = hqr.householderQ();
    VERIFY(hseq.rows() == 4);
    // Apply to a vector
    Eigen::Vector4f w = hseq * v;
    VERIFY(w.size() == 4);
}

// ===========================================================================
// 9. Eigen 3.4+ IndexedView / Reshaped
// ===========================================================================
static void test_indexed_reshaped() {
    SECTION("Core: IndexedView and Reshaped");

    #if EIGEN_HAS_CXX11
    Eigen::Matrix4f M = Eigen::Matrix4f::Random();

    // IndexedView with seq
    Eigen::Matrix2f corner = M(Eigen::seq(0, 1), Eigen::seq(0, 1));
    VERIFY(corner(0,0) == M(0,0) && corner(1,1) == M(1,1));

    // Reshaped with runtime dimensions
    Eigen::MatrixXf reshaped_2x8 = M.reshaped(2, 8);
    VERIFY(reshaped_2x8.rows() == 2 && reshaped_2x8.cols() == 8);
    VERIFY(reshaped_2x8(0,0) == M(0,0));

    // Reshaped to vector (fixed)
    auto flat_fixed = M.reshaped();
    VERIFY(flat_fixed.size() == 16);
    VERIFY(flat_fixed(0) == M(0,0));
    #endif
}

// ===========================================================================
// 10. Misc — StlIterators, Fuzzy, IO, StlSupport
// ===========================================================================
static void test_stl_support() {
    SECTION("STL Support: StdVector, StdDeque, StdList");

    // std::vector with aligned allocator
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
    vec.push_back(Eigen::Vector4f(1, 2, 3, 4));
    vec.push_back(Eigen::Vector4f(5, 6, 7, 8));
    VERIFY(vec.size() == 2);
    VERIFY(vec[0].isApprox(Eigen::Vector4f(1, 2, 3, 4)));

    // std::deque
    std::deque<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f>> dq;
    dq.push_back(Eigen::Matrix2f::Identity());
    dq.push_front(Eigen::Matrix2f::Zero());
    VERIFY(dq.size() == 2);

    // std::list
    std::list<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> lst;
    lst.push_back(Eigen::Vector3f::Zero());
    lst.push_back(Eigen::Vector3f::Ones());
    VERIFY(lst.size() == 2);
}

static void test_fuzzy() {
    SECTION("Core: Fuzzy comparisons");

    Eigen::Vector3f a(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f b(1.0f + 1e-7f, 2.0f, 3.0f);

    VERIFY(a.isApprox(b));
    VERIFY(a.isApprox(b, 1e-6f));
    VERIFY(!a.isApprox(b, 1e-9f));  // tolerance too tight

    Eigen::Vector3f c(0.0f, 0.0f, 0.0f);
    VERIFY(c.isZero());
    VERIFY(a.isMuchSmallerThan(Eigen::Vector3f(1000, 2000, 3000), 1e-2f));
}

static void test_lvalue() {
    SECTION("Core: expression template aliasing safety");

    Eigen::Matrix3f A = Eigen::Matrix3f::Random();
    Eigen::Matrix3f B = Eigen::Matrix3f::Random();

    // transposeInPlace should not alias-wreck
    A = Eigen::Matrix3f::Random();
    Eigen::Matrix3f Acopy = A;
    A.transposeInPlace();
    VERIFY(A.isApprox(Acopy.transpose()));

    // noalias
    Eigen::Matrix3f C = Eigen::Matrix3f::Zero();
    C.noalias() = A * B;
    Eigen::Matrix3f Cref = A * B;
    VERIFY(C.isApprox(Cref));
}

static void test_mixed_precision() {
    SECTION("Core: mixed float/double/integer");

    Eigen::Matrix3f mf = Eigen::Matrix3f::Random();
    Eigen::Matrix3d md = mf.cast<double>();
    Eigen::Matrix3f mf_back = md.cast<float>();
    VERIFY(mf.isApprox(mf_back, 1e-5f));

    // Integer matrix
    Eigen::Matrix3i mi = Eigen::Matrix3i::Identity();
    VERIFY(mi(0,0) == 1 && mi(1,1) == 1);
    Eigen::Matrix3i mi2 = mi + mi;
    VERIFY(mi2(0,0) == 2);
}

static void test_tridiagonal_solve() {
    SECTION("Core: Tridiagonal matrix solve via Cholesky");

    // Build a tridiagonal SPD matrix and solve
    Eigen::Matrix4f M = Eigen::Matrix4f::Zero();
    M.diagonal().setConstant(4.0f);
    M.diagonal(1).setConstant(1.0f);
    M.diagonal(-1).setConstant(1.0f);
    // This is diagonally dominant so SPD
    Eigen::LLT<Eigen::Matrix4f> llt(M);
    VERIFY(llt.info() == Eigen::Success);

    Eigen::Vector4f b(1, 2, 3, 4);
    Eigen::Vector4f x = llt.solve(b);
    VERIFY((M * x).isApprox(b, 1e-4f));
}

// ===========================================================================
// 11. Edge cases and robustness
// ===========================================================================
static void test_edge_cases() {
    SECTION("Edge cases");

    // Zero-size matrix
    Eigen::MatrixXf empty(0, 0);
    VERIFY(empty.size() == 0);

    // 1x1 matrix
    Eigen::MatrixXf single(1, 1);
    single(0, 0) = 42.0f;
    VERIFY(single.determinant() == 42.0f);
    VERIFY(single.inverse()(0, 0) == 1.0f / 42.0f);

    // Large dynamic matrix
    Eigen::MatrixXf big(100, 100);
    big.setIdentity();
    VERIFY(big.isApprox(Eigen::MatrixXf::Identity(100, 100)));
    VERIFY(big.trace() == 100.0f);
    VERIFY(big.squaredNorm() == 100.0f);

    // Solve with singular matrix
    Eigen::Matrix3f singular;
    singular << 1, 2, 3,
                2, 4, 6,
                3, 6, 9;
    Eigen::Vector3f b(1, 2, 3);
    Eigen::JacobiSVD<Eigen::Matrix3f> svd_s(singular,
        Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f x = svd_s.solve(b);
    // Should produce a solution (minimum-norm), not crash
    VERIFY(x.size() == 3);
}

// ===========================================================================
// 12. Matrix free functions
// ===========================================================================
static void test_global_functions() {
    SECTION("Global functions: transpose, conjugate, adjoint");

    Eigen::Matrix3f A = Eigen::Matrix3f::Random();
    Eigen::Matrix3f At = A.transpose();
    VERIFY(At(0,1) == A(1,0));
    // conjugate of real matrix is itself
    Eigen::Matrix3f Aconj = A.conjugate();
    VERIFY(Aconj.isApprox(A));

    // Hermitian complex matrix via comma init
    Eigen::Matrix3cf Ac;
    Ac << std::complex<float>(1,0),  std::complex<float>(2,3),  std::complex<float>(4,5),
          std::complex<float>(2,-3), std::complex<float>(6,0),  std::complex<float>(7,8),
          std::complex<float>(4,-5), std::complex<float>(7,-8), std::complex<float>(9,0);
    // Hermitian matrix
    VERIFY(Ac.isApprox(Ac.adjoint(), 1e-4f));
}

// ===========================================================================
int main() {
    std::cout << "===========================================\n";
    std::cout << "  Eigen3 Dense-only Comprehensive Test\n";
    std::cout << "===========================================\n\n";

    // -- Core --
    test_core_basics();
    test_core_arithmetic();
    test_core_block_ops();
    test_core_advanced();
    test_core_comma_init();
    test_core_arithmetic_sequence();
    test_indexed_reshaped();
    test_fuzzy();
    test_lvalue();
    test_mixed_precision();
    test_tridiagonal_solve();

    // -- Geometry --
    test_geometry_quaternion();
    test_geometry_transform();
    test_geometry_other();
    test_geometry_aligned_box();
    test_geometry_euler();
    test_geometry_umeyama();

    // -- Decompositions --
    test_lu();
    test_cholesky();
    test_qr();
    test_svd();
    test_eigenvalues();
    test_eigenvalues_decompositions();
    test_householder();

    // -- Misc --
    test_stl_support();
    test_global_functions();
    test_edge_cases();

    // -- Report --
    std::cout << "\n===========================================\n";
    std::cout << "  Results: " << (g_total - g_failures) << " / " << g_total
              << " passed";
    if (g_failures > 0) {
        std::cout << "  (" << g_failures << " FAILURES)";
    }
    std::cout << "\n===========================================\n";

    return g_failures > 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
