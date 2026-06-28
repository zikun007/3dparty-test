# Eigen3 SRKF 嵌入式裁剪版

本目录是面向 GNSS/INS 组合导航和平方根卡尔曼滤波（SRKF）的 Eigen 头文件裁剪版。目标是保留小矩阵、姿态、LLT Cholesky 和 LU 等嵌入式导航常用能力，删除在 i.MX93 板端 GCC 13.3 上反复触发编译器 ICE 的 QR、LDLT、COD、矩阵/几何层 `isApprox`，同时删除 SVD、特征值求解、外部 BLAS/LAPACK/MKL 适配和不相关硬件后端，降低源码体积和嵌入式编译负担。

Eigen 仍然是 header-only 库。通过 CMake 引入时只提供 include 路径和编译宏，不生成可链接库。

## 保留模块

| 模块 | 状态 | 保留原因 |
|---|---|---|
| `Core` | 保留 | 矩阵、向量、块操作、Map、Ref、三角视图、表达式模板和矩阵乘法基础 |
| `LU` | 保留 | 小矩阵求逆、行列式和线性方程求解的工程兼容能力；SRKF 主流程仍建议避免显式求逆 |
| `Cholesky` | 仅保留 `LLT` | 用于 SPD 协方差平方根初始化、正定性检查和 SPD 系统求解 |
| `Householder` | 轻量保留 | 保留基础 Householder 反射/序列头，供后续必要场景复用；不再提供 QR 分解入口 |
| `Jacobi` | 保留 | 保留基础旋转组件 |
| `Geometry` | 轻量保留 | 四元数、AngleAxis、Rotation2D、Transform、Translation、Scaling、欧拉角和姿态坐标变换 |
| `StdVector` / `StdDeque` / `StdList` | 保留 | Eigen 对齐分配器和 STL 容器适配 |
| `Core/arch/Default` | 保留 | 通用标量实现，所有平台都需要 |
| `Core/arch/NEON` | 保留 | i.MX93 / Cortex-A55 支持 ARMv8-A ASIMD/NEON，适合实测开启 |
| `Core/arch/SSE` / `AVX` / `AVX512` | 保留 | 方便桌面端开发验证和非 ARM 目标复用 |
| `Core/products/Parallelizer.h` | 保留 | 保留 Eigen 内部并行入口，但默认关闭 |

## 删除模块

| 内容 | 删除原因 |
|---|---|
| `SVD` / `BDCSVD` / `JacobiSVD` 模块入口和实现 | SRKF 主流程不需要奇异值分解，删除可明显降低编译负担 |
| `Eigenvalues` 模块入口和实现 | 特征值求解偏离实时嵌入式导航主流程，适合离线调试而非本裁剪版运行时 |
| `QR` / `HouseholderQR` / `ColPivHouseholderQR` / `FullPivHouseholderQR` | i.MX93 板端 GCC 13.3 多次在 QR 模板路径触发 ICE，暂不纳入板端裁剪版 |
| `CompleteOrthogonalDecomposition` (COD) | 依赖复杂 QR 路径，板端编译失败风险高 |
| `LDLT` | i.MX93 板端 GCC 13.3 在 LDLT/maxCoeff/Visitor 相关模板路径触发 ICE；保留更稳定的 `LLT` |
| 矩阵/几何层 `isApprox()` / `isMuchSmallerThan()` | 矩阵级 fuzzy comparison 会实例化复杂表达式模板，板端曾触发 ICE；建议测试代码使用显式循环比较 |
| `Geometry/Umeyama.h` | 相似变换拟合依赖 SVD，不属于 GNSS/INS 核心路径 |
| `Geometry/Hyperplane.h` / `ParametrizedLine.h` / `AlignedBox.h` | 几何辅助类型，导航姿态/坐标变换主流程不需要 |
| BLAS/LAPACK/MKL 适配头 | 本库按 header-only 嵌入式使用，不引入外部线性代数库 |
| `GPU` / `CUDA` / `HIP` / `SYCL` 后端 | 目标板不使用 GPU 通用计算 |
| `AltiVec` / `ZVector` / `MSA` / `SVE` 后端 | 非当前目标架构，删除以减小体积 |
| Sparse、unsupported、测试、文档、bench、demo | 上一轮裁剪已移除，继续保持不包含 |

## 默认开关

`thirdparty/eigen3/CMakeLists.txt` 提供两个选项：

```cmake
option(EIGEN3_ENABLE_VECTORIZATION "Enable Eigen SIMD vectorization when supported by the target compiler" ON)
option(EIGEN3_ENABLE_PARALLEL "Enable Eigen internal parallelization/OpenMP" OFF)
```

从本仓库顶层 `CMakeLists.txt` 配置时，对应使用 `USE_EIGEN_VECTORIZATION` 和 `USE_EIGEN_PARALLEL`，顶层会把它们转发给 `EIGEN3_ENABLE_VECTORIZATION` / `EIGEN3_ENABLE_PARALLEL`。

默认策略：

| 功能 | 默认 | 原因 |
|---|---|---|
| SIMD/NEON 向量化 | 启用 | i.MX93 Cortex-A55 支持 ARMv8-A ASIMD/NEON，值得在目标板实测 |
| Eigen/OpenMP 内部并行 | 禁用 | SRKF 常见矩阵维度较小，线程调度开销通常大于收益 |
| BLAS/LAPACK/MKL | 不支持 | 已删除外部适配头，避免嵌入式部署引入额外依赖 |
| CUDA/HIP/SYCL | 不支持 | 已删除相关后端 |

如果需要完全标量路径：

```cmake
set(EIGEN3_ENABLE_VECTORIZATION OFF CACHE BOOL "" FORCE)
```

如果需要试验 Eigen 内部 OpenMP 并行：

```cmake
set(EIGEN3_ENABLE_PARALLEL ON CACHE BOOL "" FORCE)
find_package(OpenMP REQUIRED)
target_link_libraries(your_target PRIVATE OpenMP::OpenMP_CXX)
```

注意：对于 15 维、18 维、30 维以内的滤波矩阵，不建议默认开启 OpenMP。

## 推荐集成方式

```cmake
add_subdirectory(thirdparty/eigen3)
target_link_libraries(your_target PRIVATE Eigen3::Eigen)
```

或者手动 include：

```cmake
target_include_directories(your_target PRIVATE thirdparty/eigen3)
```

常用头文件：

```cpp
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
```

`#include <Eigen/Eigen>` 和 `#include <Eigen/Dense>` 当前等价于引入本裁剪版 Dense 聚合能力：`Core + LU + LLT Cholesky + Geometry`。

## i.MX93 / Cortex-A55 建议

板端 `/proc/cpuinfo` 中包含：

```text
Features: fp asimd ... fphp asimdhp ... asimddp
CPU part: 0xd05
```

这说明 CPU 支持 ARMv8-A ASIMD/NEON。建议交叉编译时针对 Cortex-A55 设置目标：

```cmake
target_compile_options(your_target PRIVATE -mcpu=cortex-a55)
```

如果工具链不接受 `-mcpu=cortex-a55`，可尝试：

```cmake
target_compile_options(your_target PRIVATE -march=armv8-a+simd)
```

实际收益需要在目标板测试。四元数、`3x3`、`4x4` 极小矩阵收益可能不明显；`15x15`、`18x18`、`30x30` 矩阵乘法更值得对比。

## SRKF 使用建议

- 优先使用固定大小矩阵，例如 `Eigen::Matrix<float, 15, 15>`。
- 尽量避免 `MatrixXf`，除非维度确实运行时才确定。
- SRKF 中仍可使用 `LLT` 做 SPD 协方差平方根初始化和正定性检查。
- 本裁剪版不再提供 QR，因此平方根滤波的正交化/三角化步骤建议先使用项目内显式 Givens/Householder 小矩阵实现，或在交叉编译工具链稳定后再单独恢复 QR。
- 不要用显式 `.inverse()` 作为滤波主路径；小维度测试可保留 `.inverse()` 验证 Eigen LU 能力。
- 乘法赋值可按需使用 `noalias()` 减少临时对象。
- 若使用 STL 容器保存固定大小 Eigen 对象，使用 `Eigen::aligned_allocator`。

显式比较示例：

```cpp
template <typename Lhs, typename Rhs>
bool coeffsNear(const Eigen::MatrixBase<Lhs>& lhs,
                const Eigen::MatrixBase<Rhs>& rhs,
                float eps) {
    if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) return false;
    for (Eigen::Index r = 0; r < lhs.rows(); ++r)
        for (Eigen::Index c = 0; c < lhs.cols(); ++c)
            if (std::abs(lhs(r, c) - rhs(r, c)) > eps) return false;
    return true;
}
```

## API 边界

本裁剪版不支持：

- `#include <Eigen/SVD>`
- `#include <Eigen/Eigenvalues>`
- `#include <Eigen/QR>`
- `Eigen::JacobiSVD`
- `Eigen::BDCSVD`
- `Eigen::SelfAdjointEigenSolver`
- `Eigen::EigenSolver`
- `Eigen::HouseholderQR`
- `Eigen::ColPivHouseholderQR`
- `Eigen::FullPivHouseholderQR`
- `Eigen::CompleteOrthogonalDecomposition`
- `Eigen::LDLT`
- `MatrixBase::ldlt()`
- `MatrixBase::householderQr()`
- `MatrixBase::colPivHouseholderQr()`
- `MatrixBase::fullPivHouseholderQr()`
- `MatrixBase::completeOrthogonalDecomposition()`
- `MatrixBase::isApprox()`
- `MatrixBase::isMuchSmallerThan()`
- 几何类型的 `isApprox()`，例如 `Quaternion::isApprox()` / `Transform::isApprox()`
- `Eigen::umeyama`
- `Eigen::Hyperplane`
- `Eigen::ParametrizedLine`
- `Eigen::AlignedBox`

`Quaternion::FromTwoVectors()` 已改为不依赖 SVD 的实现，保留用于姿态初始化等导航场景。

本仓库默认构建 `test_eigen3`、`test_systime` 和 `test_syscoord`。其中 `test_eigen3` 已合并为单文件板端测试，覆盖 GNSS/INS 紧组合常用的固定维矩阵、协方差传播/更新、LU、LLT、四元数、姿态旋转、反对称矩阵和 Eigen 对齐容器，同时避开 QR、LDLT、COD 和 `isApprox()`。

`Transform::computeRotationScaling()`、`Transform::computeScalingRotation()` 和非 `Isometry` 模式下的部分 `rotation()` 路径仍属于上游 SVD 相关接口。请避免在本裁剪版中调用这些接口。

## 许可证

Eigen 使用 MPL 2.0 许可证。保留 `COPYING.MPL2`，可用于商用和闭源项目。修改或再发布此裁剪版时，请继续保留许可证文件和版权说明。
