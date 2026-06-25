# Eigen3 — Trimmed Dense-Only

基于 [Eigen](https://eigen.tuxfamily.org/) 裁剪的嵌入式版本，仅保留 **Dense 模块链**，去掉了 Sparse、外部求解器支持、unsupported 模块等嵌入式场景不需要的内容。

## 版本

从上游 Eigen 仓库裁剪，保留 MPL 2.0 许可证。

## 保留的模块

| 模块 | 说明 |
|---|---|
| **Core** | 矩阵/向量/数组基础运算，BLAS 级别操作，表达式模板 |
| **Jacobi** | Jacobi 旋转变换 |
| **Householder** | Householder 反射变换 |
| **LU** | LU 分解、逆矩阵、行列式、求解线性方程组 |
| **Cholesky** | LLT / LDLT 分解 |
| **QR** | HouseholderQR / ColPivHouseholderQR / FullPivHouseholderQR / CompleteOrthogonalDecomposition |
| **SVD** | JacobiSVD / BDCSVD 奇异值分解 |
| **Geometry** | 四元数、旋转矩阵、变换矩阵（Isometry/Affine）、欧拉角、Umeyama |
| **Eigenvalues** | SelfAdjointEigenSolver / EigenSolver / ComplexEigenSolver / GeneralizedSelfAdjointEigenSolver |
| **StdDeque / StdList / StdVector** | STL 容器对齐分配器支持 |
| **Dense** | 上述所有模块的聚合头文件 |
| **Eigen** | 顶层便捷头文件（仅 include Dense） |

`src/Core/arch/` 下保留了所有 CPU 架构目录（NEON / SSE / AVX / Default 等），保证多平台交叉编译兼容。

## 裁剪掉的内容

- **Sparse 模块**：SparseCore / SparseLU / SparseQR / SparseCholesky / OrderingMethods / IterativeLinearSolvers
- **外部求解器**：CholmodSupport / SuperLUSupport / UmfPackSupport / MetisSupport / PaStiXSupport / PardisoSupport / SPQRSupport / KLUSupport
- **unsupported/**：Tensor / AutoDiff / FFT / NonLinearOptimization 等
- **构建系统**：blas / lapack / CMake find-modules / 测试 / 文档 / bench / demos

## 集成方式

### CMake（推荐）

```cmake
add_subdirectory(thirdparty/eigen3)
target_link_libraries(your_target PRIVATE Eigen3::Eigen)
```

`Eigen3::Eigen` 是一个 INTERFACE 库（header-only），仅提供 include 路径，不产生编译产物。

### 手动 include

```cmake
target_include_directories(your_target PRIVATE thirdparty/eigen3)
```

```cpp
#include <Eigen/Eigen>   // 所有 Dense 模块
// 或按需引入：
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
```

## 嵌入式优化建议

### 编译选项

```cmake
# C++14 或更高（Eigen 3.4+ 推荐）
set(CMAKE_CXX_STANDARD 14)

# 针对目标架构优化（例如 Cortex-A53）
add_compile_options(-mcpu=cortex-a53 -mfpu=neon-fp-armv8)

# 如果不需要 IO 功能（<< 运算符），可定义此宏减小体积
target_compile_definitions(your_target PRIVATE EIGEN_NO_IO)
```

### 矩阵大小

- 优先使用固定大小矩阵（`Matrix3f`、`Vector4f`），编译器能更好地内联和向量化
- 动态大小矩阵（`MatrixXf`）会引入堆分配，嵌入式堆空间有限时需谨慎
- 大矩阵运算前检查 `EIGEN_STACK_ALLOCATION_LIMIT`，超过此阈值会改用堆分配

### NEON 向量化

Eigen 在 ARM Cortex-A 系列上会自动启用 NEON 指令集：
- 确保编译时带有 `-mfpu=neon` 或 `-march=armv8-a` 等标志
- NEON 实现位于 `Eigen/src/Core/arch/NEON/`，包含 PacketMath / MathFunctions / Complex / TypeCasting

对于 Cortex-M 系列（无 NEON），Eigen 会回退到 Default 标量路径。

## 注意事项

1. **Header-only**：Eigen 是完全模板化的，所有代码在头文件中。每个 `.cpp` 文件 include Eigen 都会触发大量模板实例化，首次编译较慢。可以考虑使用预编译头（PCH）。

2. **对齐**：Eigen 默认对固定大小对象使用 16/32 字节对齐。如果使用 `std::vector<Eigen::Matrix4f>` 等容器，请使用 `Eigen::aligned_allocator`：
   ```cpp
   #include <Eigen/StdVector>
   std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
   ```

3. **License**：Eigen 使用 MPL 2.0 许可证，允许商用和闭源项目使用。详见 `COPYING.MPL2`。

4. **API 兼容性**：此裁剪版仅移除了文件和模块，保留的文件内容未被修改（仅 `Eigen/Eigen` 顶层头文件改为 Dense-only）。与上游 Eigen 的 API 完全兼容。
