# 3dparty-test

嵌入式项目常用 C++ 第三方库的**裁剪版**集合，附带冒烟测试。专为资源受限的嵌入式平台（ARM Cortex-A / Cortex-M 等）设计，从上游官方库中精确裁剪出核心功能，去掉不必要的模块以减小体积和编译时间。

## 项目结构

```
3dparty-test/
├── CMakeLists.txt              # 顶层 CMake 项目
├── README.md                   # 本文件
├── .gitignore
├── nav_core/
│   ├── systime.hpp             # GNSS/UTC/UNIX 时间系统转换接口
│   ├── systime.cpp             # 时间系统转换实现
│   └── syscoord.hpp            # 坐标、姿态、地球模型转换接口
├── test/
│   ├── CMakeLists.txt          # 测试目标与 CTest 注册
│   ├── test_eigen3.cpp         # Eigen3 板端基础冒烟测试（36 个断言）
│   ├── test_eigen3_simple.cpp  # Eigen3 最小板端测试（11 个断言）
│   ├── test_systime.cpp        # nav_core 时间模块测试（10 个断言）
│   └── test_syscoord.cpp       # nav_core 坐标模块测试（12 个断言）
└── thirdparty/
    ├── eigen3/                 # → Eigen3 SRKF 嵌入式裁剪版（约 3.0 MB）
    │   ├── README.md           #    详细使用说明
    │   ├── CMakeLists.txt
    │   ├── COPYING.MPL2
    │   └── Eigen/
```

## 包含的库

### Eigen3 — 线性代数（SRKF 嵌入式裁剪版）

| 项目 | 详情 |
|---|---|
| **上游** | [eigen.tuxfamily.org](https://eigen.tuxfamily.org/) |
| **许可证** | MPL 2.0 |
| **保留模块** | Core, Geometry, LU, Cholesky, QR, Householder, Jacobi, STL 对齐容器支持 |
| **去除模块** | Sparse, SVD, Eigenvalues, BLAS/LAPACK/MKL, GPU/CUDA/HIP/SYCL, unsupported |
| **大小** | 约 3.0 MB |
| **CMake 目标** | `Eigen3::Eigen` |

```cpp
#include <Eigen/Eigen>
Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
```

详见 [thirdparty/eigen3/README.md](thirdparty/eigen3/README.md)

### nav_core — 组合导航基础工具

| 项目 | 详情 |
|---|---|
| **功能** | 时间系统转换、地球模型、坐标转换、姿态转换 |
| **依赖** | `Eigen3::Eigen` |
| **CMake 目标** | `nav_core::nav_core` |

```cpp
#include "nav_core/systime.hpp"
#include "nav_core/syscoord.hpp"

auto t = nav_core::ComTime::fromUtc(2026, 1, 1, 0, 0, 0.0);
Eigen::Vector3d xyz = nav_core::blh2xyz({30.0 * nav_core::kD2R, 114.0 * nav_core::kD2R, 42.0});
```

## 快速开始

### 构建和运行测试

```bash
git clone <this-repo>
cd 3dparty-test
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest
```

预期输出：

```
100% tests passed, 0 tests failed out of 4

Test #1: Eigen3Smoke ...... Passed   (36 / 36 assertions)
Test #2: Eigen3SimpleSmoke  Passed   (11 / 11 assertions)
Test #3: SysTimeSmoke ..... Passed   (10 / 10 assertions)
Test #4: SysCoordSmoke .... Passed   (12 / 12 assertions)
```

### 集成到你的嵌入式项目

**方式一：add_subdirectory（推荐）**

```cmake
# 在你的 CMakeLists.txt 中
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(3dparty-test)

target_link_libraries(your_app PRIVATE
    nav_core::nav_core
)
```

**方式二：git submodule**

```bash
git submodule add <this-repo> 3dparty-test
```

然后在 CMakeLists.txt 中按方式一引用。

## 嵌入式编译优化

### Eigen3

```cmake
# i.MX93 / Cortex-A55 板端工具链稳定性优先时建议关闭向量化。
set(USE_EIGEN_VECTORIZATION OFF CACHE BOOL "")
set(USE_EIGEN_PARALLEL OFF CACHE BOOL "")
```

更多细节见各库的 README。

## 详细说明

Eigen3 裁剪库目录下有独立的 `README.md`，包含：
- 保留/去除模块的完整清单
- CMake 集成示例
- 嵌入式优化建议
- 注意事项和常见陷阱

## 许可证

| 库 | 许可证 | 文件 |
|---|---|---|
| Eigen3 | MPL 2.0 | `thirdparty/eigen3/COPYING.MPL2` |

Eigen MPL 2.0 许可证允许商用和闭源使用。
