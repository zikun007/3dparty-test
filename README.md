# 3dparty-test

嵌入式项目常用 C++ 第三方库的**裁剪版**集合，附带冒烟测试。专为资源受限的嵌入式平台（ARM Cortex-A / Cortex-M 等）设计，从上游官方库中精确裁剪出核心功能，去掉不必要的模块以减小体积和编译时间。

## 项目结构

```
3dparty-test/
├── CMakeLists.txt              # 顶层 CMake 项目
├── README.md                   # 本文件
├── .gitignore
├── app/
│   ├── test_eigen3.cpp         # Eigen3 全面测试（169 个断言）
│   └── test_spdlog.cpp         # spdlog 全面测试（27 个断言）
└── thirdparty/
    ├── eigen3/                 # → Eigen3 裁剪版（Dense-only, 5.8 MB）
    │   ├── README.md           #    详细使用说明
    │   ├── CMakeLists.txt
    │   ├── COPYING.MPL2
    │   └── Eigen/
    └── spdlog/                 # → spdlog 裁剪版（嵌入式优化, 1.1 MB）
        ├── README.md           #    详细使用说明
        ├── CMakeLists.txt
        ├── LICENSE
        ├── include/
        └── src/
```

## 包含的库

### Eigen3 — 线性代数（Dense-only）

| 项目 | 详情 |
|---|---|
| **上游** | [eigen.tuxfamily.org](https://eigen.tuxfamily.org/) |
| **许可证** | MPL 2.0 |
| **保留模块** | Core, Geometry, LU, Cholesky, QR, SVD, Eigenvalues, Householder, Jacobi |
| **去除模块** | Sparse, 外部求解器 (SuiteSparse/Metis/PaStiX), unsupported (Tensor/AutoDiff/FFT 等) |
| **大小** | 5.8 MB（原始 ~6.8 MB） |
| **CMake 目标** | `Eigen3::Eigen` |

```cpp
#include <Eigen/Eigen>
Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
```

详见 [thirdparty/eigen3/README.md](thirdparty/eigen3/README.md)

### spdlog — 高性能日志（嵌入式优化）

| 项目 | 详情 |
|---|---|
| **上游** | [github.com/gabime/spdlog](https://github.com/gabime/spdlog) v1.17.0 |
| **许可证** | MIT |
| **保留模块** | 核心同步日志 + 5 种 sink（stdout / file / rotating / null / base_sink） |
| **去除模块** | 异步日志、颜色控制台、网络/平台特定 sink (syslog/systemd/kafka 等) |
| **格式化** | 内嵌 bundled fmt（无外部依赖） |
| **大小** | 1.1 MB |
| **CMake 目标** | `spdlog::spdlog` |

```cpp
#include <spdlog/spdlog.h>
spdlog::info("Hello {}!", "world");
auto logger = spdlog::basic_logger_mt("app", "app.log");
```

详见 [thirdparty/spdlog/README.md](thirdparty/spdlog/README.md)

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
100% tests passed, 0 tests failed out of 2

Test #1: Eigen3Smoke ... Passed   (169/169 assertions)
Test #2: SpdlogSmoke ... Passed   ( 27/ 27 assertions)
```

### 集成到你的嵌入式项目

**方式一：add_subdirectory（推荐）**

```cmake
# 在你的 CMakeLists.txt 中
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(3dparty-test/thirdparty/eigen3)
add_subdirectory(3dparty-test/thirdparty/spdlog)

target_link_libraries(your_app PRIVATE
    Eigen3::Eigen
    spdlog::spdlog
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
# Cortex-A (ARMv8, NEON)
add_compile_options(-mcpu=cortex-a53 -mfpu=neon-fp-armv8)
target_compile_definitions(your_app PRIVATE EIGEN_NO_IO)
```

### spdlog

```cmake
# 推荐在嵌入式 release 构建中开启
set(SPDLOG_NO_EXCEPTIONS    ON CACHE BOOL "")
set(SPDLOG_NO_THREAD_ID     ON CACHE BOOL "")
set(SPDLOG_NO_ATOMIC_LEVELS ON CACHE BOOL "")
set(SPDLOG_DISABLE_DEFAULT_LOGGER ON CACHE BOOL "")

# 编译期剔除低级别日志
target_compile_definitions(your_app PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN)
```

更多细节见各库的 README。

## 两个库的详细说明

每个裁剪库目录下均有独立的 `README.md`，包含：
- 保留/去除模块的完整清单
- CMake 集成示例
- 嵌入式优化建议
- 注意事项和常见陷阱

## 许可证

| 库 | 许可证 | 文件 |
|---|---|---|
| Eigen3 | MPL 2.0 | `thirdparty/eigen3/COPYING.MPL2` |
| spdlog | MIT | `thirdparty/spdlog/LICENSE` |

两个许可证均允许商用和闭源使用。
