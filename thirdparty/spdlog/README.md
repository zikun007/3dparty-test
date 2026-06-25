# spdlog — Trimmed Embedded Version

基于 [spdlog](https://github.com/gabime/spdlog) v1.17.0 裁剪的嵌入式版本。去掉了异步日志、颜色控制台、平台特定 sink、网络 sink 等嵌入式场景不需要的模块，保留核心同步日志功能。

## 版本

spdlog v1.17.0，MIT 许可证。

## 保留的模块

### 核心

| 文件 | 说明 |
|---|---|
| `spdlog.h` | 全局便捷 API（`spdlog::info()` / `spdlog::error()` 等） |
| `logger.h` | `logger` 类，支持多 sink、级别过滤、自定义格式化 |
| `common.h` | 日志级别枚举、`SPDLOG_ACTIVE_LEVEL` 编译期裁剪宏 |
| `tweakme.h` | 用户可编辑的编译期配置（时钟源、线程 ID 等） |
| `pattern_formatter.h` | `"%Y-%m-%d %H:%M:%S [%n] [%l] %v"` 模式格式化 |
| `cfg/` | `SPDLOG_LEVEL` 环境变量 / argv 配置 |

### Sinks（5 种）

| Sink | 头文件 | 使用场景 |
|---|---|---|
| `stdout_sink` / `stderr_sink` | `stdout_sinks.h` | 串口/控制台调试输出 |
| `basic_file_sink` | `basic_file_sink.h` | 写入日志文件（SD 卡 / Flash） |
| `rotating_file_sink` | `rotating_file_sink.h` | 按文件大小轮转，防止写满存储 |
| `null_sink` | `null_sink.h` | 禁用日志输出（release 构建） |
| `base_sink` | `base_sink.h` | CRTP 模板基类，用于编写自定义 sink（如 UART/I²C 输出） |

### 格式化

内嵌 **bundled fmt** 库，不依赖外部 fmt 或 C++20 `std::format`。编译时完全自包含。

## 裁剪掉的内容

| 类别 | 移除项 |
|---|---|
| **异步日志** | `async.h` / `async_logger.h` / `thread_pool` / `mpmc_blocking_q` / `periodic_worker`（`flush_every()` 除外） |
| **颜色控制台** | `ansicolor_sink` / `stdout_color_sinks` / `wincolor_sink` — 默认 logger 改用普通 `stdout_sink` |
| **平台特定** | `syslog_sink` / `systemd_sink` / `android_sink` / `msvc_sink` / `win_eventlog_sink` |
| **网络** | `tcp_sink` / `udp_sink` / `kafka_sink` / `loki_sink` / `mongo_sink` |
| **多余文件 sink** | `daily_file_sink` / `hourly_file_sink`（用 `rotating_file_sink` 替代） |
| **其他 sink** | `ostream_sink` / `dist_sink` / `dup_filter_sink` / `ringbuffer_sink` / `callback_sink` / `qt_sinks` |
| **Stopwatch / MDC** | `stopwatch.h` 已移除；`mdc.h` 保留（`pattern_formatter` 依赖） |

## 集成方式

### CMake（推荐）

```cmake
add_subdirectory(thirdparty/spdlog)
target_link_libraries(your_target PRIVATE spdlog::spdlog)
```

默认 **header-only 模式**。如需编译为静态库（加速重复编译）：

```cmake
set(SPDLOG_COMPILED_LIB ON CACHE BOOL "")
add_subdirectory(thirdparty/spdlog)
target_link_libraries(your_target PRIVATE spdlog::spdlog)
```

### 手动 include

```cmake
target_include_directories(your_target PRIVATE thirdparty/spdlog/include)
target_link_libraries(your_target PRIVATE Threads::Threads)
```

## 嵌入式优化建议

### 编译期日志裁剪（最重要）

通过 `SPDLOG_ACTIVE_LEVEL` 在编译期剔除低级别日志，零运行时开销：

```cmake
# Debug 构建：保留所有级别
target_compile_definitions(your_target PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

# Release 构建：仅保留 warn 及以上
target_compile_definitions(your_target PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN)
```

级别宏：`SPDLOG_LEVEL_TRACE(0)` → `_DEBUG(1)` → `_INFO(2)` → `_WARN(3)` → `_ERROR(4)` → `_CRITICAL(5)` → `_OFF(6)`

### CMake 构建选项

```cmake
set(SPDLOG_NO_EXCEPTIONS    ON CACHE BOOL "")  # 去掉异常处理，减小代码体积
set(SPDLOG_NO_THREAD_ID     ON CACHE BOOL "")  # 跳过 gettid() 系统调用
set(SPDLOG_NO_ATOMIC_LEVELS ON CACHE BOOL "")  # 非原子级别（单线程安全）
set(SPDLOG_NO_TLS           ON CACHE BOOL "")  # 去掉 thread_local 存储
set(SPDLOG_CLOCK_COARSE     ON CACHE BOOL "")  # 粗粒度时钟（Linux）
set(SPDLOG_NO_SOURCE_LOC    ON CACHE BOOL "")  # 不记录 __FILE__ / __LINE__
set(SPDLOG_DISABLE_DEFAULT_LOGGER ON CACHE BOOL "") # 不自动创建默认 logger
```

或在 `tweakme.h` 中直接取消注释对应宏定义。

### 减少代码体积的其他技巧

1. **使用 `spdlog::logger` 对象而非全局函数**：全局函数 `spdlog::info()` 每次调用都要查找默认 logger，直接持有 `shared_ptr<logger>` 避免查找开销。

2. **单线程场景使用 `_st` sink**：
   ```cpp
   auto logger = spdlog::stdout_logger_st("app");  // 无锁版本
   ```

3. **自定义轻量 sink**：继承 `base_sink` 直接输出到 UART 等外设，跳过 printf / FILE* 开销：
   ```cpp
   template<typename Mutex>
   class uart_sink : public spdlog::sinks::base_sink<Mutex> {
   protected:
       void sink_it_(const spdlog::details::log_msg& msg) override {
           uart_write(msg.payload.data(), msg.payload.size());
       }
       void flush_() override {}
   };
   ```

4. **格式化字符串**：spdlog 使用编译期格式检查，格式字符串错误会有编译警告。

## 注意事项

1. **fmt 版本**：保留的是 bundled fmt，与外部 fmt 库不冲突。如需使用外部 fmt（如项目中已有），设置 `SPDLOG_FMT_EXTERNAL=ON`。

2. **Threads 依赖**：spdlog 的 `stdout_sink_mt` / `basic_file_sink_mt` 等 `_mt` sink 使用 `std::mutex`，需要链接 `pthread`（CMake 自动处理）。如果使用 `_st`（single-threaded）sink 且不开启 `flush_every()`，可去掉此依赖。

3. **默认 logger**：裁剪版默认 logger 使用普通 `stdout_sink_mt`（非颜色），输出格式与上游相同。如需禁止自动创建，设置 `SPDLOG_DISABLE_DEFAULT_LOGGER`。

4. **`spdlog::shutdown()`**：程序退出前应调用此函数清理线程资源（`flush_every()` 的后台线程等）。

5. **License**：MIT 许可证，允许商用和闭源。详见 `LICENSE` 文件。
