/**
 * @file test_spdlog.cpp
 * @brief Comprehensive test for trimmed spdlog (embedded Dense-sinks only).
 *
 * Tests: stdout sink, file sink, rotating file sink, null sink,
 *        custom sink via base_sink, log levels, pattern formatting.
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/cfg/env.h>

#include <iostream>
#include <cstdio>
#include <string>
#include <mutex>

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
// 1. Default logger (should be created automatically unless disabled)
// ---------------------------------------------------------------------------
static void test_default_logger() {
    SECTION("Default logger (stdout)");

    auto logger = spdlog::default_logger();
    VERIFY(logger != nullptr);

    // Basic logging at each level
    spdlog::trace("trace msg");
    spdlog::debug("debug msg");
    spdlog::info("info msg");
    spdlog::warn("warn msg");
    spdlog::error("error msg");
    spdlog::critical("critical msg");

    // Formatted logging
    spdlog::info("Hello {} from spdlog v{}.{}.{}", "world",
                 SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR, SPDLOG_VER_PATCH);
    VERIFY(true); // if we got here, no crash
}

// ---------------------------------------------------------------------------
// 2. stdout / stderr sinks
// ---------------------------------------------------------------------------
static void test_stdout_sinks() {
    SECTION("stdout / stderr sinks");

    auto console = spdlog::stdout_logger_mt("console");
    console->info("stdout logger works");
    console->flush();

    auto err_logger = spdlog::stderr_logger_mt("stderr");
    err_logger->error("stderr logger works");
    err_logger->flush();

    // Single-threaded variant
    auto console_st = spdlog::stdout_logger_st("console_st");
    console_st->info("stdout ST works");

    VERIFY(true);
}

// ---------------------------------------------------------------------------
// 3. Basic file sink
// ---------------------------------------------------------------------------
static void test_basic_file_sink() {
    SECTION("Basic file sink");

    const char *filename = "test_basic.log";

    // Remove old file
    std::remove(filename);

    auto file_logger = spdlog::basic_logger_mt("file", filename);
    file_logger->info("file line 1");
    file_logger->info("file line 2: number={}", 42);
    file_logger->flush();

    // Verify file exists and has content
    std::FILE *fp = std::fopen(filename, "r");
    VERIFY(fp != nullptr);
    if (fp) {
        char buf[512];
        size_t n = std::fread(buf, 1, sizeof(buf) - 1, fp);
        buf[n] = '\0';
        std::string content(buf, n);
        VERIFY(content.find("file line 1") != std::string::npos);
        VERIFY(content.find("file line 2: number=42") != std::string::npos);
        std::fclose(fp);
    }
    std::remove(filename);
}

// ---------------------------------------------------------------------------
// 4. Rotating file sink
// ---------------------------------------------------------------------------
static void test_rotating_file_sink() {
    SECTION("Rotating file sink");

    const char *base = "test_rot.log";
    std::remove(base);

    // 2KB max, 3 files
    auto rot = spdlog::rotating_logger_mt("rot", base, 2048, 3);
    VERIFY(rot != nullptr);

    // Write enough to trigger rotation
    for (int i = 0; i < 200; ++i) {
        rot->info("rotation test line {} with some padding data", i);
    }
    rot->flush();

    // At least the base file should exist
    std::FILE *fp = std::fopen(base, "r");
    VERIFY(fp != nullptr);
    if (fp) std::fclose(fp);

    std::remove(base);
    std::remove("test_rot.1.log");
    std::remove("test_rot.2.log");
}

// ---------------------------------------------------------------------------
// 5. Null sink
// ---------------------------------------------------------------------------
static void test_null_sink() {
    SECTION("Null sink");

    auto null_sink = std::make_shared<spdlog::sinks::null_sink_mt>();
    auto null_logger = std::make_shared<spdlog::logger>("null", null_sink);
    spdlog::register_logger(null_logger);

    null_logger->info("this goes nowhere");
    null_logger->error("this also goes nowhere");
    null_logger->flush();

    VERIFY(true); // no crash = pass
}

// ---------------------------------------------------------------------------
// 6. Custom sink via base_sink
// ---------------------------------------------------------------------------
template <typename Mutex>
class count_sink : public spdlog::sinks::base_sink<Mutex> {
public:
    int count = 0;

protected:
    void sink_it_(const spdlog::details::log_msg &) override { ++count; }
    void flush_() override {}
};

static void test_custom_sink() {
    SECTION("Custom sink via base_sink");

    auto cs = std::make_shared<count_sink<std::mutex>>();
    auto custom = std::make_shared<spdlog::logger>("custom", cs);

    custom->info("msg 1");
    custom->info("msg 2");
    custom->warn("msg 3");

    VERIFY(cs->count == 3);

    custom->flush();
    VERIFY(cs->count == 3); // flush shouldn't change count
}

// ---------------------------------------------------------------------------
// 7. Log levels
// ---------------------------------------------------------------------------
static void test_log_levels() {
    SECTION("Log levels");

    auto logger = spdlog::stdout_logger_st("level_test");

    logger->set_level(spdlog::level::warn);
    VERIFY(logger->level() == spdlog::level::warn);
    VERIFY(logger->should_log(spdlog::level::info) == false);
    VERIFY(logger->should_log(spdlog::level::warn) == true);
    VERIFY(logger->should_log(spdlog::level::critical) == true);

    // Flush level
    logger->flush_on(spdlog::level::err);
    // (no direct way to verify flush level, just ensure no crash)

    // Level names
    VERIFY(spdlog::level::to_string_view(spdlog::level::info) == "info");
    VERIFY(spdlog::level::to_string_view(spdlog::level::critical) == "critical");
}

// ---------------------------------------------------------------------------
// 8. Pattern formatting
// ---------------------------------------------------------------------------
static void test_pattern_formatting() {
    SECTION("Pattern formatting");

    const char *filename = "test_pattern.log";
    std::remove(filename);

    auto logger = spdlog::basic_logger_mt("pattern", filename);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%n] [%^%l%$] %v");
    logger->info("pattern test");
    logger->flush();

    std::FILE *fp = std::fopen(filename, "r");
    VERIFY(fp != nullptr);
    if (fp) {
        char buf[512];
        size_t n = std::fread(buf, 1, sizeof(buf) - 1, fp);
        buf[n] = '\0';
        std::string content(buf, n);
        VERIFY(content.find("pattern test") != std::string::npos);
        VERIFY(content.find("[info]") != std::string::npos);
        std::fclose(fp);
    }
    std::remove(filename);
}

// ---------------------------------------------------------------------------
// 9. Multiple sinks
// ---------------------------------------------------------------------------
static void test_multi_sink() {
    SECTION("Multiple sinks per logger");

    auto cs = std::make_shared<count_sink<std::mutex>>();
    auto ns = std::make_shared<spdlog::sinks::null_sink_mt>();

    spdlog::sinks_init_list sinks = {cs, ns};
    auto multi = std::make_shared<spdlog::logger>("multi", sinks);

    multi->info("broadcast");

    VERIFY(cs->count == 1);
}

// ---------------------------------------------------------------------------
// 10. Error handler
// ---------------------------------------------------------------------------
static void test_error_handler() {
    SECTION("Error handler");

    // spdlog supports both function pointers and std::function lambdas.
    // For captured lambdas, use the std::function overload:
    spdlog::default_logger()->set_error_handler(
        std::function<void(const std::string &)>([](const std::string &) {
            // error handled
        }));

    spdlog::info("error handler test");

    // Reset to default (no-op)
    spdlog::default_logger()->set_error_handler({});
    VERIFY(true);
}

// ---------------------------------------------------------------------------
// 11. Registry operations
// ---------------------------------------------------------------------------
static void test_registry() {
    SECTION("Registry: get, drop, apply_all");

    auto lg = spdlog::get("registry_test");
    VERIFY(lg == nullptr);

    auto created = spdlog::stdout_logger_mt("registry_test");
    auto found = spdlog::get("registry_test");
    VERIFY(found != nullptr);

    found->set_level(spdlog::level::off);

    spdlog::drop("registry_test");
    auto gone = spdlog::get("registry_test");
    VERIFY(gone == nullptr);

    spdlog::apply_all([](const std::shared_ptr<spdlog::logger> &l) {
        // Just verifying the callback is called
        (void)l;
    });
    VERIFY(true);
}

// ---------------------------------------------------------------------------
// 12. Drop all (cleanup at end)
// ---------------------------------------------------------------------------
static void test_drop_all() {
    SECTION("Drop all loggers");
    spdlog::drop_all();
    VERIFY(spdlog::default_logger_raw() == nullptr);
}

// ===========================================================================
int main() {
    std::cout << "===========================================\n";
    std::cout << "  spdlog Trimmed Embedded Test\n";
    std::cout << "  Version: " << SPDLOG_VER_MAJOR << "."
              << SPDLOG_VER_MINOR << "." << SPDLOG_VER_PATCH << "\n";
    std::cout << "===========================================\n\n";

    test_default_logger();
    test_stdout_sinks();
    test_basic_file_sink();
    test_rotating_file_sink();
    test_null_sink();
    test_custom_sink();
    test_log_levels();
    test_pattern_formatting();
    test_multi_sink();
    test_error_handler();
    test_registry();
    test_drop_all();

    std::cout << "\n===========================================\n";
    std::cout << "  Results: " << (g_total - g_failures) << " / " << g_total
              << " passed";
    if (g_failures > 0) {
        std::cout << "  (" << g_failures << " FAILURES)";
    }
    std::cout << "\n===========================================\n";

    return g_failures > 0 ? 1 : 0;
}
