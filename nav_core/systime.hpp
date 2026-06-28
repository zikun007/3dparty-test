/**
 * @file systime.hpp
 * @author kun (2025202140016@whu.edu.cn)
 * @brief 构建统一的时间系统基准，主要包括常用的（GPST、BDT、GST、UTC、UNIX），ComTime 内部默认时间系统为GPST
 *
 * GNSS time utility with a single internal timescale.
 * Internal representation:
 *   - gpst_ns_: int64 nanoseconds since GPS Epoch (1980-01-06 00:00:00 UTC)
 *   - Time scale is GPST (continuous, no leap seconds).
 *
 * External I/O:
 *   - UTC calendar: Y/M/D h:m:s (may include leap second 23:59:60)
 *   - UNIX/POSIX timestamp: seconds/nanoseconds since 1970-01-01 00:00:00 UTC, no leap seconds.
 *   - BDT: continuous time with fixed offset to GPST: BDT = GPST - 14s  => GPST = BDT + 14s
 *
 * For RINEX usage (GPS/BDS/GAL only):
 *   - GPS (G): GPST
 *   - Galileo (E): treated as GPST (integer-second alignment; corrections handled elsewhere if needed)
 *   - BDS (C): BDT (fixed -14s from GPST)
 *
 * @note UNIX     起始时刻 1970-01-01 00:00:00 无闰秒 UnixSec = 0
         UTC      起始时刻 1970-01-01 00:00:00 闰秒（其实并无起始时刻，但一般与UNIX时间同步）
         GPST     起始时刻 1980-01-06 00:00:00 无闰秒 UnixSec = 315964800
         GLONASST 起始时刻 1996-01-01 00:00:00 无闰秒 UnixSec = 820454400
         GST      起始时刻 1999-08-22 00:00:00 无闰秒 UnixSec = 935280000
         BDT      起始时刻 2006-01-01 00:00:00 无闰秒 UnixSec = 1136073600
         QZST     与GPST对齐，无闰秒
 * @version 0.1
 * @date 2026-01-09
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <cstdint>
#include <string>

namespace nav_core {

/// 日历时刻表示，用于 UTC / GPST / BDT / GST 的日历形式输入输出。
struct Calendar {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
};

/// GPS 时的周 + 周内秒表示，与 Calendar 对称，可作为工厂输入和 toGpst() 的输出。
struct Gpst {
    int week = 0;
    double sow = 0.0;
};

/**
 * @brief 通用时间类 (核心基准: GPST)
 *
 * 内部以 int64 纳秒存储，所有时间系统在边界处完成转换。
 * 算术运算的时间差参数统一使用 double 秒，避免引入额外包装类型。
 */
class ComTime {
  public:
    static constexpr int64_t kNsPerSec = 1'000'000'000LL;

    ComTime() = default;

	// -------------------------------------------------------------------------
	// 输入构造接口 (Factories)
	// -------------------------------------------------------------------------

	// 1. 纳秒 / 秒级直接构造 (ROS / Camera / IMU 常用)
    static ComTime fromGpstNs(int64_t ns);
    static ComTime fromUnixNs(int64_t ns);
    static ComTime fromUnixSec(double sec);

	// 2. 周 + 周内秒构造 (GNSS 观测值常用)
	// fromGpst(Gpst) 为规范实现；两参数重载和其他系统保持调用兼容性。
    static ComTime fromGpst(const Gpst& gpst);
    static ComTime fromGpst(int week, double sow) { return fromGpst(Gpst{week, sow}); }
    static ComTime fromBdt(int week, double sow);
    static ComTime fromGst(int week, double sow);

  	// 3. 日历构造 (支持各类系统时间的日历形式)
    static ComTime fromUtc(const Calendar& cal);
    static ComTime fromGpst(const Calendar& cal);
    static ComTime fromBdt(const Calendar& cal);
    static ComTime fromGst(const Calendar& cal);

	// 4. 日历快捷展开构造 (inline，消除 .cpp 样板)
	// clang-format off
	static ComTime fromUtc (int y, int m, int d, int h, int min, double s) { return fromUtc ({y,m,d,h,min,s}); }
	static ComTime fromGpst(int y, int m, int d, int h, int min, double s) { return fromGpst({y,m,d,h,min,s}); }
	static ComTime fromBdt (int y, int m, int d, int h, int min, double s) { return fromBdt ({y,m,d,h,min,s}); }
	static ComTime fromGst (int y, int m, int d, int h, int min, double s) { return fromGst ({y,m,d,h,min,s}); }
  	// clang-format on

	// -------------------------------------------------------------------------
	// 输出接口 (Outputs)
	// -------------------------------------------------------------------------

    double toUnixSec() const;
    Gpst toGpst() const;
    Calendar toUtc() const;

    std::string toUtcStr(int precision = 3) const;
    std::string toGpstStr(int precision = 3) const;

	// -------------------------------------------------------------------------
	// 运算符重载 (Operators)
	// 时间差统一使用 double 秒；精度转换由内部 SecToNs 保证。
	// -------------------------------------------------------------------------
    ComTime operator+(double sec) const;
    ComTime operator-(double sec) const;
    double operator-(const ComTime& rhs) const;
    ComTime& operator+=(double sec);
    ComTime& operator-=(double sec);

  	// clang-format off
	bool operator==(const ComTime& rhs) const { return gpst_ns_ == rhs.gpst_ns_; }
	bool operator!=(const ComTime& rhs) const { return gpst_ns_ != rhs.gpst_ns_; }
	bool operator< (const ComTime& rhs) const { return gpst_ns_ <  rhs.gpst_ns_; }
	bool operator> (const ComTime& rhs) const { return gpst_ns_ >  rhs.gpst_ns_; }
	bool operator<=(const ComTime& rhs) const { return gpst_ns_ <= rhs.gpst_ns_; }
	bool operator>=(const ComTime& rhs) const { return gpst_ns_ >= rhs.gpst_ns_; }
 	 // clang-format on

  private:
    explicit ComTime(int64_t gpst_ns) : gpst_ns_(gpst_ns) {}
    ComTime roundedForStr(int& precision) const;
    int64_t gpst_ns_ = 0;
};

} // namespace nav_core
