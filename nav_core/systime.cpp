#include "nav_core/systime.hpp" // 请确保包含路径与你的工程一致

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace nav_core {

namespace {

constexpr int64_t kSecPerDay = 86400LL;
constexpr int64_t kSecPerWeek = 604800LL;

// GPS 历元 (1980-01-06 00:00:00 UTC) 对应的 UNIX 连续秒数
constexpr int64_t kUnixGpsEpoch = 315964800LL;

// BDT = GPST - 14s  =>  GPST = BDT + 14s
constexpr int64_t kGpstMinusBdt = 14LL;
constexpr int kBdtWeekOffset = 1356;
constexpr int kGstWeekOffset = 1024;

int64_t secToNs(double sec) { return static_cast<int64_t>(std::llround(static_cast<long double>(sec) * 1.0e9L)); }

inline int64_t pow10Int(int n) {
    int64_t v = 1;
    while (n-- > 0) v *= 10;
    return v;
}

// 快速向下取整除法（避免依赖 std::floor 的浮点运算）
inline void fastDivMod(int64_t total_ns, int64_t& sec, int64_t& rem_ns) {
    sec = total_ns / ComTime::kNsPerSec;
    rem_ns = total_ns % ComTime::kNsPerSec;
    if (rem_ns < 0) {
        --sec;
        rem_ns += ComTime::kNsPerSec;
    }
}

inline int64_t floorDiv(int64_t a, int64_t b) {
    int64_t q = a / b;
    int64_t r = a % b;
    if (r != 0 && ((r < 0) != (b < 0))) --q;
    return q;
}

inline int64_t roundToUnit(int64_t value, int64_t unit) {
    if (unit <= 1) return value;
    if (value >= 0) return ((value + unit / 2) / unit) * unit;
    return ((value - unit / 2) / unit) * unit;
}

// ----- 日历核心算法 (Howard Hinnant, O(1) 纯整数运算) -----
inline int64_t daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(doe) - 719468;
}

inline Calendar civilFromDays(int64_t z) {
    z += 719468;
    const int era = static_cast<int>((z >= 0 ? z : z - 146096) / 146097);
    const unsigned doe = static_cast<unsigned>(z - static_cast<int64_t>(era) * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int y = static_cast<int>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);

    Calendar ct;
    ct.year = y;
    ct.month = static_cast<int>(m);
    ct.day = static_cast<int>(d);
    return ct;
}

inline int64_t calendarToContinuousNs(const Calendar& cal) {
    const int64_t days = daysFromCivil(cal.year, cal.month, cal.day);
    const int64_t sec_whole = static_cast<int64_t>(std::floor(cal.second));
    const long double frac = static_cast<long double>(cal.second) - static_cast<long double>(sec_whole);
    int64_t frac_ns = static_cast<int64_t>(std::llround(frac * 1.0e9L));

    int64_t sec_in_day = cal.hour * 3600LL + cal.minute * 60LL + sec_whole;
    if (frac_ns >= ComTime::kNsPerSec) {
        ++sec_in_day;
        frac_ns -= ComTime::kNsPerSec;
    } else if (frac_ns < 0) {
        --sec_in_day;
        frac_ns += ComTime::kNsPerSec;
    }

    return (days * kSecPerDay + sec_in_day) * ComTime::kNsPerSec + frac_ns;
}

// ----- 预计算的双向闰秒表 -----
struct Leap {
    int64_t unix_sec;
    int64_t gpst_sec;
    int offset;
};
constexpr std::array<Leap, 19> kLeaps = {{
    {0LL, 0LL, 0},
    {362793600LL, 46828801LL, 1},
    {394329600LL, 78364802LL, 2},
    {425865600LL, 109900803LL, 3},
    {489024000LL, 173059204LL, 4},
    {567993600LL, 252028805LL, 5},
    {631152000LL, 315187206LL, 6},
    {662688000LL, 346723207LL, 7},
    {709948800LL, 393984008LL, 8},
    {741484800LL, 425520009LL, 9},
    {773020800LL, 457056010LL, 10},
    {820454400LL, 504489611LL, 11},
    {867715200LL, 551750412LL, 12},
    {915148800LL, 599184013LL, 13},
    {1136073600LL, 820108814LL, 14},
    {1230768000LL, 914803215LL, 15},
    {1341100800LL, 1025136016LL, 16},
    {1435708800LL, 1119744017LL, 17},
    {1483228800LL, 1167264018LL, 18},
}};

inline int getLeapOffsetByUnix(int64_t unix_sec) {
    for (auto it = kLeaps.rbegin(); it != kLeaps.rend(); ++it) {
        if (unix_sec >= it->unix_sec) return it->offset;
    }
    return 0;
}

inline int getLeapOffsetByGpst(int64_t gpst_sec) {
    for (auto it = kLeaps.rbegin(); it != kLeaps.rend(); ++it) {
        if (gpst_sec >= it->gpst_sec) return it->offset;
    }
    return 0;
}

inline const Leap *findLeapSecondByGpst(int64_t gpst_sec) {
    for (size_t i = 1; i < kLeaps.size(); ++i) {
        if (gpst_sec == kLeaps[i].gpst_sec - 1) return &kLeaps[i];
    }
    return nullptr;
}

inline Calendar calendarFromUnixNormal(int64_t unix_sec, int64_t rem_ns) {
    const int64_t days = floorDiv(unix_sec, kSecPerDay);
    const int64_t sec_in_day = unix_sec - days * kSecPerDay;

    Calendar ct = civilFromDays(days);
    ct.hour = static_cast<int>(sec_in_day / 3600);
    ct.minute = static_cast<int>((sec_in_day % 3600) / 60);
    ct.second = static_cast<double>(sec_in_day % 60) + static_cast<double>(rem_ns) / 1e9;
    return ct;
}

} // namespace

// -----------------------------------------------------------------------------
// ComTime: 输入构造
// -----------------------------------------------------------------------------
ComTime ComTime::fromGpstNs(int64_t ns) { return ComTime(ns); }

ComTime ComTime::fromUnixNs(int64_t ns) {
    int64_t unix_sec, rem_ns;
    fastDivMod(ns, unix_sec, rem_ns);
    const int offset = getLeapOffsetByUnix(unix_sec);
    return ComTime((unix_sec - kUnixGpsEpoch + offset) * kNsPerSec + rem_ns);
}

ComTime ComTime::fromUnixSec(double sec) { return fromUnixNs(secToNs(sec)); }

ComTime ComTime::fromGpst(const Gpst& gpst) {
    const int64_t week_ns = static_cast<int64_t>(gpst.week) * kSecPerWeek * kNsPerSec;
    return ComTime(week_ns + secToNs(gpst.sow));
}

ComTime ComTime::fromBdt(int week, double sow) {
    const int64_t week_ns = (static_cast<int64_t>(week) + kBdtWeekOffset) * kSecPerWeek * kNsPerSec;
    return ComTime(week_ns + secToNs(sow) + kGpstMinusBdt * kNsPerSec);
}

ComTime ComTime::fromGst(int week, double sow) { return fromGpst(week + kGstWeekOffset, sow); }

ComTime ComTime::fromUtc(const Calendar& cal) {
    if (cal.second >= 60.0) {
        // 处理正闰秒输入：yyyy-mm-dd 23:59:60.x
        Calendar prev = cal;
        prev.second = 59.0;
        const ComTime t_prev = fromUnixNs(calendarToContinuousNs(prev));
        const double frac = cal.second - 60.0;
        return t_prev + (1.0 + frac);
    }

    return fromUnixNs(calendarToContinuousNs(cal));
}

ComTime ComTime::fromGpst(const Calendar& cal) {
    return ComTime(calendarToContinuousNs(cal) - kUnixGpsEpoch * kNsPerSec);
}

ComTime ComTime::fromBdt(const Calendar& cal) {
    return ComTime(calendarToContinuousNs(cal) - kUnixGpsEpoch * kNsPerSec + kGpstMinusBdt * kNsPerSec);
}

// GST 与 GPST 共享相同的历元日期表示；周号偏移仅在 week+sow 形式下才需要换算。
ComTime ComTime::fromGst(const Calendar& cal) { return fromGpst(cal); }

// -----------------------------------------------------------------------------
// ComTime: 输出接口
// -----------------------------------------------------------------------------
double ComTime::toUnixSec() const {
    int64_t gpst_sec, rem_ns;
    fastDivMod(gpst_ns_, gpst_sec, rem_ns);

    int offset = getLeapOffsetByGpst(gpst_sec);

    // 修复：POSIX 闰秒平滑机制
    // 如果当前时刻恰好是物理上的闰秒本身，POSIX 时间戳不会前进，而是重复前一秒
    if (findLeapSecondByGpst(gpst_sec) != nullptr) {
        offset += 1;
    }

    const int64_t unix_sec = kUnixGpsEpoch + gpst_sec - offset;
    return static_cast<double>(unix_sec) + static_cast<double>(rem_ns) / 1e9;
}

Gpst ComTime::toGpst() const {
    int64_t gpst_sec, rem_ns;
    fastDivMod(gpst_ns_, gpst_sec, rem_ns);
    const int64_t w = floorDiv(gpst_sec, kSecPerWeek);
    const int64_t s = gpst_sec - w * kSecPerWeek;
    return {static_cast<int>(w), static_cast<double>(s) + static_cast<double>(rem_ns) / 1e9};
}

Calendar ComTime::toUtc() const {
    int64_t gpst_sec, rem_ns;
    fastDivMod(gpst_ns_, gpst_sec, rem_ns);

    if (const Leap *leap = findLeapSecondByGpst(gpst_sec)) {
        Calendar ct = calendarFromUnixNormal(leap->unix_sec - 1, 0);
        ct.hour = 23;
        ct.minute = 59;
        ct.second = 60.0 + static_cast<double>(rem_ns) / 1e9;
        return ct;
    }

    const int offset = getLeapOffsetByGpst(gpst_sec);
    const int64_t unix_sec = kUnixGpsEpoch + gpst_sec - offset;
    return calendarFromUnixNormal(unix_sec, rem_ns);
}

ComTime ComTime::roundedForStr(int& precision) const {
    precision = std::clamp(precision, 0, 9);
    return ComTime(roundToUnit(gpst_ns_, pow10Int(9 - precision)));
}

std::string ComTime::toUtcStr(int precision) const {
    const ComTime rounded = roundedForStr(precision);
    const Calendar utc = rounded.toUtc();

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(4) << utc.year << '-' << std::setw(2) << utc.month << '-' << std::setw(2)
        << utc.day << ' ' << std::setw(2) << utc.hour << ':' << std::setw(2) << utc.minute << ':';

    oss << std::fixed << std::setprecision(precision);
    if (utc.second < 10.0) oss << '0';
    oss << utc.second;
    return oss.str();
}

std::string ComTime::toGpstStr(int precision) const {
    const ComTime rounded = roundedForStr(precision);
    auto [week, sow] = rounded.toGpst();
    std::ostringstream oss;
    oss << '[' << week << ' ' << std::fixed << std::setprecision(precision) << sow << ']';
    return oss.str();
}

// -----------------------------------------------------------------------------
// 运算符重载
// -----------------------------------------------------------------------------
ComTime& ComTime::operator+=(double sec) {
    gpst_ns_ += secToNs(sec);
    return *this;
}
ComTime& ComTime::operator-=(double sec) {
    gpst_ns_ -= secToNs(sec);
    return *this;
}

ComTime ComTime::operator+(double sec) const { return ComTime(gpst_ns_ + secToNs(sec)); }
ComTime ComTime::operator-(double sec) const { return ComTime(gpst_ns_ - secToNs(sec)); }

double ComTime::operator-(const ComTime& rhs) const { return static_cast<double>(gpst_ns_ - rhs.gpst_ns_) / 1e9; }

} // namespace nav_core
