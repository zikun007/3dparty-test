#include "nav_core/systime.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

int g_total = 0;
int g_failures = 0;

#define CHECK(expr)                                                           \
    do {                                                                      \
        ++g_total;                                                            \
        if (!(expr)) {                                                        \
            ++g_failures;                                                     \
            std::cerr << "FAIL [" << __LINE__ << "]: " << #expr << "\n";    \
        }                                                                     \
    } while (0)

bool near(double lhs, double rhs, double eps = 1e-9) {
    return std::abs(lhs - rhs) <= eps;
}

void test_gps_epoch() {
    const nav_core::ComTime t = nav_core::ComTime::fromUtc(1980, 1, 6, 0, 0, 0.0);
    const nav_core::Gpst gpst = t.toGpst();

    CHECK(gpst.week == 0);
    CHECK(near(gpst.sow, 0.0));
    CHECK(near(t.toUnixSec(), 315964800.0));
}

void test_utc_roundtrip_and_arithmetic() {
    const nav_core::ComTime t = nav_core::ComTime::fromUtc(2017, 1, 1, 0, 0, 0.25);
    const nav_core::Calendar utc = t.toUtc();

    CHECK(utc.year == 2017 && utc.month == 1 && utc.day == 1);
    CHECK(utc.hour == 0 && utc.minute == 0 && near(utc.second, 0.25));

    const nav_core::ComTime shifted = t + 10.5;
    CHECK(near(shifted - t, 10.5));
    CHECK(shifted > t);
}

void test_gnss_timescales() {
    const nav_core::ComTime gpst = nav_core::ComTime::fromGpst(1356, 14.0);
    const nav_core::ComTime bdt = nav_core::ComTime::fromBdt(0, 0.0);
    CHECK(near(gpst - bdt, 0.0));

    const nav_core::ComTime gst = nav_core::ComTime::fromGst(0, 0.0);
    const nav_core::ComTime gpst_from_gst_epoch = nav_core::ComTime::fromGpst(1024, 0.0);
    CHECK(gst == gpst_from_gst_epoch);
}

void test_formatting() {
    const nav_core::ComTime t = nav_core::ComTime::fromGpst(1, 2.5);
    CHECK(t.toGpstStr(1) == "[1 2.5]");
}

} // namespace

int main() {
    test_gps_epoch();
    test_utc_roundtrip_and_arithmetic();
    test_gnss_timescales();
    test_formatting();

    std::cout << "systime: " << (g_total - g_failures) << " / " << g_total << " passed\n";
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
