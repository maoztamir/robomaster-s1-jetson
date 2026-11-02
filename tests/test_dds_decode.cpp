#include <robomaster/chassis.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

robomaster::package attitude_packet()
{
    robomaster::package p;
    // Fixed wire fixture: timestamps 1/2, yaw=1, pitch=-2, roll=0.5.
    p.data = {1,0,0,0, 2,0,0,0,
              0,0,128,63, 0,0,0,192, 0,0,0,63};
    return p;
}
}

int main()
{
    using namespace robomaster;
    try {
        auto single = attitude_packet();
        bool called = false;
        const std::function<void(const metadata&, const attitude&)> callback =
            [&](const metadata& stamp, const attitude& value) {
                called = true;
                require(stamp.time_ms == 1 && stamp.time_ns == 2, "timestamp order");
                require(value.yaw == 1 && value.pitch == -2 && value.roll == 0.5,
                        "attitude order");
            };
        detail::dispatch_subscription<attitude>(single, callback);
        require(called && single.data.empty(), "single subscription consumption");

        auto multi = attitude_packet();
        // Battery: 12000 mV, temperature raw=250, current=-100, percent=75.
        const std::deque<uint8_t> battery_bytes = {224,46, 250,0, 156,255,255,255, 75};
        multi.data.insert(multi.data.end(), battery_bytes.begin(), battery_bytes.end());
        called = false;
        const std::function<void(const metadata&, const attitude&, const battery&)> combined =
            [&](const metadata& stamp, const attitude& value, const battery& bat) {
                called = true;
                callback(stamp, value);
                require(bat.adc_val == 12000 && bat.temperature == 250 &&
                        bat.current == -100 && bat.percent == 75, "multi-subscription order");
            };
        detail::dispatch_subscription<attitude, battery>(multi, combined);
        require(called && multi.data.empty(), "multi subscription consumption");

        auto imu_packet = attitude_packet();
        // Reuse the first three floats as acceleration, then gyro=(3,-4,0).
        const std::deque<uint8_t> gyro_bytes = {0,0,64,64, 0,0,128,192, 0,0,0,0};
        imu_packet.data.insert(imu_packet.data.end(), gyro_bytes.begin(), gyro_bytes.end());
        called = false;
        const std::function<void(const metadata&, const imu&)> imu_callback =
            [&](const metadata& stamp, const imu& value) {
                called = true;
                require(stamp.time_ms == 1 && stamp.time_ns == 2, "IMU timestamp");
                require(value.acc_x == 1 && value.acc_y == -2 && value.acc_z == 0.5 &&
                        value.gyr_x == 3 && value.gyr_y == -4 && value.gyr_z == 0, "IMU axes order");
            };
        detail::dispatch_subscription<imu>(imu_packet, imu_callback);
        require(called && imu_packet.data.empty(), "IMU subscription consumption");

        auto short_packet = attitude_packet();
        short_packet.data.pop_back();
        called = false;
        bool rejected = false;
        try { detail::dispatch_subscription<attitude>(short_packet, callback); }
        catch (const protocol_error&) { rejected = true; }
        require(rejected && !called, "truncated packet must not invoke callback");
        std::cout << "DDS decoding checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
