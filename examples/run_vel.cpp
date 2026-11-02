#include <iostream>
#include <chrono>
#include <thread>

#include <robomaster/can_streambuf.hpp>
#include <robomaster/chassis.hpp>

int main(int, char**)
{
    auto can = robomaster::can_streambuf("can0", 0x201);
    std::iostream io(&can);
    robomaster::chassis chassis(io);

    chassis.send_workmode(1);

    // Send heartbeat for 1 second
    for (int i = 0; i < 100; i++)
    {
        chassis.send_heartbeat();
        chassis.send_wheel_speed(0, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Rotate counter-clockwise for 3 seconds using the physically validated
    // wheel order: front right, front left, rear left, rear right.
    for (int i = 0; i < 300; i++)
    {
        chassis.send_heartbeat();
        chassis.send_wheel_speed(30, -30, -30, 30);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop for 1 second
    for (int i = 0; i < 100; i++)
    {
        chassis.send_heartbeat();
        chassis.send_wheel_speed(0, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
