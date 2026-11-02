#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <robomaster/can_streambuf.hpp>
#include <robomaster/chassis.hpp>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt_handler(int) { interrupted = 1; }
}

int main(int argc, char** argv)
{
    const std::string interface = argc > 1 ? argv[1] : "can1";
    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);
    try {
        robomaster::can_streambuf can(interface, 0x201);
        std::iostream io(&can);
        io.exceptions(std::ios::badbit | std::ios::failbit);
        robomaster::chassis chassis(io);
        chassis.send_workmode(1);
        io.flush();
        const auto started = std::chrono::steady_clock::now();
        int next_report = 0;
        std::cout << "C++ reference heartbeat: zero wheels only, 240 seconds. Ctrl-C stops.\n";
        while (!interrupted) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - started).count();
            if (elapsed >= 240) break;
            chassis.send_heartbeat();
            chassis.send_wheel_speed(0, 0, 0, 0);
            io.flush();
            if (elapsed >= next_report) {
                std::cout << elapsed << "s | heartbeat + zero sent (controller acceptance unknown)"
                          << std::endl;
                next_report += 15;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        chassis.send_heartbeat();
        chassis.send_wheel_speed(0, 0, 0, 0);
        io.flush();
        std::cout << "Stopped; heartbeat ended. Chassis service remains stopped." << std::endl;
    } catch (const std::exception& error) {
        // Every command in this program is zero, including before any failure.
        std::cerr << "CAN test failed: " << error.what() << std::endl;
        return 1;
    }
}
