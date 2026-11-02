// Attitude diagnostic: never sends wheel/work-mode commands.
#include <robomaster/chassis.hpp>
#include <robomaster/can_streambuf.hpp>
#include <robomaster/crc.hpp>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop_handler(int) { stopped = 1; }
struct Socket {
    int fd = -1;
    ~Socket() { if (fd >= 0) ::close(fd); }
};
}

int main(int argc, char** argv)
{
    using namespace robomaster;
    const std::string iface = argc > 1 ? argv[1] : "can1";
    const std::string option = argc > 2 ? argv[2] : "";
    const bool imu_probe = option == "--imu" || option == "--imu-stream";
    const bool stream = option == "--stream" || option == "--imu-stream";
    if (argc > 3 || (argc > 2 && !stream && !imu_probe)) {
        std::cerr << "Usage: probe_attitude [can1] [--stream|--imu|--imu-stream]\n";
        return 2;
    }
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    try {
        Socket input;
        input.fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (input.fd < 0) throw can_error("socket");
        const can_filter filter{0x202, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};
        if (setsockopt(input.fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
            throw can_error("filter");
        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = if_nametoindex(iface.c_str());
        if (!address.can_ifindex || bind(input.fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
            throw can_error("bind");
        can_streambuf output(iface, 0x201);
        std::iostream out(&output);
        out.exceptions(std::ios::badbit | std::ios::failbit);
        // Diagnostic subscription ID 7; do not run with another DDS reader.
        const auto remove = [&] {
            package p{9, 3, 0x48, 4, false, false};
            p.data = {0, 9, 7};
            p.write_to(out); out.flush();
        };
        remove();
        package subscription{9, 3, 0x48, 3, false, false};
        subscription.data = {9, 7, 3, 0, 1};
        const auto uid = imu_probe ? get_uid<imu>() : get_uid<attitude>();
        subscription.data.insert(subscription.data.end(), uid.begin(), uid.end());
        subscription << uint16_t{10};
        subscription.write_to(out); out.flush();
        std::cout << (imu_probe ? "Accelerometer/gyro probe: raw protocol values.\n" : "Attitude probe.\n")
                  << (stream ? "Live data at 10 Hz; Ctrl-C stops.\n" : "Recording for 15 seconds.\n")
                  << "No heartbeat, work-mode or wheel commands.\n"
                  << "Values are diagnostic, not calibrated ground truth." << std::endl;
        const auto start = std::chrono::steady_clock::now();
        std::vector<uint8_t> buffer;
        unsigned samples = 0;
        try {
            while (!stopped && (stream || std::chrono::steady_clock::now() - start < std::chrono::seconds(15))) {
                pollfd event{input.fd, POLLIN, 0};
                if (poll(&event, 1, 100) <= 0) continue;
                can_frame frame{};
                if (read(input.fd, &frame, sizeof(frame)) != sizeof(frame)) throw can_error("read");
                buffer.insert(buffer.end(), frame.data, frame.data + std::min<unsigned>(frame.can_dlc, 8));
                while (buffer.size() >= 4) {
                    if (buffer[0] != 0x55 || crc8(119, {buffer.begin(), buffer.begin()+3}) != buffer[3]) {
                        buffer.erase(buffer.begin()); continue;
                    }
                    const size_t size = buffer[1] | ((buffer[2] & 3) << 8);
                    if (size < 13) { buffer.erase(buffer.begin()); continue; }
                    if (buffer.size() < size) break;
                    std::vector<uint8_t> bytes(buffer.begin(), buffer.begin()+size);
                    buffer.erase(buffer.begin(), buffer.begin()+size);
                    if (crc16(13970, {bytes.begin(), bytes.end()-2}) != (bytes[size-2] | (bytes[size-1]<<8))) continue;
                    if (bytes[9] != 0x48) continue;
                    if (!stream) {
                        std::cout << "DDS raw=";
                        for (auto b : bytes) std::cout << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
                        std::cout << std::dec << std::setfill(' ') << std::endl;
                    }
                    if (bytes[4] != 3 || bytes[5] != 9 || bytes[10] != 8 || size != (imu_probe ? 47u : 35u) || bytes[12] != 7) continue;
                    package p;
                    p.data.assign(bytes.begin()+13, bytes.end()-2);
                    if (imu_probe) {
                        const std::function<void(const metadata&, const imu&)> callback =
                            [&](const metadata&, const imu& value) {
                                ++samples;
                                const double elapsed = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - start).count();
                                std::cout << std::fixed << std::setprecision(5)
                                          << elapsed << "s | acc_xyz=" << value.acc_x << ',' << value.acc_y << ',' << value.acc_z
                                          << " | gyro_xyz=" << value.gyr_x << ',' << value.gyr_y << ',' << value.gyr_z << std::endl;
                            };
                        detail::dispatch_subscription<imu>(p, callback);
                        continue;
                    }
                    const std::function<void(const metadata&, const attitude&)> callback =
                        [&](const metadata& stamp, const attitude& a) {
                            ++samples;
                            if (stream) {
                                const double elapsed = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - start).count();
                                std::cout << std::fixed << std::setprecision(2)
                                          << elapsed << "s | pitch=" << a.pitch
                                          << " deg | roll=" << a.roll << " deg | yaw=" << a.yaw << " deg" << std::endl;
                            } else {
                                std::cout << "sample=" << samples << " timestamp=" << stamp.time_ms << '/' << stamp.time_ns
                                          << " yaw=" << a.yaw << " pitch=" << a.pitch << " roll=" << a.roll << std::endl;
                            }
                        };
                    detail::dispatch_subscription<attitude>(p, callback);
                }
            }
        } catch (...) { try { remove(); } catch (...) {} throw; }
        remove();
        std::cout << "Finished; decoded samples=" << samples << std::endl;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
