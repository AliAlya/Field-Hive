#include <smartspectra/container/foreground_container.hpp>
#include <smartspectra/container/settings.hpp>
#include <physiology/modules/messages/metrics.h>
#include <physiology/modules/messages/status.h>
#include <glog/logging.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>
#include <cstring>

// UDP
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace presage::smartspectra;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    std::string api_key;
    if (argc > 1) {
        api_key = argv[1];
    } else if (const char* env_key = std::getenv("SMARTSPECTRA_API_KEY")) {
        api_key = env_key;
    } else {
        std::cout << "Usage: ./hello_vitals YOUR_API_KEY\n";
        return 1;
    }

    // =====================================================
    // UDP Setup
    // =====================================================
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed\n";
        return 1;
    }

    sockaddr_in pi_addr{};
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(5005);

    // 🔴 Use your confirmed working Pi IP
    const char* pi_ip = "172.17.214.32";

    if (inet_pton(AF_INET, pi_ip, &pi_addr.sin_addr) <= 0) {
        std::cerr << "Invalid Pi IP address\n";
        return 1;
    }

    std::cout << "UDP initialized → " << pi_ip << ":5005\n";

    // Send immediate boot packet (network test)
    std::string boot_msg = "{ \"boot\": 1 }";
    sendto(sock,
           boot_msg.c_str(),
           boot_msg.size(),
           0,
           (sockaddr*)&pi_addr,
           sizeof(pi_addr));

    std::cout << "Boot packet sent.\n";

    // =====================================================
    // Reconnect Loop
    // =====================================================
    while (true) {
        try {
            std::cout << "\n=== Connecting to stream ===\n";

            container::settings::Settings<
                container::settings::OperationMode::Continuous,
                container::settings::IntegrationMode::Rest
            > settings;

            settings.video_source.device_index = -1;
            settings.video_source.auto_lock = false;
            settings.video_source.input_video_path =
                "rtsp://172.17.214.32:8554/unicast";

            settings.headless = true;
            settings.enable_edge_metrics = true;
            settings.verbosity_level = 1;
            settings.continuous.preprocessed_data_buffer_duration_s = 0.5;
            settings.integration.api_key = api_key;

            auto container =
                std::make_unique<container::CpuContinuousRestForegroundContainer>(settings);

            // =====================================================
            // Metrics Callback
            // =====================================================
            auto status = container->SetOnCoreMetricsOutput(
                [&sock, &pi_addr](const presage::physiology::MetricsBuffer& metrics,
                                  int64_t timestamp) {

                    float pulse = 0.0f;
                    float breathing = 0.0f;

                    if (!metrics.pulse().rate().empty())
                        pulse = metrics.pulse().rate().rbegin()->value();

                    if (!metrics.breathing().rate().empty())
                        breathing = metrics.breathing().rate().rbegin()->value();

                    if (pulse > 30 && pulse < 200) {

                        std::cout << "Heart Rate: "
                                  << pulse
                                  << " BPM | Breathing: "
                                  << breathing
                                  << " RPM\n";

                        std::stringstream ss;
                        ss << "{ \"hr\": " << pulse
                           << ", \"br\": " << breathing
                           << " }";

                        std::string msg = ss.str();

                        sendto(sock,
                               msg.c_str(),
                               msg.size(),
                               0,
                               (sockaddr*)&pi_addr,
                               sizeof(pi_addr));

                        std::cout << "Vitals packet sent.\n";
                    }

                    return absl::OkStatus();
                }
            );

            if (!status.ok()) {
                std::cerr << "Failed to set metrics callback\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            // =====================================================
            // Status Callback
            // =====================================================
            status = container->SetOnStatusChange(
                [&sock, &pi_addr](presage::physiology::StatusValue imaging_status) {

                    std::string desc =
                        presage::physiology::GetStatusDescription(
                            imaging_status.value());

                    std::cout << "Status: " << desc << "\n";

                    if (desc.find("No faces") != std::string::npos) {

                        std::string msg = "{ \"hr\": 0, \"br\": 0 }";

                        sendto(sock,
                               msg.c_str(),
                               msg.size(),
                               0,
                               (sockaddr*)&pi_addr,
                               sizeof(pi_addr));

                        std::cout << "No-face packet sent.\n";
                    }

                    return absl::OkStatus();
                }
            );

            if (!status.ok()) {
                std::cerr << "Failed to set status callback\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            if (auto s = container->Initialize(); !s.ok()) {
                std::cerr << "Initialize failed: "
                          << s.message()
                          << "\n";
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            std::cout << "Processing stream...\n";

            if (auto s = container->Run(); !s.ok()) {
                std::cerr << "Stream stopped: "
                          << s.message()
                          << "\n";
            }

        } catch (const std::exception& e) {
            std::cerr << "Exception: "
                      << e.what()
                      << "\n";
        }

        std::cout << "Stream lost. Reconnecting in 2 seconds...\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    close(sock);
    return 0;
}

