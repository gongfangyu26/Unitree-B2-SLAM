#include <unitree/robot/b2/front_video/front_video_client.hpp>

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

int main()
{
    unitree::robot::ChannelFactory::Instance()->Init(0, "enP8p1s0");

    unitree::robot::b2::FrontVideoClient camera;
    camera.SetTimeout(2.0f);
    camera.Init();

    std::vector<uint8_t> current;
    std::vector<uint8_t> previous;

    int total_reads = 0;
    int new_frames = 0;

    auto start = std::chrono::steady_clock::now();
    auto last_new_frame = start;

    std::cout << "Testing actual B2 camera frame update rate for 5 seconds..."
              << std::endl;

    while (true)
    {
        auto now = std::chrono::steady_clock::now();

        double elapsed =
            std::chrono::duration<double>(now - start).count();

        if (elapsed >= 5.0)
            break;

        current.clear();

        int32_t ret = camera.GetImageSample(current);
        ++total_reads;

        if (ret == 0 && !current.empty())
        {
            if (previous.empty() || current != previous)
            {
                auto frame_time = std::chrono::steady_clock::now();

                if (!previous.empty())
                {
                    double interval_ms =
                        std::chrono::duration<double, std::milli>(
                            frame_time - last_new_frame).count();

                    std::cout << "New frame " << new_frames + 1
                              << " | bytes=" << current.size()
                              << " | interval=" << interval_ms
                              << " ms" << std::endl;
                }

                previous = current;
                last_new_frame = frame_time;
                ++new_frames;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    double duration =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();

    std::cout << "\n===== Actual Frame Test =====" << std::endl;
    std::cout << "Duration: " << duration << " s" << std::endl;
    std::cout << "API reads: " << total_reads << std::endl;
    std::cout << "Changed frames: " << new_frames << std::endl;
    std::cout << "Estimated actual FPS: "
              << new_frames / duration << std::endl;

    return 0;
}
