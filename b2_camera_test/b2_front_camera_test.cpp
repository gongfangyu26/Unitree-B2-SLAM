#include <unitree/robot/b2/front_video/front_video_client.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>

int main()
{
    unitree::robot::ChannelFactory::Instance()->Init(0, "enP8p1s0");

    unitree::robot::b2::FrontVideoClient camera;
    camera.SetTimeout(2.0f);
    camera.Init();

    const int N = 20;
    int success_count = 0;
    double total_ms = 0.0;

    std::cout << "Start B2 front camera continuous test: "
              << N << " frames" << std::endl;

    for (int i = 1; i <= N; ++i)
    {
        std::vector<uint8_t> image_sample;

        auto t1 = std::chrono::steady_clock::now();
        int32_t ret = camera.GetImageSample(image_sample);
        auto t2 = std::chrono::steady_clock::now();

        double ms =
            std::chrono::duration<double, std::milli>(t2 - t1).count();

        std::cout << "Frame " << std::setw(2) << i
                  << " | ret=" << ret
                  << " | bytes=" << image_sample.size()
                  << " | time=" << std::fixed << std::setprecision(2)
                  << ms << " ms" << std::endl;

        if (ret == 0 && !image_sample.empty())
        {
            ++success_count;
            total_ms += ms;

            // 只保存第1张和最后1张，避免产生大量文件
            if (i == 1 || i == N)
            {
                std::string filename =
                    "b2_front_frame_" + std::to_string(i) + ".jpg";

                std::ofstream file(filename, std::ios::binary);
                file.write(
                    reinterpret_cast<const char*>(image_sample.data()),
                    image_sample.size()
                );
                file.close();
            }
        }
    }

    std::cout << "\n===== Test Result =====" << std::endl;
    std::cout << "Success: " << success_count << "/" << N << std::endl;

    if (success_count > 0)
    {
        double avg_ms = total_ms / success_count;
        std::cout << "Average request time: "
                  << avg_ms << " ms" << std::endl;

        if (avg_ms > 0)
            std::cout << "Approx request rate: "
                      << 1000.0 / avg_ms << " Hz" << std::endl;
    }

    return 0;
}
