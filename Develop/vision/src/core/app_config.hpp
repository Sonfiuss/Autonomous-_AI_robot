#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <cstdlib>
#include <iostream>

namespace vision
{
    struct AppConfig
    {
        std::string videoPath;
        std::string modelPath;
        std::string outputPath;
        bool showWindow = true;
        double inferenceFps = 6.0;  // depth refresh rate
        int workerCount = 3;        // inference worker thread count
        int inputQueueCapacity = 8; // bounded output queue capacity for each worker
    };

    inline std::string firstExistingPath(const std::vector<std::string> &candidates)
    {
        namespace fs = std::filesystem;
        for (const auto &p : candidates)
        {
            if (fs::exists(p))
            {
                return p;
            }
        }
        return candidates.empty() ? "" : candidates.front();
    }

    inline AppConfig parseArgs(int argc, char **argv)
    {
        AppConfig cfg;
        cfg.videoPath = firstExistingPath({
            "vision/assets/CupOnTable.mp4",
            "assets/CupOnTable.mp4",
        });
        cfg.modelPath = firstExistingPath({
            "vision/model/depth_anything_v2.onnx",
            "model/depth_anything_v2.onnx",
        });
        cfg.outputPath = firstExistingPath({
            "vision/assets/CupOnTable_depth.mp4",
            "assets/CupOnTable_depth.mp4",
        });

        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            if (arg == "--video" && i + 1 < argc)
            {
                cfg.videoPath = argv[++i];
            }
            else if (arg == "--model" && i + 1 < argc)
            {
                cfg.modelPath = argv[++i];
            }
            else if (arg == "--output" && i + 1 < argc)
            {
                cfg.outputPath = argv[++i];
            }
            else if (arg == "--no-show")
            {
                cfg.showWindow = false;
            }
            else if (arg == "--help" || arg == "-h")
            {
                std::cout
                    << "Usage: main [--video <path>] [--model <path>] [--output <path>] [--no-show] [--fps N] [--workers N] [--queue N]\n ";
                std::exit(0);
            }
        }

        return cfg;
    }
} // namespace vision