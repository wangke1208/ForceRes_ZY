#include "rokae_header/log_util.hpp"

#include <3rd/spdlog/sinks/rotating_file_sink.h>
#include <3rd/spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>
#include <iostream>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

namespace RokaeApi {

std::shared_ptr<spdlog::logger> LogUtil::logger = nullptr;

void LogUtil::initLogger(const std::string& logPath) {
    if (logger) return;

    createDirectoryIfNotExists(logPath);

    try {
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            logPath + "/RokaeLog.txt", MAX_LOG_FILE_SIZE, MAX_LOG_FILES);

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        // 设置 sink 格式
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
        console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

        std::vector<spdlog::sink_ptr> sinks { console_sink, file_sink };

        logger = std::make_shared<spdlog::logger>("combined_logger", sinks.begin(), sinks.end());
        spdlog::register_logger(logger);
        logger->set_level(spdlog::level::info);
        logger->flush_on(spdlog::level::info);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "Log initialization failed: " << ex.what() << std::endl;
    }
}

void LogUtil::logInfo(const std::string& message) {
    if (logger) {
        logger->info(message);
    }
}

void LogUtil::logError(const std::string& message) {
    if (logger) {
        logger->error(message);
    }
}

void LogUtil::createDirectoryIfNotExists(const std::string& path) {
#ifdef _WIN32
    if (_mkdir(path.c_str()) != 0 && errno != EEXIST) {
        std::cerr << "Error creating directory: " << path << std::endl;
    }
#else
    struct stat st = {0};
    if (stat(path.c_str(), &st) == -1) {
        if (mkdir(path.c_str(), 0755) != 0) {
            std::cerr << "Error creating directory: " << path << std::endl;
        }
    }
#endif
}

}  // namespace RokaeApi
