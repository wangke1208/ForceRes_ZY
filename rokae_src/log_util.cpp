#include "rokae_header/log_util.hpp"

namespace RokaeApi {

void new_Logger::Init(const LogConfig& conf) {
    // 先确保目录存在
    size_t lastSlash = conf.path.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        std::string dir = conf.path.substr(0, lastSlash);
        if (!create_dirs(dir)) {
            std::cerr << "[Logger Error] Failed to create log directory: " << dir << std::endl;
            return;
        }
    }

    try {
        // 创建文件 sink（日志轮转）
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(conf.path, conf.size, conf.count);

        // 创建控制台 sink（彩色输出）
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        // 设置两个 sink 的日志等级一致
        spdlog::level::level_enum log_level = spdlog::level::from_str(conf.level);
        file_sink->set_level(log_level);
        console_sink->set_level(log_level);

        // 合并 sink
        std::vector<spdlog::sink_ptr> sinks { file_sink, console_sink };
        loggerPtr = std::make_shared<spdlog::logger>("multi_sink", sinks.begin(), sinks.end());

        // 设置异步输出（可选）
        spdlog::register_logger(loggerPtr);

        // 设置日志格式
        loggerPtr->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [thread %t] [%s %!:%#] %v");

        // 设置日志等级
        loggerPtr->set_level(log_level);
        loggerPtr->flush_on(log_level);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "[Logger Exception] Init failed: " << ex.what() << std::endl;
    }
}

std::string new_Logger::GetLogLevel() {
    auto level = loggerPtr->level();
    return spdlog::level::to_string_view(level).data();
}

void new_Logger::SetLogLevel(const std::string& log_level) {
    auto level = spdlog::level::from_str(log_level);
    if (level == spdlog::level::off) {
        LOG_WARN("Given invalid log level {}", log_level);
    } else {
        loggerPtr->set_level(level);
        loggerPtr->flush_on(level);
    }
}

}  // namespace RokaeApi
