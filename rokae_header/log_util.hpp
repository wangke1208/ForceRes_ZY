#pragma once
#include <3rd/spdlog/async.h>
#include <3rd/spdlog/sinks/rotating_file_sink.h>
#include <3rd/spdlog/spdlog.h>


namespace RokaeApi {
// 日志的配置项
struct LogConfig {
    std::string level;
    std::string path;
    int64_t size;
    int count;
};

// 日志的单例模式
class new_Logger {
   public:
    static new_Logger* getInstance() {
        static new_Logger instance;
        return &instance;
    }

    // c++14返回值可设置为auto
    std::shared_ptr<spdlog::logger> getLogger() { return loggerPtr; }

    void Init(const LogConfig& conf);

    std::string GetLogLevel();

    void SetLogLevel(const std::string& level);

   private:
    new_Logger() = default;
    std::shared_ptr<spdlog::logger> loggerPtr;
};

// 日志相关操作的宏封装
#define INITLOG(conf) new_Logger::getInstance()->Init(conf)
#define GETLOGLEVEL() new_Logger::getInstance()->GetLogLevel()
#define SETLOGLEVEL(level) new_Logger::getInstance()->SetLogLevel(level)
#define BASELOG(new_Logger, level, ...) (new_Logger)->log(spdlog::source_loc{__FILE__, __LINE__, __func__}, level, __VA_ARGS__)
#define LOG_TRACE(...) BASELOG(new_Logger::getInstance()->getLogger(), spdlog::level::trace, __VA_ARGS__)
#define LOG_DEBUG(...) BASELOG(new_Logger::getInstance()->getLogger(), spdlog::level::debug, __VA_ARGS__)
#define LOG_INFO(...) BASELOG(new_Logger::getInstance()->getLogger(), spdlog::level::info, __VA_ARGS__)
#define LOG_WARN(...) BASELOG(new_Logger::getInstance()->getLogger(), spdlog::level::warn, __VA_ARGS__)
#define LOG_ERROR(...) BASELOG(new_Logger::getInstance()->getLogger(), spdlog::level::err, __VA_ARGS__)
#define LOG_CRITICAL(...) BASELOG(new_Logger::getInstance()->getLogger(), spdlog::level::critical, __VA_ARGS__)
}  // namespace RokaeApi
