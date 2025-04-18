#pragma once
#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#endif

namespace RokaeApi {

// 日志的配置项
struct LogConfig {
    std::string level;
    std::string path;
    int64_t size;
    int count;
};

// 工具函数：判断路径是否存在
inline bool path_exists(const std::string& path) {
#ifdef _WIN32
    return _access(path.c_str(), 0) == 0;
#else
    struct stat info;
    return stat(path.c_str(), &info) == 0;
#endif
}

// 工具函数：创建单层目录
inline bool create_dir(const std::string& path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

// 工具函数：递归创建多层目录
inline bool create_dirs(const std::string& path) {
    std::string current;
    for (size_t i = 0; i < path.size(); ++i) {
        char c = path[i];
        current += c;
        if (c == '/' || c == '\\') {
            if (!path_exists(current)) {
                if (!create_dir(current)) return false;
            }
        }
    }
    if (!path_exists(current)) {
        return create_dir(current);
    }
    return true;
}

// 日志的单例模式
class new_Logger {
   public:
    static new_Logger* getInstance() {
        static new_Logger instance;
        return &instance;
    }

    std::shared_ptr<spdlog::logger> getLogger() { return loggerPtr; }

    void Init(const LogConfig& conf);

    std::string GetLogLevel();

    void SetLogLevel(const std::string& level);
    void LogEigenMatrix(const Eigen::Matrix<double, 6, Eigen::Dynamic>& mat, const std::string& name);

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
#define SPD_EIGEN_MATRIX(MAT)                     \
    do {                                          \
        std::stringstream ss;                     \
        ss << MAT;                                \
        spdlog::info("{} =\n{}", #MAT, ss.str()); \
    } while (0)
}  // namespace RokaeApi
