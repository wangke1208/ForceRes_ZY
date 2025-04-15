#ifndef ROKAE_HEADER_LOG_UTIL_HPP
#define ROKAE_HEADER_LOG_UTIL_HPP
#include <3rd/spdlog/spdlog.h>
#include <sys/stat.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rokae_header/data_structure_define.hpp"

#define LOG_VECTOR_INFO(vec) RokaeApi::LogUtil::logVector(#vec, vec)
#define LOG_VECTOR_ERROR(vec) RokaeApi::LogUtil::logVectorError(#vec, vec)
#define LOG_ERROR(msg) RokaeApi::LogUtil::logError(msg)
#define LOG_INFO(msg) RokaeApi::LogUtil::logInfo(msg)
namespace RokaeApi {

class LogUtil {
   public:
    static void initLogger(const std::string& logPath);

    static void logInfo(const std::string& message);
    static void logError(const std::string& message);

    template <typename T>
    static void logVector(const std::string& name, const std::vector<T>& vec) {
        if (logger) {
            logger->info("{} = [{}]", name, vectorToString(vec));
        }
    }

    template <typename T>
    static void logVectorError(const std::string& name, const std::vector<T>& vec) {
        if (logger) {
            logger->error("{} = [{}]", name, vectorToString(vec));
        }
    }

   private:
    static std::shared_ptr<spdlog::logger> logger;

    static void createDirectoryIfNotExists(const std::string& path);

    static constexpr size_t MAX_LOG_FILE_SIZE = 10 * 1024 * 1024;  // 10MB
    static constexpr size_t MAX_LOG_FILES = 10;

    template <typename T>
    static std::string vectorToString(const std::vector<T>& vec) {
        std::ostringstream oss;
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << vec[i];
        }
        return oss.str();
    }
};

}  // namespace RokaeApi

#endif  // LOG_UTIL_HPP
