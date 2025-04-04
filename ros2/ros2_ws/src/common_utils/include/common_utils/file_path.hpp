#ifndef COMMON_UTILS_FILE_PATH_HPP
#define COMMON_UTILS_FILE_PATH_HPP

#include <string>

namespace common_utils {

/**
 * @brief 패키지 내 파일 경로를 생성하는 함수
 * 
 * @param package_name 패키지 이름
 * @param relative_path 패키지 기준 상대 경로
 * @return std::string 절대 경로
 */
std::string create_file_path(const std::string& package_name, const std::string& relative_path);

} // namespace common_utils

#endif // COMMON_UTILS_FILE_PATH_HPP 