#ifndef FILE_PATH_HPP
#define FILE_PATH_HPP

#include <string>

namespace auto_package_cpp {

/**
 * @brief 패키지 내 파일 경로를 생성하는 함수
 * 
 * @param package_name 패키지 이름
 * @param relative_path 패키지 기준 상대 경로
 * @return std::string 절대 경로
 */
std::string create_file_path(const std::string& package_name, const std::string& relative_path);

} // namespace auto_package_cpp

#endif // FILE_PATH_HPP 