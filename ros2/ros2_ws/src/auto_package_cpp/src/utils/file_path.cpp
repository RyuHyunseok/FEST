#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <algorithm>
#include "auto_package_cpp/file_path.hpp"

namespace auto_package_cpp {

    // 경로 생성 함수
    std::string create_file_path(const std::string& package_name, const std::string& relative_path) {
        // 경로를 얻기
        std::string base_path = ament_index_cpp::get_package_share_directory(package_name);

        // Windows에서 경로 구분자가 \로 반환될 수 있기 때문에, 이를 /로 변경
        std::replace(base_path.begin(), base_path.end(), '\\', '/');  // \ -> /

        // 파일 경로 결합
        return base_path + "/" + relative_path;
    }

} // namespace auto_package_cpp