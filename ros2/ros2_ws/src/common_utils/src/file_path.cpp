#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <algorithm>
#include "common_utils/file_path.hpp"

namespace common_utils {

    std::string create_file_path(const std::string& package_name, const std::string& relative_path) {
        std::string base_path = ament_index_cpp::get_package_share_directory(package_name);
        std::replace(base_path.begin(), base_path.end(), '\\', '/');
        return base_path + "/" + relative_path;
    }

} // namespace common_utils 