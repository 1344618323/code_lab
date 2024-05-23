#include "utils.h"

#include <boost/filesystem.hpp>

namespace vo_lab {
void mkdir(const std::string& path) {
    if (!boost::filesystem::exists(path)) {
        boost::filesystem::create_directories(path);
    }
}
}  // namespace vo_lab