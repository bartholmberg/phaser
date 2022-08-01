#include "phaser/common/data/file-system-helper.h"

//#include <boost/filesystem.hpp>
#include <filesystem>
#include <glog/logging.h>

namespace data {

void FileSystemHelper::readDirectory(
    const std::string& directory, std::vector<std::string>* files) {
  std::filesystem::path p(directory);
  if (!std::filesystem::exists(p)) {
    LOG(FATAL) << "PLY directory does not exist!";
  }

  std::filesystem::directory_iterator start(p);
  std::filesystem::directory_iterator end;
  std::transform(
      start, end, std::back_inserter(*files),
      [](const std::filesystem::directory_entry& entry) {
        return entry.path().filename().string();
      });
  std::sort(files->begin(), files->end());
}

}  // namespace data
