#ifndef BAGANALYZER_H
#define BAGANALYZER_H

#include <filesystem>
#include <optional>
#include <string>
#include <utility>   // for std::pair
#include <algorithm> // Include for std::max_element
#include <stdexcept> // Include for std::runtime_error
#include <filesystem>
#include <iostream>
#include <vector>
#include <string>
#include <optional>
#include <chrono>
#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_cpp/info.hpp"
#include <chrono>
#include <stdexcept>

namespace fs = std::filesystem;

class BagAnalyzer
{
public:
    BagAnalyzer();
    std::optional<fs::path> findNewestMcapFile(const std::string &folderPath);
    std::pair<std::string, size_t> analyzeBag(const std::string &filePath, const std::string &topicName);
};

#endif // BAGFILEHANDLER_HPP
