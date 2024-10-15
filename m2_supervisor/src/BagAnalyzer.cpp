#include "BagAnalyzer.h"

BagAnalyzer::BagAnalyzer() {}


std::optional<fs::path> BagAnalyzer::findNewestMcapFile(const std::string &folderPath)
{
    std::optional<fs::path> greatestFolder;
    for (const auto &entry : fs::directory_iterator(folderPath))
    {
        if (entry.is_directory())
        {
            if (!greatestFolder || entry.path().filename() > greatestFolder->filename())
            {
                greatestFolder = entry.path();
            }
        }
    }

    if (!greatestFolder)
    {
        return std::nullopt; // No subfolders found
    }

    std::optional<fs::path> newestFile;
    auto newestMtime = fs::file_time_type::min();
    for (const auto &entry : fs::directory_iterator(*greatestFolder))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".mcap")
        {
            auto fileMtime = fs::last_write_time(entry.path());
            if (fileMtime > newestMtime)
            {
                newestFile = entry.path();
                newestMtime = fileMtime;
            }
        }
    }

    return newestFile;
}

std::pair<std::string, size_t> BagAnalyzer::analyzeBag(const std::string &filePath, const std::string &topicName)
{
    rosbag2_cpp::Info bagInfo;
    const rosbag2_storage::BagMetadata metadata = bagInfo.read_metadata(filePath, "mcap");

    for (const auto &topicInfo : metadata.topics_with_message_count)
    {
        std::cout << "Analyzing Messages: " << topicInfo.topic_metadata.name << std::endl;
        if (topicInfo.topic_metadata.name == topicName)
        {
            return std::make_pair(topicInfo.topic_metadata.name, topicInfo.message_count);
        }
    }

    throw std::runtime_error("Topic name not found in the metadata.");
}