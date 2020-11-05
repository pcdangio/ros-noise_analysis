#include "data_set.h"

#include <rosbag/view.h>

#include <boost/filesystem/path.hpp>

data_set::data_set()
{

}

bool data_set::load_bag(std::string bag_path)
{
    // Try to load the bag file.
    try
    {
        data_set::m_bag.open(bag_path, rosbag::bagmode::Read);
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to load bag file (" << error.what() << ")");
        return false;
    }

    // Emit bag loaded.
    emit data_set::bag_loaded();

    return true;
}
std::string data_set::bag_name() const
{
    boost::filesystem::path bag_path(data_set::m_bag.getFileName());
    return bag_path.filename().string();
}
std::vector<std::string> data_set::bag_topics() const
{
    rosbag::View view(data_set::m_bag);

    // Get information about connections.
    auto connections = view.getConnections();

    // Iterate through the connections to get the topic names.
    std::vector<std::string> topics;
    for(auto connection = connections.cbegin(); connection != connections.cend(); ++connection)
    {
        topics.push_back((*connection)->topic);
    }

    return topics;
}
