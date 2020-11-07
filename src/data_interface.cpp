#include "data_interface.h"

#include <rosbag/view.h>

#include <boost/filesystem/path.hpp>

data_interface::data_interface()
{

}

bool data_interface::load_bag(std::string bag_path)
{
    // Try to load the bag file.
    try
    {
        // Close any open bag files.
        data_interface::m_bag.close();
        // Open the new bag file.
        data_interface::m_bag.open(bag_path, rosbag::bagmode::Read);
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to load bag file (" << error.what() << ")");
        return false;
    }

    // Emit bag loaded.
    emit data_interface::bag_loaded();

    return true;
}
std::string data_interface::bag_name() const
{
    boost::filesystem::path bag_path(data_interface::m_bag.getFileName());
    return bag_path.filename().string();
}
std::set<std::string> data_interface::bag_topics() const
{
    // Create set for tracking unique topic names.
    std::set<std::string> unique_topics;

    if(data_interface::m_bag.isOpen())
    {
        // Use a view to get the connections.
        rosbag::View view(data_interface::m_bag);
        auto connections = view.getConnections();
        // Iterate through the connections to get the topic names.
        for(auto connection = connections.cbegin(); connection != connections.cend(); ++connection)
        {
            unique_topics.insert((*connection)->topic);
        }
    }

    return unique_topics;
}

bool data_interface::get_definition_tree(const std::string& topic, message_introspection::definition_tree_t &definition_tree) const
{
    // Check if bag file is open.
    if(!data_interface::m_bag.isOpen())
    {
        return false;
    }

    // Use a view to get the connection info for the topic.
    rosbag::View view(data_interface::m_bag, rosbag::TopicQuery(topic));
    auto connections = view.getConnections();

    // Check if the topic exists in the bag.
    if(connections.empty())
    {
        return false;
    }

    // Build a temporary introspector to get the definition tree for the topic.
    auto connection = connections.front();
    message_introspection::introspector introspector;
    introspector.new_message_type(connection->datatype, connection->msg_def, connection->md5sum);
    definition_tree = introspector.definition_tree();

    return true;
}
