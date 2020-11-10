#include "data_interface.h"

#include <rosbag/view.h>

#include <boost/filesystem/path.hpp>

using namespace data;

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

    // Clear any existing datasets.
    data_interface::m_datasets.clear();

    // Emit datasets modified.
    emit data_interface::datasets_modified();

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

std::shared_ptr<candidate_topic_t> data_interface::get_candidate_topic(const std::string& topic_name) const
{
    // Check if bag file is open.
    if(!data_interface::m_bag.isOpen())
    {
        return nullptr;
    }

    // Use a view to get the connection info for the topic.
    rosbag::View view(data_interface::m_bag, rosbag::TopicQuery(topic_name));
    auto connections = view.getConnections();

    // Check if the topic exists in the bag.
    if(connections.empty())
    {
        return nullptr;
    }

    // Build a temporary introspector to get the definition tree for the topic.
    auto connection = connections.front();
    message_introspection::introspector introspector;    
    introspector.new_message_type(connection->datatype, connection->msg_def, connection->md5sum);

    // Create an output candidate.
    auto output = std::make_shared<candidate_topic_t>();
    output->topic_name = topic_name;
    output->definition_tree = introspector.definition_tree();

    return output;
}

void data_interface::add_dataset(const std::shared_ptr<candidate_field_t>& candidate_field)
{    
    // Check if the dataset already exists, and create a set of names.
    // NOTE: Can do this because there won't be a lot of datasets.
    std::unordered_set<std::string> names;
    for(auto dataset = data_interface::m_datasets.cbegin(); dataset != data_interface::m_datasets.cend(); ++dataset)
    {
        // Check if dataset exists.
        if((*dataset)->topic_name().compare(candidate_field->topic_name()) == 0 && (*dataset)->field_path().compare(candidate_field->full_path()) == 0)
        {
            // Dataset already exists.
            return;
        }

        // Add dataset's name to unique list.
        names.insert((*dataset)->name());
    }

    // Find a new name for the dataset.
    uint32_t name_number = 0;
    std::string dataset_name = "";
    bool name_found = false;
    while(!name_found)
    {
        dataset_name = "var_" + std::to_string(name_number++);
        name_found = names.count(dataset_name) == 0;
    }

    // Create a new dataset.
    auto dataset = std::make_shared<data::dataset>(dataset_name, candidate_field->topic_name(), candidate_field->full_path());

    // Add the dataset to the end of the deque.
    data_interface::m_datasets.push_back(dataset);

    // Load the dataset.
    dataset->load(data_interface::m_bag);

    // Emit signal.
    emit data_interface::dataset_added();
}

uint32_t data_interface::n_datasets() const
{
    return data_interface::m_datasets.size();
}
std::shared_ptr<dataset> data_interface::get_dataset(uint32_t index) const
{
    return data_interface::m_datasets[index];
}
