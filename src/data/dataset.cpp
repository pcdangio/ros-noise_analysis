#include "data/dataset.h"

#include <message_introspection/introspector.h>

#include <rosbag/view.h>

using namespace data;

dataset::dataset(const std::string& name, const std::string& topic_name, const std::string& field_path)
{
    // Store items.
    dataset::m_name = name;
    dataset::m_topic_name = topic_name;
    dataset::m_field_path = field_path;

    // Initialize members.
    dataset::m_variance = std::numeric_limits<double>::quiet_NaN();
}

void dataset::load(const rosbag::Bag& bag)
{
    // Load the raw data from the bag.

    // Clear existing data.
    dataset::m_data_time.clear();
    dataset::m_data_raw.clear();

    // Create an introspector for reading the data.
    message_introspection::introspector introspector;

    // Use a view to get the topic data.
    rosbag::View view(bag, rosbag::TopicQuery(dataset::m_topic_name));

    // Populate the raw data.
    for(auto instance = view.begin(); instance != view.end(); ++instance)
    {
        // Use introspector to read the message.
        introspector.new_message(*instance);
        // Get the field as a double.
        double value;
        if(introspector.get_number(dataset::m_field_path, value))
        {
            // Add it to the raw data buffer.
            dataset::m_data_time.push_back(instance->getTime().toSec());
            dataset::m_data_raw.push_back(value);
        }
    }

    // Remove time offset.
    // Reverse through time vector.
    for(auto time = dataset::m_data_time.rbegin(); time != dataset::m_data_time.rend(); ++time)
    {
        *time -= dataset::m_data_time.front();
    }
}
bool dataset::calculate()
{

}

// DATA ACCESS
void dataset::lock_data()
{
    dataset::m_mutex.lock();
}
void dataset::unlock_data()
{
    dataset::m_mutex.unlock();
}
const std::vector<double>& dataset::data_time() const
{
    return dataset::m_data_time;
}
const std::vector<double>& dataset::data_raw() const
{
    return dataset::m_data_raw;
}
const std::vector<double>& dataset::data_fit() const
{
    return dataset::m_data_fit;
}

// PROPERTIES
std::string dataset::name() const
{
    return dataset::m_name;
}
std::string dataset::topic_name() const
{
    return dataset::m_topic_name;
}
std::string dataset::field_path() const
{
    return dataset::m_field_path;
}

double dataset::variance()
{
    dataset::m_mutex.lock();
    double output = dataset::m_variance;
    dataset::m_mutex.unlock();

    return output;
}

uint32_t dataset::fit_bases() const
{
    return dataset::m_fit_bases;
}
double dataset::fit_smoothing() const
{
    return dataset::m_fit_smoothing;
}
