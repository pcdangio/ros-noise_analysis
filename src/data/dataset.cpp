#include "data/dataset.h"

#include <message_introspection/introspector.h>

#include <rosbag/view.h>

using namespace data;

dataset::dataset(const std::string& name, const std::string& topic_name, const std::string& field_path, std::function<void(uint64_t)> notifier)
{
    // Store items.
    dataset::m_name = name;
    dataset::m_topic_name = topic_name;
    dataset::m_field_path = field_path;
    dataset::m_notifier = notifier;

    // Initialize members.
    dataset::m_variance = std::numeric_limits<double>::quiet_NaN();
    dataset::m_thread_running = false;
}
dataset::~dataset()
{
    if(dataset::m_thread_running)
    {
        dataset::m_thread.join();
    }
}

bool dataset::load(const std::shared_ptr<rosbag::Bag>& bag)
{
    // Check if the dataset is currently loading or calculating.
    if(dataset::m_thread_running)
    {
        return false;
    }

    // Run the load worker on the thread.
    // NOTE: The load worker will automatically start the calculation worker as well.
    dataset::m_thread_running = true;
    dataset::m_thread = boost::thread(&dataset::load_worker, this, bag);

    return true;
}
bool dataset::calculate()
{
    // Check if the dataset is currently loading or calculating.
    if(dataset::m_thread_running)
    {
        return false;
    }

    // Run the calculation worker on the thread.
    dataset::m_thread_running = true;
    dataset::m_thread = boost::thread(&dataset::calculate_worker, this);

    return true;
}
bool dataset::is_calculating() const
{
    return dataset::m_thread_running;
}

// DATA ACCESS
void dataset::lock_data() const
{
    dataset::m_mutex.lock();
}
void dataset::unlock_data() const
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

double dataset::variance() const
{
    std::lock_guard<std::mutex> scoped_lock(dataset::m_mutex);
    return dataset::m_variance;
}

uint32_t dataset::fit_bases() const
{
    std::lock_guard<std::mutex> scoped_lock(dataset::m_mutex);
    return dataset::m_fit_bases;
}
void dataset::fit_bases(uint32_t value)
{
    // Do not use mutex here to allow adjusting parameter while calculation thread running.
    dataset::m_fit_bases = value;
}
double dataset::fit_smoothing() const
{
    std::lock_guard<std::mutex> scoped_lock(dataset::m_mutex);
    return dataset::m_fit_smoothing;
}
void dataset::fit_smoothing(double value)
{
    // Do not use mutex here to allow adjusting parameter while calculation thread running.
    dataset::m_fit_smoothing = value;
}

// THREADING
void dataset::load_worker(std::shared_ptr<rosbag::Bag> bag)
{
    // Lock the dataset mutex.
    dataset::m_mutex.lock();

    // Clear existing data.
    dataset::m_data_time.clear();
    dataset::m_data_raw.clear();

    // Create an introspector for reading the data.
    message_introspection::introspector introspector;

    // Use a view to get the topic data.
    rosbag::View view(*bag, rosbag::TopicQuery(dataset::m_topic_name));

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

    // Unlock the dataset mutex.
    dataset::m_mutex.unlock();

    // Run the calculation worker to initialize the calculations for the newly loaded data.
    calculate_worker();
    // NOTE: calculate worker will indicate thread is finished.
}
void dataset::calculate_worker()
{
    // Lock dataset mutex.
    dataset::m_mutex.lock();

    // Unlock dataset mutex.
    dataset::m_mutex.unlock();

    // Raise notifier and pass memory address of this instance.
    dataset::m_notifier(reinterpret_cast<uint64_t>(this));

    // Indicate that thread is no longer running.
    dataset::m_thread_running = false;
}
