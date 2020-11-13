#include "data_interface.h"

#include <rosbag/view.h>

#include <boost/filesystem/path.hpp>

using namespace data;

data_interface::data_interface()
{
    // Create new bag instance.
    data_interface::m_bag = std::make_shared<rosbag::Bag>();

    // Initialize members.
    data_interface::m_thread_running = false;
    data_interface::m_thread_stop = false;
}
data_interface::~data_interface()
{
    // Cancel covariance calculation if it's running.
    data_interface::stop_covariance_calculation();
}

bool data_interface::load_bag(std::string bag_path)
{
    // Try to load the bag file.
    try
    {
        // Close any open bag files.
        data_interface::m_bag->close();
        // Open the new bag file.
        data_interface::m_bag->open(bag_path, rosbag::bagmode::Read);
    }
    catch(const std::exception& error)
    {
        ROS_ERROR_STREAM("failed to load bag file (" << error.what() << ")");
        return false;
    }

    // Clear any existing datasets.
    data_interface::m_datasets.clear();

    return true;
}
std::string data_interface::bag_name() const
{
    boost::filesystem::path bag_path(data_interface::m_bag->getFileName());
    return bag_path.filename().string();
}
std::set<std::string> data_interface::bag_topics() const
{
    // Create set for tracking unique topic names.
    std::set<std::string> unique_topics;

    if(data_interface::m_bag->isOpen())
    {
        // Use a view to get the connections.
        rosbag::View view(*data_interface::m_bag);
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
    if(!data_interface::m_bag->isOpen())
    {
        return nullptr;
    }

    // Use a view to get the connection info for the topic.
    rosbag::View view(*data_interface::m_bag, rosbag::TopicQuery(topic_name));
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

bool data_interface::add_dataset(const std::shared_ptr<candidate_field_t>& candidate_field)
{    
    // Check if the dataset already exists, and create a set of names.
    // NOTE: Can do this because there won't be a lot of datasets.
    std::unordered_set<std::string> names;
    for(uint32_t i = 0; i < data_interface::m_datasets.size(); ++i)
    {
        // Get reference to dataset.
        auto& entry = data_interface::m_datasets[i];

        // Check if candidate dataset already exists.
        if(entry->topic_name().compare(candidate_field->topic_name()) == 0 && entry->field_path().compare(candidate_field->full_path()) == 0)
        {
            // Dataset already exists, return false.
            return false;
        }

        // Add dataset's name to unique list.
        names.insert(entry->name());
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
    auto dataset = std::make_shared<data::dataset>(data_interface::m_bag,
                                                   dataset_name,
                                                   candidate_field->topic_name(),
                                                   candidate_field->full_path(),
                                                   std::bind(&data_interface::dataset_notifier, this, std::placeholders::_1));

    // Add the dataset to the end of the deque.
    data_interface::m_datasets.push_back(dataset);

    // Begin load operation.
    dataset->load();

    return true;
}
bool data_interface::remove_dataset(uint32_t index)
{
    // Check if specified index exists.
    if(index >= data_interface::m_datasets.size())
    {
        return false;
    }

    // Cancel any operations the dataset is running.
    data_interface::m_datasets[index]->cancel();

    // Remove the dataset at the sepcified index.
    data_interface::m_datasets.erase(data_interface::m_datasets.begin() + index);

    return true;
}
bool data_interface::move_dataset(uint32_t old_index, uint32_t new_index)
{
    // Validate old and new indices.
    if(old_index >= data_interface::m_datasets.size() || new_index >= data_interface::m_datasets.size())
    {
        return false;
    }

    // Grab copy of dataset pointer at old index.
    auto move_dataset = data_interface::m_datasets[old_index];

    // Erase from old index.
    data_interface::m_datasets.erase(data_interface::m_datasets.begin() + old_index);

    // Insert at new index.
    data_interface::m_datasets.insert(data_interface::m_datasets.begin() + new_index, move_dataset);

    return true;
}
void data_interface::clear_datasets()
{
    // Iterate through all datasets to cancel and running operations.
    for(auto entry = data_interface::m_datasets.begin(); entry != data_interface::m_datasets.end(); ++entry)
    {
        (*entry)->cancel();
    }

    // Clear all datasets.
    data_interface::m_datasets.clear();
}

uint32_t data_interface::n_datasets() const
{
    return data_interface::m_datasets.size();
}
std::shared_ptr<dataset> data_interface::get_dataset(uint32_t index) const
{
    return data_interface::m_datasets[index];
}

void data_interface::dataset_notifier(uint64_t address)
{
    // Find the index of the sending dataset's address
    for(uint32_t i = 0; i < data_interface::m_datasets.size(); ++i)
    {
        if(reinterpret_cast<uint64_t>(data_interface::m_datasets[i].get()) == address)
        {
            // Emit calculation complete signal.
            emit data_interface::dataset_calculated(i);
            // Quit search.
            break;
        }
    }
}


bool data_interface::start_covariance_calculation()
{
    if(data_interface::m_thread_running)
    {
        return false;
    }

    // Start thread.
    data_interface::m_thread_running = true;
    data_interface::m_thread_stop = false;
    data_interface::m_thread = boost::thread(&data_interface::covariance_matrix_worker, this);

    return true;
}
void data_interface::stop_covariance_calculation()
{
    if(data_interface::m_thread_running)
    {
        data_interface::m_thread_stop = true;
        data_interface::m_thread.join();
    }
}
void data_interface::covariance_matrix_worker()
{
    // ASSUME FORM PREVENTS CHANGES TO DATASET WHILE CALCULATION RUNNING

    // Create covariance matrix and fill it with zeros so it can be indexed.
    auto matrix = std::make_shared<std::vector<std::vector<double>>>();
    for(uint32_t j = 0; j < data_interface::m_datasets.size(); ++j)
    {
        matrix->push_back(std::vector<double>(data_interface::m_datasets.size(), 0.0));
    }

    // Iterate through all unique combinations of datasets.
    for(uint32_t i = 0; i < data_interface::m_datasets.size(); ++i)
    {
        for(uint32_t j = i; j < data_interface::m_datasets.size(); ++j)
        {
            // Check if stop required.
            if(data_interface::m_thread_stop)
            {
                // Use goto to exit both loops.
                goto iteration_complete;
            }

            // Check if on diagonal.
            if(i == j)
            {
                // Set to dataset's covariance.
                (*matrix)[i][j] = data_interface::m_datasets[i]->variance();
            }
            else
            {
                // Calculate the covariance for the two datasets.
                // Determine which dataset has the slowest rate.
                std::shared_ptr<const std::vector<double>> slow_time, slow_raw, slow_fit, fast_time, fast_raw, fast_fit;
                if(data_interface::m_datasets[i]->size() > data_interface::m_datasets[j]->size())
                {
                    // Get slow vectors.
                    slow_time = data_interface::m_datasets[j]->data_time();
                    slow_raw = data_interface::m_datasets[j]->data_raw();
                    slow_fit = data_interface::m_datasets[j]->data_fit();
                    // Get fast vectors.
                    fast_time = data_interface::m_datasets[i]->data_time();
                    fast_raw = data_interface::m_datasets[i]->data_raw();
                    fast_fit = data_interface::m_datasets[i]->data_fit();
                }
                else
                {
                    // Get slow vectors.
                    slow_time = data_interface::m_datasets[i]->data_time();
                    slow_raw = data_interface::m_datasets[i]->data_raw();
                    slow_fit = data_interface::m_datasets[i]->data_fit();
                    // Get fast vectors.
                    fast_time = data_interface::m_datasets[j]->data_time();
                    fast_raw = data_interface::m_datasets[j]->data_raw();
                    fast_fit = data_interface::m_datasets[j]->data_fit();
                }
                // Iterate over slow dataset.
                // Initialize covariance sum.
                double covariance = 0;
                // Initialize fast tracker.
                uint32_t f = 0;
                double fast_point = fast_raw->front() - fast_fit->front();
                for(uint32_t s = 0; s < slow_time->size(); ++s)
                {
                    // Get the time of the current slow point.
                    double current_time = slow_time->at(s);
                    // Calculate the point for the slow dataset.
                    double slow_point = slow_raw->at(s) - slow_fit->at(s);
                    // Iterate forward through fast dataset to find the last point before current time.
                    for(; f < fast_time->size(); ++f)
                    {
                        // Check if fast time is greater.
                        if(fast_time->at(f) > current_time)
                        {
                            // Calculate covariance sum using the last set fast_point (at fast time <= current_time)
                            covariance += slow_point * fast_point;
                        }
                        // Calculate fast_point for this time point in fast.
                        fast_point = fast_raw->at(i) - fast_fit->at(i);
                    }
                }
                // Finish covariance calculation.
                covariance /= static_cast<double>(slow_time->size());
                // Store covariance in both triangles of the matrix.
                (*matrix)[i][j] = covariance;
                (*matrix)[j][i] = covariance;
            }
        }
    }

    // Print
    for(auto i = matrix->begin(); i != matrix->end(); ++i)
    {
        for(auto j = i->begin(); j != i->end(); ++j)
        {
            std::cout << *j << "\t";
        }
        std::cout << std::endl;
    }

    iteration_complete:

    // Check if signal needs to be emitted to pass out results.
    if(!data_interface::m_thread_stop)
    {
        emit data_interface::covariance_matrix_calculated(matrix);
    }

    // Mark thread as complete.
    data_interface::m_thread_running = false;
}
