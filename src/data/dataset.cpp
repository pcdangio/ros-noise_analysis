#include "data/dataset.h"

#include <message_introspection/introspector.h>

#include <rosbag/view.h>

#include <libalglib/interpolation.h>

using namespace data;

// CONSTRUCTORS
dataset::dataset(const std::shared_ptr<rosbag::Bag>& bag, const std::string& name, const std::string& topic_name, const std::string& field_path, std::function<void(uint64_t)> notifier)
{
    // Store items.
    dataset::m_bag = bag;
    dataset::m_name = name;
    dataset::m_topic_name = topic_name;
    dataset::m_field_path = field_path;
    dataset::m_notifier = notifier;

    // Initialize empty data vectors.
    dataset::m_data_time = std::make_shared<std::vector<double>>();
    dataset::m_data_raw = std::make_shared<std::vector<double>>();
    dataset::m_data_fit = std::make_shared<std::vector<double>>();

    // Initialize members.
    dataset::m_variance = std::numeric_limits<double>::quiet_NaN();
    dataset::m_fit_bases = 10;
    dataset::m_fit_smoothing = 1.0;
    dataset::m_thread_running = false;
    dataset::m_thread_stop = false;
}
dataset::~dataset()
{
    // Halt any running thread.
    dataset::cancel();
}

// ACTIONS
bool dataset::load()
{
    // Check if the dataset is currently loading or calculating.
    if(dataset::m_thread_running)
    {
        return false;
    }

    // Run the load worker on the thread.
    // NOTE: The load worker will automatically start the calculation worker as well.
    dataset::m_thread_running = true;
    dataset::m_thread_stop = false;
    dataset::m_thread = boost::thread(&dataset::load_worker, this);

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
    dataset::m_thread_stop = false;
    dataset::m_thread = boost::thread(&dataset::calculate_worker, this);

    return true;
}
bool dataset::is_calculating() const
{
    return dataset::m_thread_running;
}
void dataset::cancel()
{
    if(dataset::m_thread_running)
    {
        // Signal stop to thread.
        dataset::m_thread_stop = true;
        // Wait for thread to join.
        dataset::m_thread.join();
    }
}

// DATA ACCESS
std::shared_ptr<const std::vector<double>> dataset::data_time() const
{
    std::lock_guard<std::mutex> scoped_lock(dataset::m_mutex);
    return std::const_pointer_cast<const std::vector<double>>(dataset::m_data_time);
}
std::shared_ptr<const std::vector<double>> dataset::data_raw() const
{
    std::lock_guard<std::mutex> scoped_lock(dataset::m_mutex);
    return std::const_pointer_cast<const std::vector<double>>(dataset::m_data_raw);
}
std::shared_ptr<const std::vector<double>> dataset::data_fit() const
{
    std::lock_guard<std::mutex> scoped_lock(dataset::m_mutex);
    return std::const_pointer_cast<const std::vector<double>>(dataset::m_data_fit);
}

// PROPERTIES
void dataset::name(const std::string& value)
{
    dataset::m_name = value;
}
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
    return dataset::m_fit_bases;
}
void dataset::fit_bases(uint32_t value)
{
    // Store value between 4 and 100.
    // 4 is minimum allowable for alglib, and 100 is max for computation.
    dataset::m_fit_bases = std::max(4U, std::min(100U, value));
}
double dataset::fit_smoothing() const
{
    return dataset::m_fit_smoothing;
}
void dataset::fit_smoothing(double value)
{
    dataset::m_fit_smoothing = value;
}

// THREADING
void dataset::load_worker()
{
    // Create an introspector for reading the data.
    message_introspection::introspector introspector;

    // Use a view to get the topic data.
    rosbag::View view(*dataset::m_bag, rosbag::TopicQuery(dataset::m_topic_name));

    // Create new vector shared pointers for storing loaded data in.
    auto data_time = std::make_shared<std::vector<double>>();
    auto data_raw = std::make_shared<std::vector<double>>();

    // Reserve space in vectors.
    data_time->reserve(view.size());
    data_raw->reserve(view.size());

    // Populate the raw data.
    for(auto instance = view.begin(); instance != view.end(); ++instance)
    {
        // Check if thread stop requested.
        if(dataset::m_thread_stop)
        {
            break;
        }

        // Use introspector to read the message.
        introspector.new_message(*instance);
        // Get the field as a double.
        double value;
        if(introspector.get_number(dataset::m_field_path, value))
        {
            // Add it to the raw data buffer.
            data_time->push_back(instance->getTime().toSec());
            data_raw->push_back(value);
        }
    }

    // Replace member data vectors with thread protection.
    dataset::m_mutex.lock();
    dataset::m_data_time = data_time;
    dataset::m_data_raw = data_raw;
    dataset::m_mutex.unlock();

    // Run the calculation worker to initialize the calculations for the newly loaded data.
    calculate_worker();
    // NOTE: calculate worker will indicate thread is finished.
}
void dataset::calculate_worker()
{
    // Use alglib to create spline fit for data.

    // Repeat as long as the working fit parameters do not match the member parameters.
    // When they are mismatched it needs to retrigger a calculation because the
    // member is being changed externally while the calculatoin thread runs.
    uint32_t bases = 0;
    double smoothing = std::numeric_limits<double>::quiet_NaN();
    while(bases != dataset::m_fit_bases || smoothing != dataset::m_fit_smoothing)
    {
        // Grab working copy of fit parameters.
        bases = dataset::m_fit_bases;
        smoothing = dataset::m_fit_smoothing;

        // Set up alglib structures.
        // Set up X and Y arrays.
        alglib::real_1d_array x, y;
        // Set up spline interpolant and fit report structures to capture output.
        alglib::spline1dinterpolant interpolant;
        alglib::spline1dfitreport fit_report;
        alglib::ae_int_t result = 0;

        // Try to fit spline.
        try
        {
            // Store content from internal vectors to avoid copies.
            x.setcontent(dataset::m_data_time->size(), dataset::m_data_time->data());
            y.setcontent(dataset::m_data_raw->size(), dataset::m_data_raw->data());

            // Perform fitting.
            alglib::spline1dfitpenalized(x, y, x.length(), bases, smoothing, result, interpolant, fit_report);

            if(result <= 0)
            {
                ROS_ERROR_STREAM("alglib failed to generate spline fit (" << result << ")");
            }
        }
        catch(const alglib::ap_error& e)
        {
            ROS_ERROR_STREAM("alglib failed to generate spline fit (" << e.msg << ")");
        }

        // Check result.
        if(result > 0)
        {
            // Populate fit vector using interpolant.
            std::shared_ptr<std::vector<double>> data_fit = std::make_shared<std::vector<double>>();
            data_fit->reserve(dataset::m_data_time->size());
            for(auto time = dataset::m_data_time->cbegin(); time != dataset::m_data_time->cend(); ++time)
            {
                data_fit->push_back(alglib::spline1dcalc(interpolant, *time));
            }

            // Calculate variance.
            // Assume noise is zero mean.
            double variance = 0;
            for(uint32_t i = 0; i < dataset::m_data_time->size(); ++i)
            {
                variance += std::pow(data_fit->at(i) - dataset::m_data_raw->at(i), 2.0);
            }
            variance /= static_cast<double>(dataset::m_data_time->size());

            // Store fit vector and variance in member variables with thread protection.
            dataset::m_mutex.lock();
            dataset::m_data_fit = data_fit;
            dataset::m_variance = variance;
            dataset::m_mutex.unlock();

            // Raise notifier and pass memory address of this instance.
            dataset::m_notifier(reinterpret_cast<uint64_t>(this));
        }
    }

    // Indicate that thread is no longer running.
    dataset::m_thread_running = false;
}
