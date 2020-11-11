#ifndef DATA___DATASET_H
#define DATA___DATASET_H

#include <rosbag/bag.h>

#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <functional>

namespace data {

class dataset
{
public:
    dataset(const std::shared_ptr<rosbag::Bag>& bag, const std::string& name, const std::string& topic_name, const std::string& field_path, std::function<void(uint64_t)> notifier);
    ~dataset();

    bool load();
    bool calculate();
    bool is_calculating() const;

    // DATA ACCESS
    void lock_data() const;
    void unlock_data() const;
    const std::vector<double>& data_time() const;
    const std::vector<double>& data_raw() const;
    const std::vector<double>& data_fit() const;

    // PROPERTIES
    std::string name() const;
    std::string topic_name() const;
    std::string field_path() const;

    double variance() const;

    uint32_t fit_bases() const;
    void fit_bases(uint32_t value);
    double fit_smoothing() const;
    void fit_smoothing(double value);

private:
    std::shared_ptr<rosbag::Bag> m_bag;

    std::string m_name;
    std::string m_topic_name;
    std::string m_field_path;

    std::function<void(uint64_t)> m_notifier;

    std::vector<double> m_data_time;
    std::vector<double> m_data_raw;
    std::vector<double> m_data_fit;

    uint32_t m_fit_bases;
    double m_fit_smoothing;

    double m_variance;

    boost::thread m_thread;
    std::atomic<bool> m_thread_running;
    mutable std::mutex m_mutex;
    void load_worker();
    void calculate_worker();
};

}

#endif // DATASET_H
