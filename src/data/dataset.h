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
    void cancel();

    // DATA ACCESS
    std::shared_ptr<const std::vector<double>> data_time() const;
    std::shared_ptr<const std::vector<double>> data_raw() const;
    std::shared_ptr<const std::vector<double>> data_fit() const;

    // PROPERTIES
    std::string name() const;
    std::string topic_name() const;
    std::string field_path() const;

    double variance() const;

    double fit_basis_ratio() const;
    void fit_basis_ratio(double value);
    double fit_smoothing() const;
    void fit_smoothing(double value);

private:
    std::shared_ptr<rosbag::Bag> m_bag;

    std::string m_name;
    std::string m_topic_name;
    std::string m_field_path;

    std::function<void(uint64_t)> m_notifier;

    std::shared_ptr<std::vector<double>> m_data_time;
    std::shared_ptr<std::vector<double>> m_data_raw;
    std::shared_ptr<std::vector<double>> m_data_fit;

    double m_fit_basis_ratio;
    double m_fit_smoothing;

    double m_variance;

    boost::thread m_thread;
    std::atomic<bool> m_thread_running;
    std::atomic<bool> m_thread_stop;
    mutable std::mutex m_mutex;
    void load_worker();
    void calculate_worker();
};

}

#endif // DATASET_H
