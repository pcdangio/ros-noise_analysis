#ifndef DATA___DATASET_H
#define DATA___DATASET_H

#include <rosbag/bag.h>

#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <mutex>

namespace data {

class dataset
{
public:
    dataset(const std::string& name, const std::string& topic_name, const std::string& field_path);

    void load(const rosbag::Bag& bag);
    bool calculate();

    // DATA ACCESS
    void lock_data();
    void unlock_data();
    const std::vector<double>& data_time() const;
    const std::vector<double>& data_raw() const;
    const std::vector<double>& data_fit() const;

    // PROPERTIES
    std::string name() const;
    std::string topic_name() const;
    std::string field_path() const;

    double variance();

    uint32_t fit_bases() const;
    double fit_smoothing() const;

private:
    std::string m_name;
    std::string m_topic_name;
    std::string m_field_path;

    std::vector<double> m_data_time;
    std::vector<double> m_data_raw;
    std::vector<double> m_data_fit;

    uint32_t m_fit_bases;
    double m_fit_smoothing;

    double m_variance;

    boost::thread m_thread;
    std::mutex m_mutex;
};

}

#endif // DATASET_H
