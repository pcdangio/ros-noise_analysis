#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#include "data/candidate_topic.h"
#include "data/candidate_field.h"
#include "data/dataset.h"

#include <message_introspection/introspector.h>

#include <rosbag/bag.h>

#include <QObject>

#include <string>
#include <deque>
#include <unordered_set>
#include <mutex>

#include <boost/thread.hpp>

namespace data {

class data_interface
    : public QObject
{
    Q_OBJECT
public:
    data_interface();
    ~data_interface();

    bool load_bag(std::string bag_path);
    std::string bag_name() const;
    std::set<std::string> bag_topics() const;

    std::shared_ptr<candidate_topic_t> get_candidate_topic(const std::string& topic_name) const;


    bool add_dataset(const std::shared_ptr<candidate_field_t>& candidate_field);
    bool remove_dataset(uint32_t index);
    bool move_dataset(uint32_t old_index, uint32_t new_index);
    void clear_datasets();

    uint32_t n_datasets() const;
    std::shared_ptr<dataset> get_dataset(uint32_t index) const;

    bool start_covariance_calculation();
    void stop_covariance_calculation();
signals:
    void dataset_calculated(quint32 index);
    void covariance_matrix_calculated(std::shared_ptr<std::vector<std::vector<double>>> matrix);

private:
    std::shared_ptr<rosbag::Bag> m_bag;

    std::deque<std::shared_ptr<dataset>> m_datasets;
    std::shared_ptr<dataset> m_selected_dataset;

    void dataset_notifier(uint64_t address);

    boost::thread m_thread;
    std::atomic<bool> m_thread_running;
    std::atomic<bool> m_thread_stop;
    void covariance_matrix_worker();
};

}

#endif
