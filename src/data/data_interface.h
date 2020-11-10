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

namespace data {

class data_interface
    : public QObject
{
    Q_OBJECT
public:
    data_interface();

    bool load_bag(std::string bag_path);
    std::string bag_name() const;
    std::set<std::string> bag_topics() const;

    std::shared_ptr<candidate_topic_t> get_candidate_topic(const std::string& topic_name) const;



    uint32_t n_datasets() const;
    void add_dataset(const std::shared_ptr<candidate_field_t>& candidate_field);
    void select_dataset(uint32_t index);
    std::shared_ptr<dataset> selected_dataset() const;


signals:
    void bag_loaded();
    void dataset_added();
    void dataset_calculated();

private:
    rosbag::Bag m_bag;

    std::deque<std::shared_ptr<dataset>> m_datasets;
    std::shared_ptr<dataset> m_selected_dataset;
};

}

#endif
