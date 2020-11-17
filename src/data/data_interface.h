/// \file data/data_interface.h
/// \brief Defines the data::data_interface class.
#ifndef DATA___DATA_INTERFACE_H
#define DATA___DATA_INTERFACE_H

#include "data/candidate_topic.h"
#include "data/candidate_field.h"
#include "data/dataset.h"

#include <message_introspection/introspector.h>

#include <rosbag/bag.h>

#include <QObject>

#include <string>
#include <deque>
#include <unordered_set>

/// \brief Includes all components related to data IO.
namespace data {

/// \brief An interface for interacting with data loaded from ROS Bag files.
class data_interface
    : public QObject
{
    Q_OBJECT
public:
    // CONSTRUCTORS
    /// \brief Creates a new data_interface instance.
    data_interface();

    // BAG METHODS
    /// \brief Loads a bag file from disk.
    /// \param bag_path The filepath of the bag file to load.
    /// \returns TRUE if the bag file was loaded, otherwise FALSE.
    bool load_bag(std::string bag_path);
    /// \brief Gets the name of the laoded bag file.
    /// \returns The name of the loaded bag file.
    std::string bag_name() const;
    /// \brief Gets the list of topics included in the laoded bag file.
    /// \returns A sorted set of bag topics by name.
    std::set<std::string> bag_topics() const;

    // CANDIDATE METHODS
    /// \brief Gets information for a candidate topic to choose a dataset from.
    /// \param topic_name The name of the candidate topic.
    /// \returns Information about the candidate topic. May be nullptr if the topic does not exist.
    std::shared_ptr<candidate_topic_t> get_candidate_topic(const std::string& topic_name) const;

    // DATASET MANAGEMENT
    /// \brief Adds a new dataset to the analysis.
    /// \param candidate_field The candidate field to add as a dataset.
    /// \returns TRUE if the dataset was added, otherwise FALSE.
    bool add_dataset(const std::shared_ptr<candidate_field_t>& candidate_field);
    /// \brief Removes a dataset from the analysis.
    /// \param index The index of the dataset to remove.
    /// \returns TRUE if the indicated dataset was removed, otherwise FALSE.
    bool remove_dataset(uint32_t index);
    /// \brief Moves an existing dataset to a new position in the analysis.
    /// \param old_index The original index of the dataset to move.
    /// \param new_index The destination index of the dataset to move.
    /// \returns TRUE if the dataset was moved, otherwise FALSE.
    bool move_dataset(uint32_t old_index, uint32_t new_index);
    /// \brief Clears all datasets from the analysis.
    void clear_datasets();

    // DATASET ACCESS
    /// \brief Gets the number of datasets in the analysis.
    /// \returns The number of datasets.
    uint32_t n_datasets() const;
    /// \brief Gets a specified dataset from the anaylsis.
    /// \param index The index of the dataset in the analysis.
    /// \returns The specified dataset. May return nullptr if the index is invalid.
    std::shared_ptr<dataset> get_dataset(uint32_t index) const;

signals:
    /// \brief Indicates that a dataset has finished it's calculation thread.
    /// \param index The index of the dataset that completed the calculation.
    void dataset_calculated(quint32 index);

private:
    // VARIABLES
    /// \brief The bag file where data is loaded from.
    std::shared_ptr<rosbag::Bag> m_bag;
    /// \brief The collection of datasets in the analysis.
    std::deque<std::shared_ptr<dataset>> m_datasets;

    // CALLBACKS
    /// \brief A callback datasets use to notify that their calculation thread is complete.
    /// \param address The unique memory address of the dataset that completed a calculation.
    void dataset_notifier(uint64_t address);
};

}

#endif
