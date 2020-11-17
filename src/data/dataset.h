/// \file data/dataset.h
/// \brief Defines the data::dataset class.
#ifndef DATA___DATASET_H
#define DATA___DATASET_H

#include <rosbag/bag.h>

#include <boost/thread.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <functional>

namespace data {

/// \brief A signal dataset for analysis.
class dataset
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new dataset instance.
    /// \param bag The bag file that the dataset will load data from.
    /// \param name The name of the dataset.
    /// \param topic_name The name of the topic that the dataset will load data from.
    /// \param field_path The path of the field that the dataset will load data from.
    /// \param notifier The callback for notifying completed calculations.
    dataset(const std::shared_ptr<rosbag::Bag>& bag, const std::string& name, const std::string& topic_name, const std::string& field_path, std::function<void(uint64_t)> notifier);
    ~dataset();

    // ACTIONS
    /// \brief Starts loading data from the bag file on a dedicated thread.
    /// \returns TRUE if the load operation started, otherwise FALSE.
    bool load();
    /// \brief Starts calculating noise characteristics on a dedicated thread.
    /// \returns TRUE if the calculation started, otherwise FALSE.
    bool calculate();
    /// \brief Indicates if the dataset is currently loading/calculating data.
    /// \returns TRUE if the dataset is loading/calculating data, otherwise FALSE.
    bool is_calculating() const;
    /// \brief Cancels any currently running load/calculation operations.
    void cancel();

    // DATA ACCESS
    /// \brief Gets the time vector of the loaded data.
    /// \returns The loaded time vector.
    std::shared_ptr<const std::vector<double>> data_time() const;
    /// \brief Gets the raw data vector of the loaded data.
    /// \returns The loaded raw data vector.
    std::shared_ptr<const std::vector<double>> data_raw() const;
    /// \brief Gets the fitted data vector of the loaded data.
    /// \returns The calculated fit data vector.
    std::shared_ptr<const std::vector<double>> data_fit() const;

    // PROPERTIES
    /// \brief Sets the name of the dataset.
    /// \param value The new name to set.
    void name(const std::string& value);
    /// \brief Gets the name of the dataset.
    /// \returns The name of the dataset.
    std::string name() const;
    /// \brief Gets the name of the topic the dataset was loaded from.
    /// \returns The name of the topic.
    std::string topic_name() const;
    /// \brief Gets the field path that the dataset was loaded from.
    /// \returns The field path.
    std::string field_path() const;

    /// \brief Gets the calculated variance of the dataset.
    /// \returns The calculated variance. Will return NaN if not yet calculated.
    double variance() const;

    /// \brief Gets the number of bases used in the fit operation.
    /// \returns The number of bases.
    uint32_t fit_bases() const;
    /// \brief Sets the number of bases used in the fit operation.
    /// \param value The new number of bases.
    void fit_bases(uint32_t value);
    /// \brief Gets the smoothing ratio of the fit operation.
    /// \returns The smoothing ratio.
    double fit_smoothing() const;
    /// \brief Sets the smoothing ratio of the fit operation.
    /// \param value The new smoothing ratio.
    void fit_smoothing(double value);

private:
    // VARIABLES
    /// \brief The bag file that the dataset loads data from.
    std::shared_ptr<rosbag::Bag> m_bag;
    /// \brief The name of the dataset.
    std::string m_name;
    /// \brief The name of the topic the dataset is loaded from.
    std::string m_topic_name;
    /// \brief The field path that the dataset is loaded from.
    std::string m_field_path;

    // CALLBACKS
    /// \brief The notifier callback to use when calculations complete.
    std::function<void(uint64_t)> m_notifier;

    // DATA
    /// \brief Stores the time vector of loaded data.
    std::shared_ptr<std::vector<double>> m_data_time;
    /// \brief Stores the raw data vector of the laoded data.
    std::shared_ptr<std::vector<double>> m_data_raw;
    /// \brief Stores the fit data vector that was calculated.
    std::shared_ptr<std::vector<double>> m_data_fit;

    // FIT
    /// \brief The number of bases used in the fit operation.
    uint32_t m_fit_bases;
    /// \brief The smoothing ratio used in the fit operation.
    double m_fit_smoothing;

    // VARIANCE
    /// \brief Stores the calculated variance of the dataset.
    double m_variance;

    // THREADING
    /// \brief The dedicated thread for load/calculation operations.
    boost::thread m_thread;
    /// \brief Indicates if the thread is running.
    std::atomic<bool> m_thread_running;
    /// \brief Notifies the thread to stop.
    std::atomic<bool> m_thread_stop;
    /// \brief Protects thread r/w operations.
    mutable std::mutex m_mutex;
    /// \brief The worker method for loading data from the bag file.
    void load_worker();
    /// \brief The worker method for calculating noise variance.
    void calculate_worker();
};

}

#endif // DATASET_H
