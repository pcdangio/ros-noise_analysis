/// \file gui/form_main.h
/// \brief Defines the form_main class.
#ifndef FORM_MAIN_H
#define FORM_MAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class form_main; }
QT_END_NAMESPACE

#include "data/data_interface.h"
#include "graph/chart.h"

#include <QTimer>
#include <QTreeWidgetItem>

#include <ros/ros.h>

/// \brief The main form of the application.
class form_main : public QMainWindow
{
    Q_OBJECT

public:
    // CONSTRUCTORS
    /// \brief Creates a new form_main instance.
    /// \param parent The parent widget of the form.
    form_main(QWidget *parent = nullptr);
    ~form_main();

private slots:
    // SLOTS - FORM
    /// \brief Handles button_open_bag::clicked signals.
    void on_button_open_bag_clicked();
    /// \brief Handles combobox_topics::currentTextChanged signals.
    /// \param text The new text that the combobox changed to.
    void on_combobox_topics_currentTextChanged(const QString& text);
    /// \brief Handles table_datasets::cellClicked signals.
    /// \param row The row of the cell that was clicked.
    /// \param column The column of the cell that was clicked.
    void on_table_datasets_cellClicked(int row, int column);
    /// \brief Handles table_datasets::cellChanged signals.
    /// \param row The row of the cell that was changed.
    /// \param column The column of the cell that was changed.
    void on_table_datasets_cellChanged(int row, int column);
    /// \brief Handles slider_bases::valueChanged signals.
    /// \param value The new value of the slider.
    void on_slider_bases_valueChanged(int value);
    /// \brief Handles slider_smoothness::valueChanged signals.
    /// \param value The new value of the slider.
    void on_slider_smoothness_valueChanged(int value);

    // SLOTS - TOOLBAR TABLE
    /// \brief Handles toolbar_table::add action signals.
    void toolbar_table_add();
    /// \brief Handles toolbar_table::remove action signals.
    void toolbar_table_remove();
    /// \brief Handles toolbar_table::clear action signals.
    void toolbar_table_clear();
    /// \brief Handles toolbar_table::up action signals.
    void toolbar_table_up();
    /// \brief Handles toolbar_table::down action signals.
    void toolbar_table_down();

    // SLOTS - TOOLBAR CHART
    /// \brief Handles toolbar_chart::zoom_all action signals.
    void toolbar_zoom_all();
    /// \brief Handles toolbar_chart::zoom_rectangle action signals.
    void toolbar_zoom_rectangle();
    /// \brief Handles toolbar_chart::zoom_horizontal action signals.
    void toolbar_zoom_horizontal();
    /// \brief Handles toolbar_charts::zoom_vertical action signals.
    void toolbar_zoom_vertical();

    // SLOTS - COMPONENTS
    /// \brief Handles data_interface::dataset_calculated signals.
    /// \param index The index of the dataset that completed a calculation.
    void dataset_calculated(quint32 index);

private:
    /// \brief The form's UI instance.
    Ui::form_main *ui;

    // ROS
    /// \brief Stores the node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();

    // COMPONENTS
    /// \brief The application's data_interface instance.
    data::data_interface m_data_interface;
    /// \brief The application's chart instance.
    graph::chart m_chart;

    // CANDIDATES
    /// \brief The current candidate topic instance.
    std::shared_ptr<data::candidate_topic_t> m_candidate_topic;

    // UI - SETUP
    /// \brief Sets up the splitter_main (vertical) splitter instance.
    void setup_splitter();
    /// \brief Sets up the candidate topic tree viewer.
    void setup_tree_message();
    /// \brief Sets up the dataset table's toolbar.
    void setup_toolbar_table();
    /// \brief Sets up the dataset table.
    void setup_table_datasets();
    /// \brief Sets up the plotting area.
    void setup_chartview();
    /// \brief Sets up the chart's toolbar.
    void setup_toolbar_chart();

    // UI - UPDATE
    /// \brief Updates the topic combobox.
    void update_combobox_topics();
    /// \brief Updates the candidate topic tree view.
    void update_tree_message();
    /// \brief A recursive method for populating the topic tree view.
    /// \param definition_tree The current definition tree to populate into the tree.
    /// \param item The current item level to populate into the tree.
    void add_tree_item(const message_introspection::definition_tree_t& definition_tree, QTreeWidgetItem* item);
    /// \brief Updates the dataset table.
    void update_table_datasets();
    /// \brief Updates the plotting area with a new dataset.
    /// \param dataset The dataset to show in the plotting area.
    void update_plot_view(std::shared_ptr<data::dataset> dataset);

    // METHODS
    /// \brief Gets a specified dataset from the data_interface.
    /// \param index The index of the dataset to get.
    /// \returns TRUE if the dataset was retrieved, otherwise FALSE.
    bool get_selected_dataset(uint32_t& index);
};

#endif
