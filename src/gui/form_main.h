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

class form_main : public QMainWindow
{
    Q_OBJECT

public:
    // CONSTRUCTORS
    form_main(QWidget *parent = nullptr);
    ~form_main();

private slots:
    // SLOTS - FORM
    void on_button_open_bag_clicked();
    void on_combobox_topics_currentTextChanged(const QString& text);
    void on_table_datasets_cellClicked(int row, int column);
    void on_table_datasets_cellChanged(int row, int column);
    void on_slider_bases_valueChanged(int value);
    void on_slider_smoothness_valueChanged(int value);

    // SLOTS - TOOLBAR TABLE
    void toolbar_table_add();
    void toolbar_table_remove();
    void toolbar_table_clear();
    void toolbar_table_up();
    void toolbar_table_down();

    // SLOTS - TOOLBAR CHART
    void toolbar_zoom_all();
    void toolbar_zoom_rectangle();
    void toolbar_zoom_horizontal();
    void toolbar_zoom_vertical();

    // SLOTS - COMPONENTS
    void dataset_calculated(quint32 index);

private:
    Ui::form_main *ui;

    // ROS
    /// \brief Stores the node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();

    // COMPONENTS
    data::data_interface m_data_interface;
    graph::chart m_chart;

    // CANDIDATES
    std::shared_ptr<data::candidate_topic_t> m_candidate_topic;

    // UI - SETUP
    void setup_splitter();
    void setup_tree_message();
    void setup_toolbar_table();
    void setup_table_datasets();
    void setup_chartview();
    void setup_toolbar_chart();

    // UI - UPDATE
    void update_combobox_topics();
    void update_tree_message();
    void add_tree_item(const message_introspection::definition_tree_t& definition_tree, QTreeWidgetItem* item);
    void update_table_datasets();
    void update_plot_view(std::shared_ptr<data::dataset> dataset);

    // METHODS
    bool get_selected_dataset(uint32_t& index);
};
#endif
