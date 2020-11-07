#ifndef FORM_MAIN_H
#define FORM_MAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class form_main; }
QT_END_NAMESPACE

#include "data_interface.h"

#include <QTimer>

#include <ros/ros.h>

class form_main : public QMainWindow
{
    Q_OBJECT

public:
    form_main(QWidget *parent = nullptr);
    ~form_main();

private slots:
    void on_button_open_bag_clicked();

    void toolbar_table_add();
    void toolbar_table_remove();
    void toolbar_table_clear();
    void toolbar_table_up();
    void toolbar_table_down();
    void toolbar_table_save();
    void toolbar_table_saveas();
    void toolbar_table_open();

    void bag_loaded();

    void on_combobox_topics_currentTextChanged(const QString& text);

    void on_tree_message_currentItemChanged(QTreeWidgetItem *current, QTreeWidgetItem *previous);

private:
    Ui::form_main *ui;
    void setup_splitter();
    void setup_tree_message();
    void setup_toolbar_table();

    void update_combobox_topics();
    void update_tree_message(bool clear = false);
    void add_tree_item(const message_introspection::definition_tree_t& definition_tree, QTreeWidgetItem* item);

    // ROS
    /// \brief Stores the node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();

    // COMPONENTS
    data_interface m_data_interface;

    std::string m_candidate_topic_name;
    message_introspection::definition_tree_t m_candidate_topic_definition_tree;


};
#endif // FORM_MAIN_H
