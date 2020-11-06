#ifndef FORM_MAIN_H
#define FORM_MAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class form_main; }
QT_END_NAMESPACE

#include "data_set.h"

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

private:
    Ui::form_main *ui;
    void setup_toolbar_table();

    // ROS
    /// \brief Stores the node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();

    // COMPONENTS
    std::shared_ptr<data_set> m_data_set;
};
#endif // FORM_MAIN_H
