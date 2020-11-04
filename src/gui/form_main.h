#ifndef FORM_MAIN_H
#define FORM_MAIN_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class form_main; }
QT_END_NAMESPACE

#include <QTimer>

#include <ros/ros.h>

class form_main : public QMainWindow
{
    Q_OBJECT

public:
    form_main(QWidget *parent = nullptr);
    ~form_main();

private:
    Ui::form_main *ui;

    // ROS
    /// \brief Stores the node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;
    /// \brief A timer for spinning ROS.
    QTimer m_ros_spinner;
    /// \brief The worker method for spinning ROS.
    void ros_spin();
};
#endif // FORM_MAIN_H
