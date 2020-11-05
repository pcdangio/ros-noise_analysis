#include "gui/form_main.h"
#include "ui_form_main.h"

// CONSTRUCTORS
form_main::form_main(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::form_main)
{
    // Set up UI.
    ui->setupUi(this);

    // Set up node handle.
    form_main::m_node = std::make_shared<ros::NodeHandle>();

    // Start ros spinner.
    connect(&(form_main::m_ros_spinner), &QTimer::timeout, this, &form_main::ros_spin);
    form_main::m_ros_spinner.start(10);
}

form_main::~form_main()
{
    delete ui;
}

// ROS
void form_main::ros_spin()
{
    // Handle callbacks.
    ros::spinOnce();

    // Quit if ROS shutting down.
    if(!ros::ok())
    {
        QApplication::quit();
    }
}

