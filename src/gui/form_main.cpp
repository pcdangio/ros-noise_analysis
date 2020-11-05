#include "gui/form_main.h"
#include "ui_form_main.h"

#include <QFileDialog>

// CONSTRUCTORS
form_main::form_main(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::form_main)
{
    // Set up UI.
    form_main::ui->setupUi(this);

    // Set up node handle.
    form_main::m_node = std::make_shared<ros::NodeHandle>();

    // Connect form to data_set events.
    connect(&(form_main::m_data_set), &data_set::bag_loaded, this, &form_main::on_bag_loaded);

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


void form_main::on_button_open_bag_clicked()
{
    // Build open file dialog.
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::FileMode::ExistingFile);
    dialog.setViewMode(QFileDialog::ViewMode::Detail);
    dialog.setNameFilter("ROS Bag (*.bag)");

    // Show dialog.
    if(!dialog.exec())
    {
        return;
    }

    // Try to load bag file.
    if(!form_main::m_data_set.load_bag(dialog.selectedFiles().front().toStdString()))
    {
        QMessageBox::warning(this, "Error", "Error loading bag file. See ROS log for more information.");
    }
}

void form_main::on_bag_loaded()
{
    // Update bag name line edit.
    form_main::ui->lineedit_bag->setText(QString::fromStdString(form_main::m_data_set.bag_name()));
}