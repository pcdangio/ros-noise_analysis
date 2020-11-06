#include "gui/form_main.h"
#include "ui_form_main.h"

#include "gui/form_field.h"

#include <QFileDialog>
#include <QToolBar>

// CONSTRUCTORS
form_main::form_main(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::form_main)
{
    // Set up UI.
    form_main::ui->setupUi(this);
    // Add toolbars.
    form_main::setup_toolbar_table();

    // Set up data_set.
    form_main::m_data_set = std::make_shared<data_set>();

    // Set up node handle.
    form_main::m_node = std::make_shared<ros::NodeHandle>();

    // Connect form to data_set events.
    connect(form_main::m_data_set.get(), &data_set::bag_loaded, this, &form_main::bag_loaded);

    // Start ros spinner.
    connect(&(form_main::m_ros_spinner), &QTimer::timeout, this, &form_main::ros_spin);
    form_main::m_ros_spinner.start(10);
}

form_main::~form_main()
{
    delete ui;
}

// UI
void form_main::setup_toolbar_table()
{
    // Create new toolbar for table.
    QToolBar* toolbar_table = new QToolBar(this);

    // Add actions for field control.
    auto action_add = toolbar_table->addAction(QIcon::fromTheme("list-add"), "Add field...");
    auto action_remove = toolbar_table->addAction(QIcon::fromTheme("list-remove"), "Remove field");
    auto action_clear = toolbar_table->addAction(QIcon::fromTheme("edit-clear"), "Clear all fields");

    // Add actions for field movement.
    toolbar_table->addSeparator();
    auto action_up = toolbar_table->addAction(QIcon::fromTheme("go-up"), "Move field up");
    auto action_down = toolbar_table->addAction(QIcon::fromTheme("go-down"), "Move field down");

    // Add actions for analysis saving.
    toolbar_table->addSeparator();
    auto action_save = toolbar_table->addAction(QIcon::fromTheme("document-save"), "Save analysis...");
    auto action_saveas = toolbar_table->addAction(QIcon::fromTheme("document-save-as"), "Save analysis as...");
    auto action_open = toolbar_table->addAction(QIcon::fromTheme("document-open"), "Open analysis...");

    // Add toolbar to table's layout.
    form_main::ui->layout_table->insertWidget(0, toolbar_table);

    // Make connections.
    connect(action_add, &QAction::triggered, this, &form_main::toolbar_table_add);
    connect(action_remove, &QAction::triggered, this, &form_main::toolbar_table_remove);
    connect(action_clear, &QAction::triggered, this, &form_main::toolbar_table_clear);
    connect(action_up, &QAction::triggered, this, &form_main::toolbar_table_up);
    connect(action_down, &QAction::triggered, this, &form_main::toolbar_table_down);
    connect(action_save, &QAction::triggered, this, &form_main::toolbar_table_save);
    connect(action_saveas, &QAction::triggered, this, &form_main::toolbar_table_saveas);
    connect(action_open, &QAction::triggered, this, &form_main::toolbar_table_open);
}

// SLOTS - TOOLBAR_TABLE
void form_main::toolbar_table_add()
{
    // Show form for adding field.
    form_field dialog(form_main::m_data_set);
    dialog.exec();
}
void form_main::toolbar_table_remove()
{

}
void form_main::toolbar_table_clear()
{

}
void form_main::toolbar_table_up()
{

}
void form_main::toolbar_table_down()
{

}
void form_main::toolbar_table_save()
{

}
void form_main::toolbar_table_saveas()
{

}
void form_main::toolbar_table_open()
{

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
    if(!form_main::m_data_set->load_bag(dialog.selectedFiles().front().toStdString()))
    {
        QMessageBox::warning(this, "Error", "Error loading bag file. See ROS log for more information.");
    }
}

void form_main::bag_loaded()
{
    // Update bag name line edit.
    form_main::ui->lineedit_bag->setText(QString::fromStdString(form_main::m_data_set->bag_name()));
}
