#include "gui/form_main.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "noise_analysis");

    QApplication a(argc, argv);
    form_main w;
    w.show();
    return a.exec();
}
