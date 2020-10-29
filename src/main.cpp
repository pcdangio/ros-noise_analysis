#include <ros/ros.h>

#include <topic_tools/shape_shifter.h>
#include <message_introspection/message.h>

void subscriber_shape_shifter(const topic_tools::ShapeShifter& message)
{
    std::cout << message.getDataType() << std::endl << std::endl;
    std::cout << message.getMessageDefinition() << std::endl << std::endl;

    message_introspection::message msg(message);
}

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "shape_shifter_tester");

    ros::NodeHandle m_node;

    ros::Subscriber m_subscriber_shape_shifter = m_node.subscribe("some_topic", 1, subscriber_shape_shifter);

    ros::spin();

    return 0;
}