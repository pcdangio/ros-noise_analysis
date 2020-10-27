#include <ros/ros.h>

#include <topic_tools/shape_shifter.h>

#include <boost/tokenizer.hpp>

void subscriber_shape_shifter(const topic_tools::ShapeShifter& message)
{
    std::cout << message.getDataType() << std::endl << std::endl;
    std::cout << message.getMessageDefinition() << std::endl << std::endl;

    // Remove comment lines, empty lines, and comment segments.
    std::stringstream input, output;
    input << message.getMessageDefinition();

    std::string current_line;
    while(std::getline(input, current_line))
    {
        if(!current_line.empty())
        {
            if(current_line.front() != '#')
            {
                if(current_line.front() != '=')
                {
                    // Line is not empty, a full comment line, or a full === line.
                    
                    // Remove any comments from the line.
                    auto comment_position = current_line.find_first_of('#');

                    // Output the line.
                    output << current_line.substr(0, comment_position) << std::endl;
                }
            }
        }
    }

    std::cout << output.str() << std::endl;

    // boost::tokenizer<boost::char_separator<char>> tokenizer(message.getMessageDefinition());
    // for(auto token = tokenizer.begin(); token != tokenizer.end(); ++token)
    // {
    //     std::cout << *token << std::endl;
    // }
}

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "shape_shifter_tester");

    ros::NodeHandle m_node;

    ros::Subscriber m_subscriber_shape_shifter = m_node.subscribe("some_topic", 1, subscriber_shape_shifter);

    ros::spin();

    return 0;
}