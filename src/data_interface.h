#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#include <rosbag/bag.h>
#include <message_introspection/introspector.h>

#include <QObject>
#include <QTreeWidgetItem>

#include <string>
#include <unordered_map>

class data_interface
    : public QObject
{
    Q_OBJECT
public:
    data_interface();

    bool load_bag(std::string bag_path);
    std::string bag_name() const;
    std::set<std::string> bag_topics() const;

    bool get_definition_tree(const std::string& topic, message_introspection::definition_tree_t& definition_tree) const;

signals:
    void bag_loaded();

private:
    rosbag::Bag m_bag;

};

#endif
