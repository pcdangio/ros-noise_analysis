#ifndef DATA_SET_H
#define DATA_SET_H

#include <rosbag/bag.h>

#include <QObject>

#include <string>
#include <vector>

class data_set
    : public QObject
{
    Q_OBJECT
public:
    data_set();

    bool load_bag(std::string bag_path);
    std::string bag_name() const;
    std::vector<std::string> bag_topics() const;

signals:
    void bag_loaded();

private:
    rosbag::Bag m_bag;
};

#endif
