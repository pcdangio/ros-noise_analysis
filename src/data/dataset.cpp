#include "data/dataset.h"

using namespace data;

dataset::dataset(const std::string& name, const std::string& topic_name, const std::string& field_path)
{
    // Store items.
    dataset::m_name = name;
    dataset::m_topic_name = topic_name;
    dataset::m_field_path = field_path;

    // Initialize members.
    dataset::m_variance = std::numeric_limits<double>::quiet_NaN();
}

void dataset::load(const rosbag::Bag& bag)
{

}
bool dataset::calculate()
{

}

// DATA ACCESS
const std::vector<double>& dataset::data_time() const
{

}
const std::vector<double>& dataset::data_raw() const
{

}
const std::vector<double>& dataset::data_fit() const
{

}
const std::vector<double>& dataset::data_noise() const
{

}

// PROPERTIES
std::string dataset::name() const
{
    return dataset::m_name;
}
std::string dataset::topic_name() const
{
    return dataset::m_topic_name;
}
std::string dataset::field_path() const
{
    return dataset::m_field_path;
}

double dataset::variance() const
{
    return dataset::m_variance;
}

uint32_t dataset::fit_bases() const
{
    return dataset::m_fit_bases;
}
double dataset::fit_smoothing() const
{
    return dataset::m_fit_smoothing;
}
