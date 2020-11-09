#ifndef GRAPH___CHART_H
#define GRAPH___CHART_H

#include "data/dataset.h"

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLineSeries>

namespace graph {

class chart
{
public:
    chart();

    QtCharts::QChart* get_chart() const;

    void plot_dataset(const std::shared_ptr<data::dataset>& dataset);
    void clear();

private:
    QtCharts::QChart* m_chart;
    QtCharts::QValueAxis* m_axis_x;
    QtCharts::QValueAxis* m_axis_y;

    QtCharts::QLineSeries* m_series_raw;
    QtCharts::QLineSeries* m_series_fit;
};

}

#endif // GRAPH_H
