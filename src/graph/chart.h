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

    void set_noise_range(uint32_t standard_deviations);

    void plot_dataset(const std::shared_ptr<data::dataset>& dataset);

    void zoom_reset();

private:
    QtCharts::QChart* m_chart;
    QtCharts::QValueAxis* m_axis_x;
    QtCharts::QValueAxis* m_axis_y;

    QtCharts::QLineSeries* m_series_raw;
    QtCharts::QLineSeries* m_series_fit;
    QtCharts::QLineSeries* m_series_noise_p;
    QtCharts::QLineSeries* m_series_noise_m;

    uint32_t m_standard_deviations;
    double m_dataset_variance;
    void plot_noise_range();
};

}

#endif // GRAPH_H
