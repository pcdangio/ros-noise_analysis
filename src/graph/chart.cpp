#include "graph/chart.h"

using namespace graph;

chart::chart()
{
    // Create chart instance.
    chart::m_chart = new QtCharts::QChart();

    // Set up axes.
    // Axis X.
    chart::m_axis_x = new QtCharts::QValueAxis();
    chart::m_axis_x->setTitleText("time (sec)");
    chart::m_chart->addAxis(chart::m_axis_x, Qt::AlignmentFlag::AlignBottom);
    // Axis Y.
    chart::m_axis_y = new QtCharts::QValueAxis();
    chart::m_axis_y->setTitleText("value");
    chart::m_chart->addAxis(chart::m_axis_y, Qt::AlignmentFlag::AlignLeft);

    // Set up series.
    // RAW
    chart::m_series_raw = new QtCharts::QLineSeries();
    chart::m_series_raw->setName("raw");
    chart::m_series_raw->attachAxis(chart::m_axis_x);
    chart::m_series_raw->attachAxis(chart::m_axis_y);
    chart::m_chart->addSeries(chart::m_series_raw);
    // FIT
    chart::m_series_fit = new QtCharts::QLineSeries();
    chart::m_series_fit->setName("fit");
    chart::m_series_fit->attachAxis(chart::m_axis_x);
    chart::m_series_fit->attachAxis(chart::m_axis_y);
    chart::m_chart->addSeries(chart::m_series_fit);
}

QtCharts::QChart* chart::get_chart() const
{
    return chart::m_chart;
}

void chart::plot_dataset(const std::shared_ptr<data::dataset>& dataset)
{
    // Get references to data.
    auto time = dataset->data_time();
    auto raw = dataset->data_raw();
    auto fit = dataset->data_fit();

    // Populate each series.
    QVector<QPointF> points;
    points.reserve(time.size());

    // RAW
    for(uint32_t i = 0; i < time.size(); ++i)
    {
        points.push_back(QPointF(time.at(i), raw.at(i)));
    }
    chart::m_series_raw->replace(points);

    // FIT
    points.clear();
    for(uint32_t i = 0; i < time.size(); ++i)
    {
        points.push_back(QPointF(time.at(i), fit.at(i)));
    }
    chart::m_series_fit->replace(points);
}
void chart::clear()
{
    chart::m_series_raw->clear();
    chart::m_series_fit->clear();
}
