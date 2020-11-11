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
    chart::m_chart->addSeries(chart::m_series_raw);
    chart::m_series_raw->attachAxis(chart::m_axis_x);
    chart::m_series_raw->attachAxis(chart::m_axis_y);

    // FIT
    chart::m_series_fit = new QtCharts::QLineSeries();
    chart::m_series_fit->setName("fit");
    chart::m_chart->addSeries(chart::m_series_fit);
    chart::m_series_fit->attachAxis(chart::m_axis_x);
    chart::m_series_fit->attachAxis(chart::m_axis_y);

    // Configure Legend
    chart::m_chart->legend()->setAlignment(Qt::AlignmentFlag::AlignRight);
}

QtCharts::QChart* chart::get_chart() const
{
    return chart::m_chart;
}

void chart::plot_dataset(const std::shared_ptr<data::dataset>& dataset)
{
    // Get pointers to the data.
    auto time = dataset->data_time();
    auto raw = dataset->data_raw();
    auto fit = dataset->data_fit();

    // Get initial time point.
    double t0 = 0;
    if(!time->empty())
    {
        t0 = time->front();
    }

    // Populate each series.
    QVector<QPointF> points;
    points.reserve(time->size());

    // RAW
    for(uint32_t i = 0; i < raw->size(); ++i)
    {
        points.push_back(QPointF(time->at(i) - t0, raw->at(i)));
    }
    chart::m_series_raw->replace(points);

    // FIT
    points.clear();
    for(uint32_t i = 0; i < fit->size(); ++i)
    {
        points.push_back(QPointF(time->at(i) - t0, fit->at(i)));
    }
    chart::m_series_fit->replace(points);

    // Reset zoom.
    chart::zoom_reset();
}
void chart::clear()
{
    chart::m_series_raw->clear();
    chart::m_series_fit->clear();
    zoom_reset();
}

void chart::zoom_reset()
{
    // Check if any data exists.
    if(chart::m_series_raw->points().empty())
    {
        chart::m_axis_x->setRange(0, 1);
        chart::m_axis_y->setRange(-1, 1);
        return;
    }

    // Get the time limits.
    double x_min = chart::m_series_raw->points().first().x();
    double x_max = chart::m_series_raw->points().last().x();
    chart::m_axis_x->setRange(x_min, x_max);

    // Get the min/max part of the raw signal.
    auto raw_points = chart::m_series_raw->points();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for(auto point = raw_points.cbegin(); point != raw_points.cend(); ++point)
    {
        y_min = std::min(y_min, point->y());
        y_max = std::max(y_max, point->y());
    }

    // Calculate buffer.
    double buffer = 0.2 * (y_max - y_min);

    // Set y range.
    chart::m_axis_y->setRange(y_min - buffer, y_max + buffer);
}
