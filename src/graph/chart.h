/// \file graph/chart.h
/// \brief Defines the graph::chart class.
#ifndef GRAPH___CHART_H
#define GRAPH___CHART_H

#include "data/dataset.h"

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLineSeries>

/// \brief Includes all components for graphing operations.
namespace graph {

/// \brief Manages a chart for plotting noise data.
class chart
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new chart instance.
    chart();

    // METHODS
    /// \brief Gets a pointer to the internal chart instance.
    /// \returns A pointer to the internal chart.
    QtCharts::QChart* get_chart() const;
    /// \brief Plots a dataset to the chart.
    /// \param dataset The dataset to plot.
    void plot_dataset(const std::shared_ptr<data::dataset>& dataset);
    /// \brief Resets the zoom level to a default range.
    void zoom_reset();

private:
    // CHART
    /// \brief The internal chart instance.
    QtCharts::QChart* m_chart;

    // AXES
    /// \brief The chart's x axis.
    QtCharts::QValueAxis* m_axis_x;
    /// \brief The chart's y axis.
    QtCharts::QValueAxis* m_axis_y;

    // SERIES
    /// \brief The raw data series.
    QtCharts::QLineSeries* m_series_raw;
    /// \brief The fit data series.
    QtCharts::QLineSeries* m_series_fit;
    /// \brief The noise + data series.
    QtCharts::QLineSeries* m_series_noise_p;
    /// \brief The noise - data series.
    QtCharts::QLineSeries* m_series_noise_m;
};

}

#endif // GRAPH_H
