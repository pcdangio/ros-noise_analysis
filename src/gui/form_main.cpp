#include "gui/form_main.h"
#include "ui_form_main.h"

#include "gui/form_array.h"
#include "data/candidate_field.h"

#include <QFileDialog>
#include <QToolBar>

// CONSTRUCTORS
form_main::form_main(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::form_main)
{
    // Set up UI.
    form_main::ui->setupUi(this);
    // Set up UI elements.
    form_main::setup_splitter();
    form_main::setup_tree_message();
    form_main::setup_toolbar_table();
    form_main::setup_table_datasets();
    form_main::setup_chartview();
    form_main::setup_toolbar_chart();

    // Connect to data_interface signals.
    connect(&(form_main::m_data_interface), &data::data_interface::dataset_calculated, this, &form_main::dataset_calculated);

    // Set up node handle.
    form_main::m_node = std::make_shared<ros::NodeHandle>();

    // Start ros spinner.
    connect(&(form_main::m_ros_spinner), &QTimer::timeout, this, &form_main::ros_spin);
    form_main::m_ros_spinner.start(10);
}

form_main::~form_main()
{
    delete ui;
}

// UI
void form_main::setup_splitter()
{
    // Set data splitter to stretch equally.
    form_main::ui->splitter_fields->setStretchFactor(0,1);
    form_main::ui->splitter_fields->setStretchFactor(1,1);

    // Set main splitter to stretch equally.
    form_main::ui->splitter_main->setStretchFactor(0,1);
    form_main::ui->splitter_main->setStretchFactor(1,1);
}
void form_main::setup_tree_message()
{
    // Set up treeview.
    form_main::ui->tree_message->setHeaderLabels({"Name", "Type"});
    form_main::ui->tree_message->header()->setDefaultAlignment(Qt::AlignHCenter);
    form_main::ui->tree_message->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::Stretch);
    form_main::ui->tree_message->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::Interactive);

    // Initialize combobox.
    form_main::ui->combobox_topics->addItem("Select topic...");
}
void form_main::setup_toolbar_table()
{
    // Create new toolbar for table.
    QToolBar* toolbar_table = new QToolBar(this);

    // Add actions for field control.
    auto action_add = toolbar_table->addAction(QIcon::fromTheme("list-add"), "Add field...");
    auto action_remove = toolbar_table->addAction(QIcon::fromTheme("list-remove"), "Remove field");
    auto action_clear = toolbar_table->addAction(QIcon::fromTheme("edit-clear"), "Clear all fields");

    // Add actions for field movement.
    toolbar_table->addSeparator();
    auto action_up = toolbar_table->addAction(QIcon::fromTheme("go-up"), "Move field up");
    auto action_down = toolbar_table->addAction(QIcon::fromTheme("go-down"), "Move field down");

    // Add actions for analysis saving.
    toolbar_table->addSeparator();
    auto action_save = toolbar_table->addAction(QIcon::fromTheme("document-save"), "Save analysis...");
    auto action_saveas = toolbar_table->addAction(QIcon::fromTheme("document-save-as"), "Save analysis as...");
    auto action_open = toolbar_table->addAction(QIcon::fromTheme("document-open"), "Open analysis...");

    // Add toolbar to table's layout.
    form_main::ui->layout_fields->insertWidget(0, toolbar_table);

    // Make connections.
    connect(action_add, &QAction::triggered, this, &form_main::toolbar_table_add);
    connect(action_remove, &QAction::triggered, this, &form_main::toolbar_table_remove);
    connect(action_clear, &QAction::triggered, this, &form_main::toolbar_table_clear);
    connect(action_up, &QAction::triggered, this, &form_main::toolbar_table_up);
    connect(action_down, &QAction::triggered, this, &form_main::toolbar_table_down);
    connect(action_save, &QAction::triggered, this, &form_main::toolbar_table_save);
    connect(action_saveas, &QAction::triggered, this, &form_main::toolbar_table_saveas);
    connect(action_open, &QAction::triggered, this, &form_main::toolbar_table_open);
}
void form_main::setup_table_datasets()
{
    // Set header captions.
    form_main::ui->table_datasets->setHorizontalHeaderLabels({"Name", "Variance", "Path"});

    // Set horizontal header sizers.
    form_main::ui->table_datasets->setColumnWidth(0, 100);
    form_main::ui->table_datasets->setColumnWidth(1, 100);
    form_main::ui->table_datasets->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeMode::Fixed);
    form_main::ui->table_datasets->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeMode::Fixed);
    form_main::ui->table_datasets->horizontalHeader()->setStretchLastSection(true);

    // Hide vertical header.
    form_main::ui->table_datasets->verticalHeader()->setVisible(false);

    // Set selection mode.
    form_main::ui->table_datasets->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
    form_main::ui->table_datasets->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
}
void form_main::setup_chartview()
{
    // Delete old chart and add new chart.
    auto chart_old = form_main::ui->chartview->chart();
    form_main::ui->chartview->setChart(form_main::m_chart.get_chart());
    delete chart_old;
}
void form_main::setup_toolbar_chart()
{
    // Create new toolbar for the chart.
    QToolBar* toolbar_chart = new QToolBar(this);

    // Set vertical orientation.
    toolbar_chart->setOrientation(Qt::Orientation::Vertical);

    // Add actions for zoom.
    auto action_zoom_all = toolbar_chart->addAction(QIcon::fromTheme("zoom-out"), "Reset Zoom");
    auto action_zoom_rectangle = toolbar_chart->addAction(QIcon::fromTheme("zoom-in"), "Zoom Rectangle");
    auto action_zoom_horizontal = toolbar_chart->addAction(QIcon::fromTheme("object-flip-horizontal"), "Zoom Horizontal");
    auto action_zoom_vertical = toolbar_chart->addAction(QIcon::fromTheme("object-flip-vertical"), "Zoom Vertical");

    // Add toolbar to chart's layout.
    form_main::ui->layout_chartview->insertWidget(0, toolbar_chart);

    // Make connections.
    connect(action_zoom_all, &QAction::triggered, this, &form_main::toolbar_zoom_all);
    connect(action_zoom_rectangle, &QAction::triggered, this, &form_main::toolbar_zoom_rectangle);
    connect(action_zoom_horizontal, &QAction::triggered, this, &form_main::toolbar_zoom_horizontal);
    connect(action_zoom_vertical, &QAction::triggered, this, &form_main::toolbar_zoom_vertical);
}

void form_main::update_combobox_topics()
{
    // Clear existing topics.
    form_main::ui->combobox_topics->clear();

    // Add new topics.
    form_main::ui->combobox_topics->addItem("Select topic...");
    auto topics = form_main::m_data_interface.bag_topics();
    for(auto topic = topics.cbegin(); topic != topics.cend(); ++topic)
    {
        form_main::ui->combobox_topics->addItem(QString::fromStdString(*topic));
    }
}
void form_main::update_tree_message()
{
    // Clear the current tree contents.
    form_main::ui->tree_message->clear();

    // Check if candidate topic is empty.
    if(!form_main::m_candidate_topic)
    {
        return;
    }

    // Create root tree item.
    QTreeWidgetItem* root_item = new QTreeWidgetItem();

    // Recursively add tree items.
    form_main::add_tree_item(form_main::m_candidate_topic->definition_tree, root_item);

    // Update root item's empty name to topic name.
    root_item->setText(0, QString::fromStdString(form_main::m_candidate_topic->topic_name));

    // Add root item to tree widget.
    form_main::ui->tree_message->addTopLevelItem(root_item);

    // Resize type column.
    form_main::ui->tree_message->resizeColumnToContents(1);

    // Expand all items.
    form_main::ui->tree_message->expandAll();
}
void form_main::add_tree_item(const message_introspection::definition_tree_t& definition_tree, QTreeWidgetItem* item)
{
    // Set the item's data.
    item->setText(0, QString::fromStdString(definition_tree.definition.name() + definition_tree.definition.array()));
    item->setText(1, QString::fromStdString(definition_tree.definition.type()));

    // Store the field's path in the user data.
    item->setData(0, Qt::ItemDataRole::UserRole, QString::fromStdString(definition_tree.definition.path()));

    // Add children.
    for(auto field = definition_tree.fields.cbegin(); field != definition_tree.fields.cend(); ++field)
    {
        // Create new child item.
        QTreeWidgetItem* child = new QTreeWidgetItem();
        // Recurse into child item.
        form_main::add_tree_item(*field, child);
        // Add child to item.
        item->addChild(child);
    }
}
void form_main::update_table_datasets()
{
    // Update table size.
    form_main::ui->table_datasets->setRowCount(form_main::m_data_interface.n_datasets());

    // Populate cells.
    for(uint32_t i = 0; i < form_main::m_data_interface.n_datasets(); ++i)
    {
        // Get dataset pointer.
        auto dataset = form_main::m_data_interface.get_dataset(i);

        // Set name column.
        auto item_name = new QTableWidgetItem();
        item_name->setText(QString::fromStdString(dataset->name()));
        item_name->setTextAlignment(Qt::AlignmentFlag::AlignCenter);
        form_main::ui->table_datasets->setItem(i, 0, item_name);

        // Set variance column.
        auto item_variance = new QTableWidgetItem();
        if(std::isnan(dataset->variance()))
        {
            item_variance->setText("-");
        }
        else
        {
            item_variance->setText(QString::number(dataset->variance()));
        }
        item_variance->setTextAlignment(Qt::AlignmentFlag::AlignCenter);
        item_variance->setFlags(item_variance->flags() & ~Qt::ItemFlag::ItemIsEditable);
        form_main::ui->table_datasets->setItem(i, 1, item_variance);

        // Set path column
        auto item_path = new QTableWidgetItem();
        item_path->setText(QString::fromStdString(dataset->topic_name() + "::" + dataset->field_path()));
        item_path->setFlags(item_path->flags() & ~Qt::ItemFlag::ItemIsEditable);
        form_main::ui->table_datasets->setItem(i, 2, item_path);
    }
}
void form_main::update_plot_view(std::shared_ptr<data::dataset> dataset)
{
    // Check if there is a dataset to plot.
    if(dataset)
    {
        // Update the plot view.

        // Update the chart.
        form_main::m_chart.plot_dataset(dataset);

        // Update the sliders.
        // Block signals so slider value changed is not signaled.
        form_main::blockSignals(true);
        form_main::ui->slider_basis_ratio->setValue(dataset->fit_basis_ratio() * 100.0);
        form_main::ui->slider_smoothness->setValue(dataset->fit_smoothing() * 10.0);
        form_main::blockSignals(false);
    }
    else
    {
        // Clear the plot view.

        // Clear the chart.
        form_main::m_chart.plot_dataset(nullptr);

        // Reset the sliders.
        // Block signals so slider value changed is not signaled.
        form_main::blockSignals(true);
        form_main::ui->slider_basis_ratio->setValue(1);
        form_main::ui->slider_smoothness->setValue(10);
        form_main::blockSignals(false);
    }
}

bool form_main::get_selected_dataset(uint32_t& index)
{
    // Get current selected range.
    auto selected_range = form_main::ui->table_datasets->selectedRanges();

    // Check if the selection is empty.
    if(selected_range.empty())
    {
        return false;
    }

    // Determine what the selected row index is.
    index = selected_range.front().topRow();

    return true;
}

// SLOTS - TOOLBAR_TABLE
void form_main::toolbar_table_add()
{
    // Get the currently selected item from tree_message.
    auto selected_items = form_main::ui->tree_message->selectedItems();
    if(selected_items.empty())
    {
        // Warn that no item is selected to add.
        QMessageBox::warning(this, "Error", "You must select a message topic/field to add.");
        return;
    }

    // Get the selected item's path.
    std::string path = selected_items.front()->data(0, Qt::ItemDataRole::UserRole).toString().toStdString();

    // Create a new candidate field.
    auto candidate_field = std::make_shared<data::candidate_field_t>(form_main::m_candidate_topic, path);

    // Check if the selected field is primitive.
    if(!candidate_field->is_primitive())
    {
        // Warn that the selected path is not a primitive type.
        QMessageBox::warning(this, "Error", "You may only add fields that are a primitive ROS message type.");
        return;
    }

    // Check if the selected field has array path parts.
    if(candidate_field->has_arrays())
    {
        // Show form_array to get array indices.
        form_array dialog(candidate_field, this);
        if(!dialog.exec())
        {
            return;
        }
    }

    // Add the candidate as a new dataset.
    if(!form_main::m_data_interface.add_dataset(candidate_field))
    {
        // Warn that the dataset already exists.
        QMessageBox::warning(this, "Error", "Dataset already exists.");
        return;
    }

    // Update datasets table.
    form_main::update_table_datasets();

    // Select the last dataset in the list, which was the added.
    form_main::ui->table_datasets->selectRow(form_main::ui->table_datasets->rowCount() - 1);

    // Clear the plot view AFTER selection has been updated.
    form_main::update_plot_view(nullptr);
}
void form_main::toolbar_table_remove()
{
    // Get the selected dataset index to remove.
    uint32_t selected_index;
    if(!form_main::get_selected_dataset(selected_index))
    {
        QMessageBox::warning(this, "Error", "You must first select a dataset to remove.");
        return;
    }

    if(!form_main::m_data_interface.remove_dataset(selected_index))
    {
        QMessageBox::warning(this, "Error", "Failed to remove dataset");
        return;
    }

    // Update table.
    form_main::update_table_datasets();

    // Clear table selection.
    form_main::ui->table_datasets->clearSelection();

    // Clear plot view.
    form_main::update_plot_view(nullptr);
}
void form_main::toolbar_table_clear()
{
    // Remove all datasets.
    form_main::m_data_interface.clear_datasets();

    // Update table.
    form_main::update_table_datasets();

    // Clear plot view.
    form_main::update_plot_view(nullptr);;
}
void form_main::toolbar_table_up()
{
    // Get the selected dataset index to move.
    uint32_t selected_index;
    if(!form_main::get_selected_dataset(selected_index))
    {
        QMessageBox::warning(this, "Error", "You must first select a dataset to move.");
        return;
    }

    // Check if dataset can be moved up.
    if(selected_index == 0)
    {
        return;
    }

    // Get new index.
    uint32_t new_index = selected_index - 1;

    if(!form_main::m_data_interface.move_dataset(selected_index, new_index))
    {
        QMessageBox::warning(this, "Error", "Failed to move dataset.");
        return;
    }

    // Update table.
    form_main::update_table_datasets();

    // Select the new index to maintain selection.
    form_main::ui->table_datasets->selectRow(new_index);
}
void form_main::toolbar_table_down()
{
    // Get the selected dataset index to move.
    uint32_t selected_index;
    if(!form_main::get_selected_dataset(selected_index))
    {
        QMessageBox::warning(this, "Error", "You must first select a dataset to move.");
        return;
    }

    // Check if dataset can be moved down.
    if(selected_index == (form_main::m_data_interface.n_datasets() - 1))
    {
        return;
    }

    // Get new index.
    uint32_t new_index = selected_index + 1;

    if(!form_main::m_data_interface.move_dataset(selected_index, new_index))
    {
        QMessageBox::warning(this, "Error", "Failed to move dataset.");
        return;
    }

    // Update table.
    form_main::update_table_datasets();

    // Select the new index to maintain selection.
    form_main::ui->table_datasets->selectRow(new_index);
}
void form_main::toolbar_table_save()
{

}
void form_main::toolbar_table_saveas()
{

}
void form_main::toolbar_table_open()
{

}

// SLOTS - TOOLBAR_CHART
void form_main::toolbar_zoom_all()
{
    // Set chart to reset the zoom.
    form_main::m_chart.zoom_reset();
    form_main::ui->chartview->setCursor(Qt::CursorShape::ArrowCursor);
}
void form_main::toolbar_zoom_rectangle()
{
    form_main::ui->chartview->setRubberBand(QChartView::RubberBand::RectangleRubberBand);
    form_main::ui->chartview->setCursor(Qt::CursorShape::CrossCursor);
}
void form_main::toolbar_zoom_horizontal()
{
    form_main::ui->chartview->setRubberBand(QChartView::RubberBand::HorizontalRubberBand);
    form_main::ui->chartview->setCursor(Qt::CursorShape::SplitHCursor);
}
void form_main::toolbar_zoom_vertical()
{
    form_main::ui->chartview->setRubberBand(QChartView::RubberBand::VerticalRubberBand);
    form_main::ui->chartview->setCursor(Qt::CursorShape::SplitVCursor);
}

// ROS
void form_main::ros_spin()
{
    // Handle callbacks.
    ros::spinOnce();

    // Quit if ROS shutting down.
    if(!ros::ok())
    {
        QApplication::quit();
    }
}


void form_main::on_button_open_bag_clicked()
{
    // Build open file dialog.
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::FileMode::ExistingFile);
    dialog.setViewMode(QFileDialog::ViewMode::Detail);
    dialog.setNameFilter("ROS Bag (*.bag)");

    // Show dialog.
    if(!dialog.exec())
    {
        return;
    }

    // Try to load bag file.
    if(!form_main::m_data_interface.load_bag(dialog.selectedFiles().front().toStdString()))
    {
        QMessageBox::warning(this, "Error", "Error loading bag file. See ROS log for more information.");
        return;
    }

    // Update bag name line edit.
    form_main::ui->lineedit_bag->setText(QString::fromStdString(form_main::m_data_interface.bag_name()));

    // Update topics combobox.
    form_main::update_combobox_topics();

    // Update the dataset table.
    form_main::update_table_datasets();
}



void form_main::on_combobox_topics_currentTextChanged(const QString& text)
{
    // Get a new candidate topic from the data interface.
    form_main::m_candidate_topic = form_main::m_data_interface.get_candidate_topic(text.toStdString());

    // Update the message tree view.
    form_main::update_tree_message();
}

void form_main::dataset_calculated(quint32 index)
{
    // If the newly calculated dataset is the one being displayed, update the plot.

    // Determine what the selected row index is.
    uint32_t selected_index;
    if(!form_main::get_selected_dataset(selected_index))
    {
        // No dataset selected.
        return;
    }

    // Check if the calculated index matches the currently selected index.
    if(selected_index == index)
    {
        // Get pointer to dataset.
        auto calculated_dataset = form_main::m_data_interface.get_dataset(selected_index);

        // Update variance display in table.
        form_main::ui->table_datasets->item(selected_index, 1)->setText(QString::number(calculated_dataset->variance()));

        // Update the plot view for selected dataset.
        form_main::update_plot_view(calculated_dataset);
    }
}


void form_main::on_table_datasets_cellClicked(int row, int /*column*/)
{
    // Update the plot view for selected index.
    form_main::update_plot_view(form_main::m_data_interface.get_dataset(row));
}

void form_main::on_slider_bases_valueChanged(int value)
{
    // Update the selected dataset's basis ratio.
    uint32_t index;
    if(!form_main::get_selected_dataset(index))
    {
        // No dataset selected.
        return;
    }

    // Get selected dataset.
    auto selected_dataset = form_main::m_data_interface.get_dataset(index);

    // Update basis ratio.
    selected_dataset->fit_bases(static_cast<uint32_t>(value));

    // Rerun calculation if it's not already running.
    selected_dataset->calculate();
}

void form_main::on_slider_smoothness_valueChanged(int value)
{
    // Update the selected dataset's smoothing.
    uint32_t index;
    if(!form_main::get_selected_dataset(index))
    {
        // No dataset selected.
        return;
    }

    // Get selected dataset.
    auto selected_dataset = form_main::m_data_interface.get_dataset(index);

    // Update basis ratio.
    selected_dataset->fit_smoothing(static_cast<double>(value) / 10.0);

    // Rerun calculation if it's not already running.
    selected_dataset->calculate();
}
