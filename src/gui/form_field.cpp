#include "form_field.h"
#include "ui_form_field.h"

form_field::form_field(const std::shared_ptr<data_set> data_set, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::form_field)
{
    // Set up UI
    form_field::ui->setupUi(this);
    // Set up treeview columns.
    form_field::ui->tree_message->setHeaderLabels({"Name", "Type", "Index"});
    form_field::ui->tree_message->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::Stretch);
    form_field::ui->tree_message->header()->setDefaultAlignment(Qt::AlignHCenter);
    form_field::ui->tree_message->setColumnWidth(1, 200);
    form_field::ui->tree_message->setColumnWidth(2, 50);

    // Store data_set.
    form_field::m_data_set = data_set;

    // Update topics combobox.
    form_field::ui->combobox_topic->addItem("Select topic...");
    auto topics = form_field::m_data_set->bag_topics();
    for(auto topic = topics.cbegin(); topic != topics.cend(); ++topic)
    {
        form_field::ui->combobox_topic->addItem(QString::fromStdString(*topic));
    }
}

form_field::~form_field()
{
    delete ui;
}

void form_field::on_combobox_topic_currentIndexChanged(const QString& string)
{
    // Clear the tree.
    form_field::ui->tree_message->clear();

    // Populate the new tree.
    if(form_field::ui->combobox_topic->currentIndex() > 0)
    {
        // Populate the tree.
        // Get definition tree from data set.
        message_introspection::definition_tree_t definition_tree;
        if(form_field::m_data_set->get_definition_tree(string.toStdString(), definition_tree))
        {
            // Create a top level item for the message itself.
            QTreeWidgetItem* top_level = new QTreeWidgetItem();

            // Recurse through tree.
            form_field::add_tree_item(definition_tree, top_level);

            // Fix top level's name to topic name.
            top_level->setText(0, string);

            // Add top level item to tree.
            form_field::ui->tree_message->addTopLevelItem(top_level);

            // Expand all children.
            form_field::ui->tree_message->expandAll();
        }
    }
}

void form_field::add_tree_item(const message_introspection::definition_tree_t& definition_tree, QTreeWidgetItem* tree)
{
    // Set top level's details.
    tree->setText(0, QString::fromStdString(definition_tree.definition.name() + definition_tree.definition.array()));
    tree->setText(1, QString::fromStdString(definition_tree.definition.type()));
    if(definition_tree.definition.is_array())
    {
        tree->setText(2, "0");
    }

    // Set top level's format.
    tree->setTextAlignment(2, Qt::AlignmentFlag::AlignHCenter);

    // Iterate through sub trees.
    for(auto field = definition_tree.fields.cbegin(); field != definition_tree.fields.cend(); ++field)
    {
        // Create new item.
        QTreeWidgetItem* item = new QTreeWidgetItem();

        // Recurse into sub tree.
        form_field::add_tree_item(*field, item);

        // Add item to parent tree.
        tree->addChild(item);
    }
}
