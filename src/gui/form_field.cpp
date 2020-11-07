#include "form_field.h"
#include "ui_form_field.h"

#include "models/definition_tree_model.h"

form_field::form_field(const std::shared_ptr<data_set> data_set, QWidget *parent)
    : QDialog(parent),
    ui(new Ui::form_field)
{
    // Set up UI
    form_field::ui->setupUi(this);
    // Set up treeview.
    form_field::ui->tree_message->setModel(new models::definition_tree_model());
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
    // Get the definition tree.
    message_introspection::definition_tree_t definition_tree;
    form_field::m_data_set->get_definition_tree(string.toStdString(), definition_tree);

    // Update the model.
    models::definition_tree_model* model = reinterpret_cast<models::definition_tree_model*>(form_field::ui->tree_message->model());
    model->set_definition_tree(string.toStdString(), definition_tree);
}
