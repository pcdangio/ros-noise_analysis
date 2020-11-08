#include "form_array.h"
#include "ui_form_array.h"

#include <QLineEdit>
#include <QValidator>

// CONSTRUCTORS
form_array::form_array(const std::vector<message_introspection::definition_t>& path_definitions, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::form_array)
{
    ui->setupUi(this);

    // Store path definitions.
    form_array::m_path_definitions = path_definitions;

    // Reset selected path.
    form_array::m_selected_path = "";

    // Populate path indicator and selected path.
    std::string path_indicator;
    for(auto path_definition = path_definitions.cbegin(); path_definition != path_definitions.cend(); ++path_definition)
    {
        if(path_definition != path_definitions.cbegin())
        {
            path_indicator += ".";
        }
        path_indicator += path_definition->name() + path_definition->array();
    }
    form_array::ui->lineedit_selected_path->setText(QString::fromStdString(path_indicator));

    // Populate the path grid.
    uint32_t row_index = 0;
    for(auto path_definition = path_definitions.cbegin(); path_definition != path_definitions.cend(); ++path_definition)
    {
        // Build and add the label.
        QLabel* path_label = new QLabel();
        path_label->setText(QString::fromStdString(path_definition->name() + path_definition->array()));
        form_array::ui->layout_arrays->addWidget(path_label, row_index, 0, Qt::AlignmentFlag::AlignHCenter);

        // Build and add array input.
        if(path_definition->is_array())
        {
            // Set up line edit.
            QLineEdit* lineedit_array = new QLineEdit();
            lineedit_array->setText("0");
            lineedit_array->setAlignment(Qt::AlignmentFlag::AlignHCenter);
            lineedit_array->setMinimumWidth(50);
            lineedit_array->setMaximumWidth(50);
            // Add validator.
            QIntValidator* validator = new QIntValidator(lineedit_array);
            validator->setBottom(0);
            if(path_definition->array_length() != 0)
            {
                validator->setTop(std::max(path_definition->array_length(), 1U) - 1U);
            }
            lineedit_array->setValidator(validator);
            // Add it to the grid.
            form_array::ui->layout_arrays->addWidget(lineedit_array, row_index, 1, Qt::AlignmentFlag::AlignHCenter);
        }
        else
        {
            // Display not-array indicator.
            QLabel* array_label = new QLabel();
            array_label->setText("-");
            form_array::ui->layout_arrays->addWidget(array_label, row_index, 1, Qt::AlignmentFlag::AlignHCenter);
        }

        // Increment row counter.
        row_index++;
    }

    // Resize the form.
    form_array::adjustSize();
}
form_array::~form_array()
{
    delete ui;
}

// SLOTS
void form_array::on_buttonBox_accepted()
{
    // Calculate selected path.
    form_array::m_selected_path = "";

    // Build the seleted path with the array indices.
    for(uint32_t i = 0; i < form_array::m_path_definitions.size(); ++i)
    {
        // Get reference to definition.
        auto& definition = form_array::m_path_definitions[i];

        // Add name to path.
        if(i > 0)
        {
            form_array::m_selected_path += ".";
        }
        form_array::m_selected_path += definition.name();

        // Add array part.
        if(definition.is_array())
        {
            // Add open bracket.
            form_array::m_selected_path += "[";
            // Get the index from the associate line edit.
            QLineEdit* lineedit = reinterpret_cast<QLineEdit*>(form_array::ui->layout_arrays->itemAtPosition(i, 1)->widget());
            form_array::m_selected_path += lineedit->text().toStdString();
            // Add close bracket.
            form_array::m_selected_path += "]";
        }
    }

    // Signal that dialog as completed.
    form_array::done(QDialog::DialogCode::Accepted);
}
void form_array::on_buttonBox_rejected()
{
    // Signal that dialog has been cancelled.
    form_array::done(QDialog::DialogCode::Rejected);
}

// METHODS
std::string form_array::selected_path()
{
    return form_array::m_selected_path;
}
