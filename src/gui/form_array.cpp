#include "form_array.h"
#include "ui_form_array.h"

#include <QLineEdit>
#include <QValidator>

// CONSTRUCTORS
form_array::form_array(const std::shared_ptr<data::candidate_field_t>& candidate_field, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::form_array)
{
    ui->setupUi(this);

    // Store candidate field
    form_array::m_candidate_field = candidate_field;

    // Populate selected path.
    form_array::ui->lineedit_selected_path->setText(QString::fromStdString(form_array::m_candidate_field->array_path()));

    // Populate the path grid.
    for(uint32_t i = 0; i < form_array::m_candidate_field->n_path_parts(); ++i)
    {
        // Get reference to path part's definition.
        auto definition = form_array::m_candidate_field->definition(i);

        // Build and add the label.
        QLabel* path_label = new QLabel();
        path_label->setText(QString::fromStdString(definition.name() + definition.array()));
        form_array::ui->layout_arrays->addWidget(path_label, i, 0, Qt::AlignmentFlag::AlignHCenter);

        if(definition.is_array())
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
            if(definition.array_length() != 0)
            {
                validator->setTop(std::max(definition.array_length(), 1U) - 1U);
            }
            lineedit_array->setValidator(validator);
            // Add it to the grid.
            form_array::ui->layout_arrays->addWidget(lineedit_array, i, 1, Qt::AlignmentFlag::AlignHCenter);
        }
        else
        {
            // Display not-array indicator.
            QLabel* array_label = new QLabel();
            array_label->setText("-");
            form_array::ui->layout_arrays->addWidget(array_label, i, 1, Qt::AlignmentFlag::AlignHCenter);
        }
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
    // Build array indices.
    std::vector<uint32_t> indices;
    for(uint32_t i = 0; i < form_array::m_candidate_field->n_path_parts(); ++i)
    {
        // Get reference to path part's definition.
        auto& definition = form_array::m_candidate_field->definition(i);

        // Check if this path part is an array.
        if(definition.is_array())
        {
            // Get the associated line edit.
            QLineEdit* lineedit = reinterpret_cast<QLineEdit*>(form_array::ui->layout_arrays->itemAtPosition(i, 1)->widget());
            indices.push_back(lineedit->text().toUInt());
        }
        else
        {
            indices.push_back(0);
        }
    }

    // Select indices on candidate field.
    form_array::m_candidate_field->select_array_indices(indices);

    // Signal that dialog as completed.
    form_array::done(QDialog::DialogCode::Accepted);
}
void form_array::on_buttonBox_rejected()
{
    // Signal that dialog has been cancelled.
    form_array::done(QDialog::DialogCode::Rejected);
}
