/// \file gui/form_array.h
/// \brief Defines the form_array class.
#ifndef FORM_ARRAY_H
#define FORM_ARRAY_H

#include <QDialog>

namespace Ui {
class form_array;
}

#include "data/candidate_field.h"

/// \brief A form for selecting path array indices.
class form_array
    : public QDialog
{
    Q_OBJECT

public:
    // CONSTRUCTORS
    /// \brief Creates a new form_array instance.
    /// \param candidate_field The candidate field to poulate arrays for.
    /// \param parent The parent widget.
    explicit form_array(const std::shared_ptr<data::candidate_field_t>& candidate_field, QWidget *parent = nullptr);
    ~form_array();

private slots:
    /// \brief Handles buttonBox::accepted signals.
    void on_buttonBox_accepted();
    /// \brief Handles buttonBox::rejected signals.
    void on_buttonBox_rejected();

private:
    // UI
    /// \brief The UI instance of the form.
    Ui::form_array *ui;

    // COMPONENTS
    /// \brief The candidate field instance the form manages.
    std::shared_ptr<data::candidate_field_t> m_candidate_field;
};

#endif // FORM_ARRAY_H
