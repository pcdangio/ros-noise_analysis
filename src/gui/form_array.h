/// \file gui/form_array.h
/// \brief Defines the form_array class.
#ifndef FORM_ARRAY_H
#define FORM_ARRAY_H

#include <QDialog>

namespace Ui {
class form_array;
}

#include <message_introspection/definition.h>

/// \brief A form for selecting path array indices.
class form_array
    : public QDialog
{
    Q_OBJECT

public:
    // CONSTRUCTORS
    /// \brief Creates a new form_array instance.
    /// \param path_definitions The path definitions to get array indices for.
    /// \param parent The parent widget.
    explicit form_array(const std::vector<message_introspection::definition_t>& path_definitions, QWidget *parent = nullptr);
    ~form_array();

    /// \brief Gets the fully indexed path selected by the user.
    /// \returns The fully index path. Will be an empty string if form cancelled.
    std::string selected_path();

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();

private:
    // UI
    Ui::form_array *ui;

    // VARIABLES
    /// \brief Stores the path definitions to get array indices for.
    std::vector<message_introspection::definition_t> m_path_definitions;
    /// \brief Stores the calculated selected path based on user inputs.
    std::string m_selected_path;
};

#endif // FORM_ARRAY_H
