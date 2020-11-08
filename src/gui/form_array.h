#ifndef FORM_ARRAY_H
#define FORM_ARRAY_H

#include <QDialog>

namespace Ui {
class form_array;
}

#include <message_introspection/definition.h>

class form_array : public QDialog
{
    Q_OBJECT

public:
    explicit form_array(const std::vector<message_introspection::definition_t>& path_definitions, QWidget *parent = nullptr);
    ~form_array();

    std::string selected_path();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::form_array *ui;

    std::vector<message_introspection::definition_t> m_path_definitions;

    std::string m_selected_path;
};

#endif // FORM_ARRAY_H
