#ifndef FORM_FIELD_H
#define FORM_FIELD_H

#include <QDialog>

namespace Ui {
class form_field;
}

#include "data_set.h"

class form_field : public QDialog
{
    Q_OBJECT

public:
    explicit form_field(const std::shared_ptr<data_set> data_set, QWidget *parent = nullptr);
    ~form_field();

private slots:
    void on_combobox_topic_currentIndexChanged(const QString& string);

private:
    Ui::form_field *ui;

    std::shared_ptr<data_set> m_data_set;

    void add_tree_item(const message_introspection::definition_tree_t& definition_tree, QTreeWidgetItem* tree);
};

#endif // FORM_FIELD_H
