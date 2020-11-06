#ifndef FORM_FIELD_H
#define FORM_FIELD_H

#include <QWidget>

namespace Ui {
class form_field;
}

#include "data_set.h"

class form_field : public QWidget
{
    Q_OBJECT

public:
    explicit form_field(const std::shared_ptr<data_set>& data_set, QWidget* parent = nullptr);
    ~form_field();

private:
    Ui::form_field *ui;

    std::shared_ptr<data_set> m_data_set;
};

#endif // FORM_FIELD_H
