#include "form_field.h"
#include "ui_form_field.h"

form_field::form_field(const std::shared_ptr<data_set>& data_set, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::form_field)
{
    // Set up UI
    ui->setupUi(this);


}

form_field::~form_field()
{
    delete ui;
}
