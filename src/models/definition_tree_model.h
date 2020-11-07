#ifndef DEFINITION_TREE_MODEL_H
#define DEFINITION_TREE_MODEL_H

#include <QAbstractItemModel>

#include <message_introspection/definition_tree.h>

#include "models/definition_tree_item.h"

namespace models {

class definition_tree_model
    : public QAbstractItemModel
{
    Q_OBJECT
public:
    definition_tree_model(QObject* parent = nullptr);
    ~definition_tree_model();

    void set_definition_tree(const message_introspection::definition_tree_t& definition_tree);

    QVariant headerData(int32_t section, Qt::Orientation orientation, int32_t role = Qt::ItemDataRole::DisplayRole) const override;

    QVariant data(const QModelIndex& index, int32_t role) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int32_t role = Qt::ItemDataRole::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;

    QModelIndex index(int32_t row, int32_t column, const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;

    int32_t rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int32_t columnCount(const QModelIndex& parent = QModelIndex()) const override;

    message_introspection::definition_t get_definition(const QModelIndex& index) const;
    uint32_t get_array_index(const QModelIndex& index) const;

private:
    message_introspection::definition_tree_t m_definition_tree;
    definition_tree_item* m_root_item;

    void add_definition_tree(const message_introspection::definition_tree_t& definition_tree, definition_tree_item* item);

    definition_tree_item* get_item(const QModelIndex& index = QModelIndex()) const;
};

}

#endif
