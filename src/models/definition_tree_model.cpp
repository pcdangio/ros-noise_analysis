#include "definition_tree_model.h"

using namespace models;

definition_tree_model::definition_tree_model(QObject* parent)
    : QAbstractItemModel(parent)
{
    // Initialize pointers.
    definition_tree_model::m_root_item = nullptr;

    // Set a blank definition tree.
    definition_tree_model::set_definition_tree("", message_introspection::definition_tree_t());
}
definition_tree_model::~definition_tree_model()
{
    // Clean up root item.
    delete definition_tree_model::m_root_item;
}

void definition_tree_model::set_definition_tree(const std::string& topic, const message_introspection::definition_tree_t& definition_tree)
{
    // Indicate that the model is being reset.
    definition_tree_model::beginInsertRows()

    // Initialize new root item.
    delete definition_tree_model::m_root_item;
    definition_tree_model::m_root_item = new definition_tree_item(nullptr);

    // Recurse through tree.
    definition_tree_model::add_definition_tree(definition_tree, definition_tree_model::m_root_item);

    // Fix the root item's name.
    definition_tree_model::m_root_item->definition.update_name(topic);

    // Indicate that the model has been reset.
    definition_tree_model::endResetModel();
}
void definition_tree_model::add_definition_tree(const message_introspection::definition_tree_t& definition_tree, definition_tree_item* item)
{
    // Store this tree's definition.
    item->definition = definition_tree.definition;
    // Initialize the array index.
    item->indexed_element = 0;

    // Iterate through children.
    for(auto field = definition_tree.fields.cbegin(); field != definition_tree.fields.cend(); ++field)
    {
        // Add a new child item.
        auto child = item->add_child(field->definition);

        // Recurse into field/child.
        definition_tree_model::add_definition_tree(*field, child);
    }
}

QVariant definition_tree_model::headerData(int32_t section, Qt::Orientation orientation, int32_t role) const
{
    if(orientation != Qt::Horizontal || role != Qt::ItemDataRole::DisplayRole)
    {
        return QVariant();
    }

    switch(section)
    {
        case 0:
        {
            return QString("Name");
        }
        case 1:
        {
            return QString("Type");
        }
        case 2:
        {
            return QString("Index");
        }
        default:
        {
            return QVariant();
        }
    }
}
#include <ros/console.h>
QVariant definition_tree_model::data(const QModelIndex& index, int32_t role) const
{
    ROS_ERROR_STREAM("get data");
    // Check for valid index.
    if(!index.isValid())
    {
        return QVariant();
    }

    // Check for valid role (must be display or edit)
    if(role != Qt::ItemDataRole::DisplayRole && role != Qt::ItemDataRole::EditRole)
    {
        return QVariant();
    }

    // Get the item.
    auto item = definition_tree_model::get_item(index);

    // Display the desired piece of the definition.
    switch(index.column())
    {
        case 0:
        {
            ROS_ERROR_STREAM(item->definition.name());
            return QString::fromStdString(item->definition.name());
        }
        case 1:
        {
            return QString::fromStdString(item->definition.type());
        }
        case 2:
        {
            if(item->definition.is_array())
            {
                return QString::number(item->indexed_element);
            }
            else
            {
                return QString("-");
            }
        }
        default:
        {
            return QVariant();
        }
    }
}
bool definition_tree_model::setData(const QModelIndex& index, const QVariant& value, int32_t role)
{

}
Qt::ItemFlags definition_tree_model::flags(const QModelIndex& index) const
{
    if(!index.isValid())
    {
        return Qt::ItemFlag::NoItemFlags;
    }

    // Only allow editing array index column and for items that are an array.
    if(index.column() != 2 || !definition_tree_model::get_item(index)->definition.is_array())
    {
        return QAbstractItemModel::flags(index);
    }

    return QAbstractItemModel::flags(index) | Qt::ItemFlag::ItemIsEditable;
}

definition_tree_item* definition_tree_model::get_item(const QModelIndex& index) const
{
    // Check if index is valid.
    if(!index.isValid())
    {
        // Invalid index. Point to definition tree root.
        return definition_tree_model::m_root_item;
    }

    // The QModelIndex internally points to an entry in the root definition tree.
    return reinterpret_cast<definition_tree_item*>(index.internalPointer());
}
QModelIndex definition_tree_model::index(int32_t row, int32_t column, const QModelIndex& parent) const
{
    // Check for valid parent index.
    if(!parent.isValid())
    {
        return QModelIndex();
    }

    // Get the child from the parent.
    auto child_item = definition_tree_model::get_item(parent)->get_child(row);
    if(child_item)
    {
        return definition_tree_model::createIndex(row, column, child_item);
    }
    else
    {
        return QModelIndex();
    }
}
QModelIndex definition_tree_model::parent(const QModelIndex& index) const
{
    if(index.isValid())
    {
        auto parent = definition_tree_model::get_item(index)->parent();
        if(parent)
        {
            // Get the parent's parent.
            return definition_tree_model::createIndex(parent->get_parent_index(), 0, parent);
        }
        else
        {
            return QModelIndex();
        }
    }
    else
    {
        return QModelIndex();
    }
}

int32_t definition_tree_model::rowCount(const QModelIndex& parent) const
{
    // Get the number of children/rows in the parent tree.
    return definition_tree_model::get_item(parent)->n_children();
}
int32_t definition_tree_model::columnCount(const QModelIndex& /*parent*/) const
{
    // Column count is always 3.
    return 3;
}
