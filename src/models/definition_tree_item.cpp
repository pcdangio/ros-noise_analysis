#include "definition_tree_item.h"

using namespace models;

definition_tree_item::definition_tree_item(definition_tree_item* parent)
{
    definition_tree_item::m_parent = parent;
}
definition_tree_item::~definition_tree_item()
{
    // Delete all children.
    for(auto child = definition_tree_item::m_children.begin(); child != definition_tree_item::m_children.end(); ++child)
    {
        delete (*child);
    }
}

definition_tree_item* definition_tree_item::parent() const
{
    return definition_tree_item::m_parent;
}
uint32_t definition_tree_item::get_parent_index() const
{
    // Check if this item has a parent.
    if(!definition_tree_item::m_parent)
    {
        return 0;
    }

    // Iterate through parent's children to find this.
    for(uint32_t i = 0; i < definition_tree_item::m_parent->n_children(); ++i)
    {
        if(definition_tree_item::m_parent->get_child(i) == this)
        {
            return i;
        }
    }

    return 0;
}

void definition_tree_item::add_child(const message_introspection::definition_t *definition)
{
    // Create new item instance.
    definition_tree_item* new_item = new definition_tree_item(this);

    // Set it's values.
    new_item->definition = definition;
    new_item->indexed_element = 0;

    definition_tree_item::m_children.push_back(new_item);
}
uint32_t definition_tree_item::n_children() const
{
    return definition_tree_item::m_children.size();
}
definition_tree_item* definition_tree_item::get_child(uint32_t index) const
{
    if(index < definition_tree_item::m_children.size())
    {
        return definition_tree_item::m_children[index];
    }
    else
    {
        return nullptr;
    }
}
