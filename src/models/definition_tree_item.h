#ifndef DEFINITION_TREE_ITEM_H
#define DEFINITION_TREE_ITEM_H

#include <message_introspection/definition.h>

namespace models {

class definition_tree_item
{
public:
    definition_tree_item(definition_tree_item* parent);
    ~definition_tree_item();

    definition_tree_item* parent() const;
    uint32_t get_parent_index() const;

    void add_child(const message_introspection::definition_t* definition);
    uint32_t n_children() const;
    definition_tree_item* get_child(uint32_t index) const;

    const message_introspection::definition_t* definition;
    uint32_t indexed_element;

private:
    definition_tree_item* m_parent;
    std::vector<definition_tree_item*> m_children;
};

}

#endif
