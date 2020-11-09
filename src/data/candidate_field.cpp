#include "data/candidate_field.h"

using namespace data;

// CONSTRUCTORS
candidate_field_t::candidate_field_t(const std::shared_ptr<candidate_topic_t>& parent_topic, const std::string& field_path)
{
    // Store parent candidate topic.
    candidate_field_t::m_parent_topic = parent_topic;

    // Store the base path.
    candidate_field_t::m_base_path = field_path;

    // Get the path part definitions
    candidate_field_t::m_parent_topic->definition_tree.get_path_definitions(field_path, candidate_field_t::m_definitions);

    // Determine if path has arrays.
    candidate_field_t::m_has_arrays = false;
    for(auto path_part = candidate_field_t::m_definitions.cbegin(); path_part != candidate_field_t::m_definitions.cend(); ++path_part)
    {
        if(path_part->is_array())
        {
            candidate_field_t::m_has_arrays = true;
            break;
        }
    }

    // Set up array path.
    if(!candidate_field_t::m_has_arrays)
    {
        candidate_field_t::m_array_path = candidate_field_t::m_base_path;
    }
    else
    {
        for(auto path_part = candidate_field_t::m_definitions.cbegin(); path_part != candidate_field_t::m_definitions.cend(); ++path_part)
        {
            if(path_part != candidate_field_t::m_definitions.cbegin())
            {
                candidate_field_t::m_array_path += ".";
            }
            candidate_field_t::m_array_path += path_part->name() + path_part->array();
        }
    }

    // Initialize full path with zero indices.
    if(!candidate_field_t::m_has_arrays)
    {
        candidate_field_t::m_full_path = candidate_field_t::m_base_path;
    }
    else
    {
        std::vector<uint32_t> indices(candidate_field_t::m_definitions.size(), 0);
        candidate_field_t::select_array_indices(indices);
    }
}

// PATH
std::string candidate_field_t::topic_name() const
{
    return candidate_field_t::m_parent_topic->topic_name;
}
std::string candidate_field_t::base_path() const
{
    return candidate_field_t::m_base_path;
}
std::string candidate_field_t::array_path() const
{
    return candidate_field_t::m_array_path;
}
std::string candidate_field_t::full_path() const
{
    return candidate_field_t::m_full_path;
}
uint32_t candidate_field_t::n_path_parts() const
{
    return candidate_field_t::m_definitions.size();
}

// DEFINITION
bool candidate_field_t::is_primitive() const
{
    if(candidate_field_t::m_definitions.empty())
    {
        return false;
    }

    return candidate_field_t::m_definitions.back().is_primitive();
}
const message_introspection::definition_t& candidate_field_t::definition(uint32_t path_part_index) const
{
    return candidate_field_t::m_definitions[path_part_index];
}

// ARRAY
bool candidate_field_t::has_arrays() const
{
    return candidate_field_t::m_has_arrays;
}
void candidate_field_t::select_array_indices(const std::vector<uint32_t> array_indices)
{
    // First check if any arrays exist.
    if(!candidate_field_t::m_has_arrays)
    {
        return;
    }

    // Reset full path to build new.
    candidate_field_t::m_full_path.clear();

    // Iterate through definitions/indices to rebuild full path.
    for(uint32_t i = 0; i < candidate_field_t::m_definitions.size() && i < array_indices.size(); ++i)
    {
        // Get reference to definition.
        auto& definition = candidate_field_t::m_definitions[i];

        // Add delimiter.
        if(i > 0)
        {
            candidate_field_t::m_full_path += ".";
        }

        // Add name.
        candidate_field_t::m_full_path += definition.name();

        // If array, add index.
        if(definition.is_array())
        {
            candidate_field_t::m_full_path += "[" + std::to_string(array_indices[i]) + "]";
        }
    }
}
