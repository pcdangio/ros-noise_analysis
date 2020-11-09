/// \file data/candidate_field.h
/// \brief Defines the data::candidate_field_t class.
#ifndef DATA___CANDIDATE_FIELD_H
#define DATA___CANDIDATE_FIELD_H

#include "data/candidate_topic.h"

#include <memory>

namespace data {

/// \brief A field that is a candidate for a dataset.
struct candidate_field_t
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new candidate_field_t instance.
    /// \param parent_topic A shared reference to the candidate field's parent candidate topic.
    /// \param field_path The base path of the candidate field in the definition tree.
    candidate_field_t(const std::shared_ptr<candidate_topic_t>& parent_topic, const std::string& field_path);

    // PATH
    /// \brief Gets the topic name of the candidate field.
    /// \returns The topic name of the field.
    std::string topic_name() const;
    /// \brief Gets the field's defined path, not including defined array indicators.
    /// \returns The field's base path.
    std::string base_path() const;
    /// \brief Gets the field's defined path including defined array indicators.
    /// \returns The field's array path.
    /// \note Array indicators are populated as the message defines, and does not include chosen indices.
    std::string array_path() const;
    /// \brief Gets the field's path including chosen indices for path parts that are arrays.
    /// \returns The field's full path.
    std::string full_path() const;
    /// \brief Gets the number of path parts in the field.
    /// \returns The number of path parts.
    uint32_t n_path_parts() const;

    // DEFINITION
    /// \brief Indicates if the candidate field is a primitive type.
    /// \returns TRUE if the field is a primitive type, otherwise FALSE.
    bool is_primitive() const;
    /// \brief Gets a path part's definition.
    /// \param path_part_index The index to the desired path part.
    /// \returns A const reference to the path part's definition.
    const message_introspection::definition_t& definition(uint32_t path_part_index) const;

    // ARRAY
    /// \brief Indicates if the candidate field has any path parts that are arrays.
    /// \returns TRUE if the field has at least one path part that is an array, otherwise FALSE.
    bool has_arrays() const;
    /// \brief Selects indices for each path part that is an array.
    /// \param array_indices An array of indices that coincides with ALL path parts. Fill non-array parts parts with 0.
    void select_array_indices(const std::vector<uint32_t> array_indices);

private:
    // TOPIC
    /// \brief Stores a shared reference to the candidate field's topic information.
    std::shared_ptr<candidate_topic_t> m_parent_topic;

    // PATH
    /// \brief Stores the field's defined path, with no array indicators.
    std::string m_base_path;
    /// \brief Stores the field's defined path with array indicators.
    std::string m_array_path;
    /// \brief Stores the field's full path, with array indices.
    std::string m_full_path;

    // ARRAY
    /// \brief Indicates if the field has any path parts that are an array.
    bool m_has_arrays;

    // DEFINITION
    /// \brief Stores the definition list of the field's path parts.
    std::vector<message_introspection::definition_t> m_definitions;
};

}

#endif
