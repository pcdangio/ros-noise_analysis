/// \file data/candidate_topic.h
/// \brief Defines the data::candidate_topic_t struct.
#ifndef DATA___CANDIDATE_TOPIC_H
#define DATA___CANDIDATE_TOPIC_H

#include <message_introspection/definition_tree.h>

namespace data {

/// \brief A candidate topic for pulling candidate fields from.
struct candidate_topic_t
{
    /// \brief The name of the candidate topic.
    std::string topic_name;
    /// \brief The definition tree of the topic's message.
    message_introspection::definition_tree_t definition_tree;
};

}

#endif
