
#ifndef PROPHESEE_CORE_EVENT_TRAITS_H
#define PROPHESEE_CORE_EVENT_TRAITS_H

#include <cstddef>

/// \brief Template function used to ensure id uniqueness across events
namespace Prophesee {
namespace Detail {
template<std::size_t Id>
constexpr std::size_t unique_id();
} // namespace Detail
} // namespace Prophesee

#define PROPHESEE_DEFINE_EVENT_TRAIT(type_, id_, name_)             \
    /* this will fail to compile if the id has already been used */ \
    namespace Prophesee {                                           \
    namespace Detail {                                              \
    template<>                                                      \
    constexpr std::size_t unique_id<id_>() {                        \
        return id_;                                                 \
    }                                                               \
    }                                                               \
    }                                                               \
                                                                    \
    namespace Prophesee {                                           \
    template<>                                                      \
    struct event_traits<type_> {                                    \
        typedef type_ type;                                         \
        static constexpr const char *name() {                       \
            return name_;                                           \
        }                                                           \
        static constexpr unsigned char id() {                       \
            return id_;                                             \
        }                                                           \
        static constexpr unsigned char size() {                     \
            return sizeof(type_::RawEvent);                         \
        }                                                           \
    };                                                              \
    }

namespace Prophesee {

///
/// \brief Trait class that describes each event property (id, size, name, etc.)
///
/// \tparam EventType : the type of event for which the trait is defined
///
template<typename EventType>
struct event_traits;

///
/// \brief Convenience function to get an event name
///
/// \tparam EventType : type of event for which the name is requested
/// \return the name of the event
///
template<typename EventType>
constexpr const char *get_event_name() {
    return event_traits<EventType>::name();
}

///
/// \brief Convenience function to get an event id
///
/// \tparam EventType : type of event for which the id is requested
/// \return the id of the event
///
template<typename EventType>
constexpr unsigned char get_event_id() {
    return event_traits<EventType>::id();
}

///
/// \brief Convenience function to get an event size
///
/// \tparam EventType : type of event for which the size is requested
/// \return the size of the event in bytes
///
template<typename EventType>
constexpr unsigned char get_event_size() {
    return event_traits<EventType>::size();
}

} /* namespace Prophesee */

#endif /* PROPHESEE_CORE_EVENT_TRAITS_H */
