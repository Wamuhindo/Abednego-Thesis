
#ifndef PROPHESEE_CORE_EVENT2D_H
#define PROPHESEE_CORE_EVENT2D_H

#include <cstdint>
#include <vector>
#include <iterator>
#include <type_traits>
#include <iostream>

#include "utils/struct_pack.h"
#include "utils/timestamp.h"
#include "events/event_traits.h"

namespace Prophesee {

/// \brief Class representing basic 2d events:
///     - Contrast Detection -CD- event
///     - Exposure Measurement -EM- event
class Event2d {
public:
    /// \brief Column position in the sensor at which the event happened
    unsigned short x;

    /// \brief Row position in the sensor at which the event happened
    unsigned short y;

    /// \brief Polarity, whose value depends on the type of the event (CD or EM)
    ///
    /// - In case of CD event: polarity representing the change of contrast
    ///     - 1: a positive contrast change
    ///     - 0: a negative contrast change
    /// - In case of EM event: polarity representing the exposure measurement type
    ///     - 1: EM high i.e. the exposure measurement begins
    ///     - 0: EM low i.e. the exposure measurement ends
    short p;

    /// \brief Timestamp at which the event happened in &micro;s
    timestamp t;

    /// \brief Default constructor
    Event2d() = default;

    /// \brief Constructor
    /// @cond DO_NOT_SHOW_IN_DOC
    /// @endcond
    /// @param x : column position of the event in the sensor
    /// @param y : row position of the event in the sensor
    /// @param p : polarity specialising the event
    /// @param t : timestamp of the event, in &micro;s
    inline Event2d(unsigned short x, unsigned short y, short p, timestamp t) : x(x), y(y), p(p), t(t) {}

    /// @cond DO_NOT_SHOW_IN_DOC

    // Begin function shifted returning class Event2d
    inline Event2d shifted(timestamp dt) {
        return Event2d(x, y, p, t + dt);
    }
    // End function shifted

    // Begin function operator< returning _Bool
    inline bool operator<(const Event2d &e) const {
        return t < e.t;
    }
    // End function operator<

    // Begin function operator<= returning _Bool
    inline bool operator<=(const Event2d &e) const {
        return t <= e.t;
    }
    // End function operator<=

    // Begin function operator> returning _Bool
    inline bool operator>(const Event2d &e) const {
        return t > e.t;
    }
    // End function operator>

    // Begin function operator>= returning _Bool
    inline bool operator>=(const Event2d &e) const {
        return t >= e.t;
    }
    // End function operator>=

    // Begin function operator<< returning int &
    friend std::ostream &operator<<(std::ostream &output, const Event2d &e) {
        output << "Event2d: (";
        output << (int)e.x << ", " << (int)e.y << ", ";
        output << (int)e.p << ", " << e.t;
        output << ")";
        return output;
    }
    // End function operator<<

    /**
     * Read Event2d (old format) from buffer
     */
    static Event2d read_event_v1(void *buf, const timestamp &delta_ts = 0) {
        RawEventV1 *buffer = static_cast<RawEventV1 *>(buf);
        return Event2d(buffer->x, buffer->y, buffer->p, buffer->ts + delta_ts);
    }

    /**
     * Read event 2d from buffer
     */
    static Event2d read_event(void *buf, const timestamp &delta_ts = 0) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        return Event2d(buffer->x, buffer->y, buffer->p, buffer->ts + delta_ts);
    }

    /**
     * Write Event2d in buffer
     */
    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        buffer->ts       = t - origin;
        buffer->x        = x;
        buffer->y        = y;
        buffer->p        = p;
    }

    /**
     * Get the size of the RawEvent
     */
    //    static size_t get_raw_event_size();

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int padding : 14;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
    });

    /// @endcond
};

/// \brief Class representing basic 2d CD (Contrast Detection) events:
class Event2dTD : public Event2d {
public:
    /// \brief Default constructor
    Event2dTD() = default;

    /// \brief Constructor from Event2d
    inline Event2dTD(const Event2d &ev) : Event2d(ev) {}
    using Event2d::Event2d;
};
/// @cond DO_NOT_SHOW_IN_DOC
using Event2dTDLeft = Event2dTD;
/// @endcond

/// \brief Class representing basic 2d EM (Exposure Measurement) events:
class Event2dAPS : public Event2d {
public:
    /// \brief Default constructor
    Event2dAPS() = default;

    /// \brief Constructor from Event2d
    inline Event2dAPS(const Event2d &ev) : Event2d(ev) {}
    using Event2d::Event2d;
};
/// @cond DO_NOT_SHOW_IN_DOC
using Event2dAPSLeft = Event2dAPS;
/// @endcond

/// @cond DO_NOT_SHOW_IN_DOC
class Event2dTDRight : public Event2d {
public:
    Event2dTDRight() = default;
    inline Event2dTDRight(const Event2d &ev) : Event2d(ev) {}
    using Event2d::Event2d;
};

class Event2dAPSRight : public Event2d {
public:
    Event2dAPSRight() = default;
    inline Event2dAPSRight(const Event2d &ev) : Event2d(ev) {}
    using Event2d::Event2d;
};

/// @endcond
} /* namespace Prophesee */

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2d, 0, "TD/APS")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dTD, 12, "CD")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dAPS, 13, "EM")

namespace std {
// when calling std::copy with a back_insert_iterator<vector>, some implementations of the STL
// do the right thing and others do not.
// this overload of std::copy is defined to make sure that the most efficient implementation
// is always used
template<typename InputIterator, typename EventType>
typename enable_if<is_base_of<Prophesee::Event2d, EventType>::value,
                   std::back_insert_iterator<std::vector<EventType>>>::type
    copy(InputIterator begin, InputIterator end, std::back_insert_iterator<std::vector<EventType>> d_begin,
         std::forward_iterator_tag * =
             static_cast<typename std::iterator_traits<InputIterator>::iterator_category *>(0)) {
    struct container_exposer : public std::back_insert_iterator<std::vector<EventType>> {
        using std::back_insert_iterator<std::vector<EventType>>::container;
    };
    std::vector<EventType> *c = static_cast<container_exposer &>(d_begin).container;
    c->insert(c->end(), begin, end);
    return std::back_inserter(*c);
}
} // namespace std

#endif /* PROPHESEE_CORE_EVENT2D_H */
