
#ifndef PROPHESEE_CORE_EVENT_EXT_TRIGGER_H
#define PROPHESEE_CORE_EVENT_EXT_TRIGGER_H

#include <cstddef>
#include <cstdint>
#include <iostream>

#include "utils/struct_pack.h"
#include "utils/timestamp.h"
#include "events/event_traits.h"

namespace Prophesee {

/// \brief Class representing an external trigger event.
class EventExtTrigger {
public:
    EventExtTrigger() = default;

    /// \param p polarity of the event
    /// \param t timestamp of the event, in &micro;s
    /// \param id id of the external trigger
    inline EventExtTrigger(short p, timestamp t, short id) : p(p), t(t), id(id) {}

    /// Write EventExtTrigger in buffer
    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        buffer->ts       = t - origin;
        buffer->p        = p;
        buffer->id       = id;
        buffer->pad1     = 0;
    }

    /// Read EventExtTrigger encoded in an old format from buffer
    static EventExtTrigger read_event_v1(void *buf, const timestamp &delta_ts) {
        return EventExtTrigger::read_event(buf, delta_ts);
    }

    /// Read EventExtTrigger from buffer
    static EventExtTrigger read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        return EventExtTrigger(buffer->p, buffer->ts + delta_ts, buffer->id);
    }

    /// Returns raw event size
    static size_t get_raw_event_size() {
        return sizeof(RawEvent);
    }

    /// Begin function shifted returning class EventExtTrigger
    inline EventExtTrigger shifted(timestamp dt) {
        return EventExtTrigger(p, t + dt, id);
    }

    /// Event timestamp comparison operator.
    inline bool operator<(const EventExtTrigger &e) const {
        return t < e.t;
    }

    /// Event timestamp comparison operator.
    inline bool operator<=(const EventExtTrigger &e) const {
        return t <= e.t;
    }

    /// Event timestamp comparison operator.
    inline bool operator>(const EventExtTrigger &e) const {
        return t > e.t;
    }

    /// Event timestamp comparison operator.
    inline bool operator>=(const EventExtTrigger &e) const {
        return t >= e.t;
    }

    // Begin function operator<< returning int &
    friend std::ostream &operator<<(std::ostream &output, const EventExtTrigger &e) {
        output << "EventExtTrigger: (";
        output << (int)e.p << ", " << e.t << ", " << e.id;
        output << ")";
        return output;
    }

    /// The raw event format of an external trigger event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int p : 4;
        unsigned int pad1 : 22;
        unsigned int id : 6;
    });

    typedef RawEvent RawEventV1;

    /// Polarity representing the change of contrast (1: positive, 0: negative)
    short p;

    /// Timestamp at which the event happened in &micro;s
    timestamp t;

    /// ID of the external trigger
    short id;
};
} /* namespace Prophesee */

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::EventExtTrigger, 14, "Trigger")

#endif /* PROPHESEE_CORE_EVENT_EXT_TRIGGER_H */
