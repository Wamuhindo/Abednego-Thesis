
#ifndef PROPHESEE_CORE_EVENT_LIFETIME_H
#define PROPHESEE_CORE_EVENT_LIFETIME_H

#include "events/events.h"
#include "events/event_traits.h"

class Event2dLifetime : public Event2d {
public:
    timestamp lifetime;

    inline Event2dLifetime() : Event2d() {}
    inline Event2dLifetime(const Event2d &ev, timestamp lifetime) : Event2d(ev), lifetime(lifetime) {}

    /**
     * Destructor
     */
    inline ~Event2dLifetime();

    /**
     * Write EventFeature2d in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 96 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        uint32_t lifetime;
    });
};

Event2dLifetime::~Event2dLifetime() {}

void Event2dLifetime::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->lifetime = lifetime;
}

PROPHESEE_DEFINE_EVENT_TRAIT(Event2dLifetime, 151, "Lifetime")

#endif // PROPHESEE_CORE_EVENT_LIFETIME_H
