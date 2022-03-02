
#ifndef PROPHESEE_CORE_FREQUENCY_EVENT_H
#define PROPHESEE_CORE_FREQUENCY_EVENT_H

#include "events/events.h"
#include "events/event_traits.h"

namespace Prophesee {

/**
 * @brief Event2dFreq is an Event2d extended with a frequency information.
 */
class Event2dFreq : public Event2d {
public:
    using type = float;

    type freq_;

    Event2dFreq() : Event2d() {}
    Event2dFreq(unsigned short x, unsigned short y, short p, timestamp t, float freq) :
        Event2d(x, y, p, t),
        freq_(freq) {}
    Event2dFreq(const Event2d &ev, float freq) : Event2d(ev.x, ev.y, ev.p, ev.t), freq_(freq) {}
    ~Event2dFreq() {}

    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        buffer->ts       = t - origin;
        buffer->x        = x;
        buffer->y        = y;
        buffer->p        = p;
        buffer->freq     = freq_;
    }

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float freq;
    });
};

} // namespace Prophesee

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dFreq, 18, "Freq")
#endif // PROPHESEE_CORE_FREQUENCY_EVENT_H
