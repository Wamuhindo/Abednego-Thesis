
#ifndef PROPHESEE_CORE_PERIOD_EVENT_H
#define PROPHESEE_CORE_PERIOD_EVENT_H

#include "events/events.h"
#include "events/event_traits.h"

namespace Prophesee {
/**
 * @brief Event2dPeriod is an Event2d extended with a period information.
 */
template<typename T = timestamp>
class Event2dPeriod : public Event2dTD {
public:
    using type = T;

    T period_;

    Event2dPeriod() : Event2dTD() {}
    Event2dPeriod(unsigned short x, unsigned short y, short p, timestamp t, T period) :
        Event2dTD(x, y, p, t),
        period_(period) {}
    Event2dPeriod(const Event2d &ev, T period) : Event2dTD(ev.x, ev.y, ev.p, ev.t), period_(period) {}
    ~Event2dPeriod() {}

    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        buffer->ts       = t - origin;
        buffer->x        = x;
        buffer->y        = y;
        buffer->p        = p;
        buffer->period   = period_;
    }

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        uint32_t period;
    });
};

template<typename T = timestamp>
class Event2dStringPeriod : public Event2dPeriod<T> {
public:
    int string_{-1};
    int fret_{-1};

    Event2dStringPeriod() : Event2dPeriod<T>() {}
    Event2dStringPeriod(short p, timestamp t, int string, T period, int fret) :
        Event2dPeriod<T>(0, 0, p, t, period),
        string_(string),
        fret_(fret) {}
    Event2dStringPeriod(unsigned short x, unsigned short y, short p, timestamp t, int string, T period, int fret) :
        Event2dPeriod<T>(x, y, p, t, period),
        string_(string),
        fret_(fret) {
        this->period_ = period;
    }
    Event2dStringPeriod(const Event2d &ev, int string, T period, int fret) :
        Event2dPeriod<T>(ev, period),
        string_(string),
        fret_(fret) {}

    ~Event2dStringPeriod() {}

    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        buffer->ts       = this->t - origin;
        buffer->x        = this->x;
        buffer->y        = this->y;
        buffer->p        = this->p;
        buffer->period   = this->period_;
        buffer->string   = string_;
        buffer->fret     = fret_;
    }

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        uint32_t period;
        int string;
        int fret;
    });
};

} // namespace Prophesee

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dPeriod<long long>, 70, "Period 64bits")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dPeriod<std::uint32_t>, 74, "Period u 32bits")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dPeriod<std::int32_t>, 71, "Period 32bits")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dPeriod<double>, 72, "Period double")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dPeriod<float>, 73, "Period float")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dStringPeriod<long long>, 80, "String period 64bits")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dStringPeriod<std::uint32_t>, 84, "String period u 32bits")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dStringPeriod<std::int32_t>, 81, "String period 32bits")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dStringPeriod<double>, 82, "String period double")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event2dStringPeriod<float>, 83, "String period float")
#endif // PROPHESEE_CORE_PERIOD_EVENT_H
