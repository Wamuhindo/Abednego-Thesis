
#ifndef PROPHESEE_CORE_EVENT_TEMPERATURE_MONITOR_H
#define PROPHESEE_CORE_EVENT_TEMPERATURE_MONITOR_H

#include "events/event2d.h"
#include "utils/float_10_12_helper.h"

namespace Prophesee {

class EventMonitorTemperature : public Event2d {
public:
    EventMonitorTemperature() {}

    EventMonitorTemperature(timestamp t, short s, bool over_alarm, bool user_alarm, float v) :
        Event2d(0, 0, 0, t),
        source(s),
        over_temp_alarm(over_alarm),
        user_temp_alarm(user_alarm),
        value(v) {}

    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        buffer->ts       = t - origin;

        buffer->source               = source;
        buffer->over_temp_alarm      = over_temp_alarm;
        buffer->user_temp_alarm      = user_temp_alarm;
        buffer->temp_10_dot_12_float = Prophesee::Float_10_12_Helper::encode(value);
    }

    static EventMonitorTemperature read_event_v1(void *buf, const timestamp &delta_ts) {
        return EventMonitorTemperature::read_event(buf, delta_ts);
    }

    static EventMonitorTemperature read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        float value      = Prophesee::Float_10_12_Helper::toFloat(buffer->temp_10_dot_12_float);
        return EventMonitorTemperature(buffer->ts + delta_ts, buffer->source, buffer->over_temp_alarm,
                                       buffer->user_temp_alarm, value);
    }

    static size_t get_raw_event_size() {
        return sizeof(RawEvent);
    }

public:
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int temp_10_dot_12_float : 22;
        unsigned int over_temp_alarm : 1;
        unsigned int user_temp_alarm : 1;
        unsigned int source : 4;
    });
    short source;
    uint8_t over_temp_alarm;
    uint8_t user_temp_alarm;
    float value;
};

} /* namespace Prophesee */

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::EventMonitorTemperature, 53, "MonitorTemperature")

#endif /* PROPHESEE_CORE_EVENT_TEMPERATURE_MONITOR_H */
