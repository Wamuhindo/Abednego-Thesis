
#ifndef PROPHESEE_CORE_EVENTS_3D_H
#define PROPHESEE_CORE_EVENTS_3D_H

#include "events/event_traits.h"
#include "events/events.h"

namespace Prophesee {

class Event3dDisparity : public Event3d {
public:
    Event3dDisparity() = default;
    Event3dDisparity(float x, float y, float z, float d, short p, timestamp t) :
        Event3d(Event2d(0, 0, p, t), x, y, z),
        d_(d) {}
    float d_;

    /**
     * Destructor
     */
    inline ~Event3dDisparity() {}

    /**
     * Write Event3dDisparity in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float x_;
        float y_;
        float z_;
        float d_;
    });
};

class EventStereoMatch : public Event2d {
public:
    EventStereoMatch(float xl, float yl, float xr, float yr, short p, timestamp t, float d) :
        Event2d(xl, yl, p, t),
        xl_(xl),
        yl_(yl),
        xr_(xr),
        yr_(yr),
        d_(d) {}

    EventStereoMatch() {}

    float xl_ = 0;
    float yl_ = 0;
    float xr_ = 0;
    float yr_ = 0;
    float d_  = 0;

    /**
     * Write Event3dDisparity in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float xl_;
        float yl_;
        float xr_;
        float yr_;
        float d_;
    });
};

void Event3dDisparity::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = (RawEvent *)buf;
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->x_       = x_;
    buffer->y_       = y_;
    buffer->z_       = z_;
    buffer->d_       = d_;
}

void EventStereoMatch::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = (RawEvent *)buf;
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->xl_      = xl_;
    buffer->yl_      = yl_;
    buffer->xr_      = xr_;
    buffer->yr_      = yr_;
    buffer->d_       = d_;
}

} // namespace Prophesee

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::Event3dDisparity, 10, "Disparity")
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::EventStereoMatch, 11, "StereoMatch")

#endif // PROPHESEE_CORE_EVENTS_3D_H
