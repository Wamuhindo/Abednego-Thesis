
#ifndef PROPHESEE_CORE_EVENT_CAMERA_MOTION_H
#define PROPHESEE_CORE_EVENT_CAMERA_MOTION_H

#include "utils/timestamp.h"
#include "events/event2d.h"

namespace Prophesee {

class EventCameraMotion : public Event2d {
public:
    float omega_x_, omega_y_, omega_z_;
    float v_x_, v_y_, v_z_;

    inline EventCameraMotion() : Event2d() {}
    inline EventCameraMotion(Event2d ev) : Event2d(ev) {
        omega_x_ = 0;
        omega_y_ = 0;
        omega_z_ = 0;
        v_x_     = 0;
        v_y_     = 0;
        v_z_     = 0;
    }
    inline EventCameraMotion(timestamp t, float omega_x, float omega_y, float omega_z, float v_x, float v_y,
                             float v_z) :
        Event2d(0, 0, 0, t) {
        omega_x_ = omega_x;
        omega_y_ = omega_y;
        omega_z_ = omega_z;
        v_x_     = v_x;
        v_y_     = v_y;
        v_z_     = v_z;
    }

    /**
     * Destructor
     */
    inline ~EventCameraMotion();
    /**
     * Write EventAddr in buffer
     */
    void write_event(void *buf, timestamp origin) const;

    /// Read EventExtTrigger encoded in an old format from buffer
    static EventCameraMotion read_event_v1(void *buf, const timestamp &delta_ts);

    /// Read EventExtTrigger from buffer
    static EventCameraMotion read_event(void *buf, const timestamp &delta_ts);

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        float om_x;
        float om_y;
        float om_z;
        float v_x;
        float v_y;
        float v_z;
    });
};

EventCameraMotion::~EventCameraMotion() {}

inline void EventCameraMotion::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = (RawEvent *)buf;
    buffer->ts       = t - origin;
    buffer->om_x     = omega_x_;
    buffer->om_y     = omega_y_;
    buffer->om_z     = omega_z_;
    buffer->v_x      = v_x_;
    buffer->v_y      = v_y_;
    buffer->v_z      = v_z_;
}

inline EventCameraMotion EventCameraMotion::read_event_v1(void *buf, const timestamp &delta_ts) {
    return EventCameraMotion::read_event(buf, delta_ts);
}

inline EventCameraMotion EventCameraMotion::read_event(void *buf, const timestamp &delta_ts) {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    return EventCameraMotion(buffer->ts + delta_ts, buffer->om_x, buffer->om_y, buffer->om_z, buffer->v_x, buffer->v_y,
                             buffer->v_z);
}

} // namespace Prophesee

PROPHESEE_DEFINE_EVENT_TRAIT(EventCameraMotion, 20, "CameraMotion")

#endif // PROPHESEE_CORE_EVENT_CAMERA_MOTION_H
