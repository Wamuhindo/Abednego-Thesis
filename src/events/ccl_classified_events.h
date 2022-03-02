

#ifndef PROPHESEE_CORE_EVENT_CCL_CLASSIFIED_H
#define PROPHESEE_CORE_EVENT_CCL_CLASSIFIED_H

#include "events/events.h"

namespace Prophesee {

// Event definition
class EventCCLClassified : public Event2dVec {
public:
    EventCCLClassified() {}
    EventCCLClassified(const Event2d &ev) : Event2dVec(ev.x, ev.y, ev.p, 0.f, 0.f, ev.t) {}
    EventCCLClassified(const Event2dVec &ev) : Event2dVec(ev) {}
    ~EventCCLClassified() {}
    inline void write_event(void *buf, timestamp origin) const;

    unsigned int id_    = 0;
    int static_dynamic_ = 0;
    float center_x_     = 0.f;
    float center_y_     = 0.f;

    void debug_display(std::ostream &stream) const {
        stream << "timestamp: " << t << "  "
               << "x: " << x << "   "
               << "y: " << y << "   "
               << "p: " << p << "   "
               << "vx_: " << vx_ << "   "
               << "vy_: " << vy_ << "   "
               << "id_: " << id_ << "   "
               << "center_x_: " << center_x_ << "   "
               << "center_y_: " << center_y_ << "   "
               << "static_dynamic_: " << static_dynamic_ << "   ";
    }

    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float vx_;
        float vy_;
        float center_x_;
        float center_y_;
        uint32_t id_;
        int32_t static_dynamic_;
    });

    inline static EventCCLClassified read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        EventCCLClassified ev;
        ev.t               = buffer->ts + delta_ts;
        ev.x               = buffer->x;
        ev.y               = buffer->y;
        ev.p               = buffer->p;
        ev.vx_             = buffer->vx_;
        ev.vy_             = buffer->vy_;
        ev.id_             = buffer->id_;
        ev.center_x_       = buffer->center_x_;
        ev.center_y_       = buffer->center_y_;
        ev.static_dynamic_ = buffer->static_dynamic_;
        return ev;
    }
};

void EventCCLClassified::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer        = (RawEvent *)buf;
    buffer->ts              = t - origin;
    buffer->x               = x;
    buffer->y               = y;
    buffer->p               = p;
    buffer->id_             = id_;
    buffer->vx_             = vx_;
    buffer->vy_             = vy_;
    buffer->center_x_       = center_x_;
    buffer->center_y_       = center_y_;
    buffer->static_dynamic_ = static_dynamic_;
}

} // namespace Prophesee
PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::EventCCLClassified, 201, "EventCCLClassified")
#endif // PROPHESEE_CORE_EVENT_CCL_CLASSIFIED_H
