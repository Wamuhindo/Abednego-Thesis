
#ifndef PROPHESEE_CORE_EVENTS_DISP_FLOW_CCL_H
#define PROPHESEE_CORE_EVENTS_DISP_FLOW_CCL_H

#include "events/events.h"

namespace Prophesee {

// Event definition
class EventDispFlowCCL : public Event2dVec {
public:
    EventDispFlowCCL() {}
    EventDispFlowCCL(const Event2d &ev) : Event2dVec(ev.x, ev.y, ev.p, 0.f, 0.f, ev.t) {}
    EventDispFlowCCL(const Event2dVec &ev) : Event2dVec(ev) {}
    ~EventDispFlowCCL() {}
    inline void write_event(void *buf, timestamp origin) const;

    unsigned int id_                   = 0;
    float disparity_                   = 0;
    float x_from_motion_               = 0;
    float y_from_motion_               = 0;
    float z_from_motion_               = 0;
    float z_from_stereo_               = 0;
    float vz_from_stereo_              = 0;
    float compensated_vx_              = 0;
    float compensated_vy_              = 0;
    float background_vx_               = 0;
    float background_vy_               = 0;
    float freq_                        = 0;
    float center_x_                    = 0;
    float center_y_                    = 0;
    float collision_course_confidence_ = 0;
    int static_dynamic_                = 0;

    void debug_display(std::ostream &stream) const {
        stream << "timestamp: " << t << "  "
               << "x: " << x << "   "
               << "y: " << y << "   "
               << "p: " << p << "   "
               << "vx_: " << vx_ << "   "
               << "vy_: " << vy_ << "   "
               << "id_: " << id_ << "   "
               << "disparity_: " << disparity_ << "   "
               << "x_from_motion_: " << x_from_motion_ << "   "
               << "y_from_motion_: " << y_from_motion_ << "   "
               << "z_from_motion_: " << z_from_motion_ << "   "
               << "z_from_stereo_: " << z_from_stereo_ << "   "
               << "vz_from_stereo_: " << vz_from_stereo_ << "   "
               << "compensated_vx_: " << compensated_vx_ << "   "
               << "compensated_vy_: " << compensated_vy_ << "   "
               << "background_vx_: " << background_vx_ << "   "
               << "background_vy_: " << background_vy_ << "   "
               << "freq_: " << freq_ << "   "
               << "center_x_: " << center_x_ << "   "
               << "center_y_: " << center_y_ << "   "
               << "collision_course_confidence_: " << collision_course_confidence_ << "   "
               << "static_dynamic_: " << static_dynamic_ << "   ";
    }

    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        uint32_t id_;
        float vx_;
        float vy_;
        float motion_x_;
        float motion_y_;
        float motion_z_;
        float d_;
        float stereo_z_;
        float vz_from_stereo_;
        float compensated_vx_;
        float compensated_vy_;
        float background_vx_;
        float background_vy_;
        float freq_;
        float center_x_;
        float center_y_;
        float collision_course_confidence_;
    });

    inline static EventDispFlowCCL read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        EventDispFlowCCL ev;
        ev.t                            = buffer->ts + delta_ts;
        ev.x                            = buffer->x;
        ev.y                            = buffer->y;
        ev.p                            = buffer->p;
        ev.vx_                          = buffer->vx_;
        ev.vy_                          = buffer->vy_;
        ev.id_                          = buffer->id_;
        ev.disparity_                   = buffer->d_;
        ev.x_from_motion_               = buffer->motion_x_;
        ev.y_from_motion_               = buffer->motion_y_;
        ev.z_from_motion_               = buffer->motion_z_;
        ev.z_from_stereo_               = buffer->stereo_z_;
        ev.vz_from_stereo_              = buffer->vz_from_stereo_;
        ev.compensated_vx_              = buffer->compensated_vx_;
        ev.compensated_vy_              = buffer->compensated_vy_;
        ev.background_vx_               = buffer->background_vx_;
        ev.background_vy_               = buffer->background_vy_;
        ev.freq_                        = buffer->freq_;
        ev.center_x_                    = buffer->center_x_;
        ev.center_y_                    = buffer->center_y_;
        ev.collision_course_confidence_ = buffer->collision_course_confidence_;

        ev.static_dynamic_ = 0;

        return ev;
    }
};

void EventDispFlowCCL::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer                     = (RawEvent *)buf;
    buffer->ts                           = t - origin;
    buffer->x                            = x;
    buffer->y                            = y;
    buffer->p                            = p;
    buffer->id_                          = id_;
    buffer->vx_                          = vx_;
    buffer->vy_                          = vy_;
    buffer->motion_x_                    = x_from_motion_;
    buffer->motion_y_                    = y_from_motion_;
    buffer->motion_z_                    = z_from_motion_;
    buffer->d_                           = disparity_;
    buffer->stereo_z_                    = z_from_stereo_;
    buffer->vz_from_stereo_              = vz_from_stereo_;
    buffer->compensated_vx_              = compensated_vx_;
    buffer->compensated_vy_              = compensated_vy_;
    buffer->background_vx_               = background_vx_;
    buffer->background_vy_               = background_vy_;
    buffer->freq_                        = freq_;
    buffer->center_x_                    = center_x_;
    buffer->center_y_                    = center_y_;
    buffer->collision_course_confidence_ = collision_course_confidence_;
}

} // namespace Prophesee

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::EventDispFlowCCL, 200, "DispFlowCCL")

#endif // PROPHESEE_CORE_DISPARITY_AND_FLOW_CCL_EVENTS_ALGORITHM_H
