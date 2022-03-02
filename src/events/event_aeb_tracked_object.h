
#ifndef PROPHESEE_CORE_EVENT_AEB_TRACKED_OBJECT_H
#define PROPHESEE_CORE_EVENT_AEB_TRACKED_OBJECT_H

#include "events/events.h"

/// Class representing an object tracked on the ground plane for Automatic Emergency Braking (AEB). The x, y & p member
/// variables (defined in the base class Event2d) are not used.
class EventAEBTrackedObject : public Event2d {
public:
    unsigned int object_id;   ///< Unique object ID linking the data to the corresponding object.
    unsigned int class_id;    ///< Predicted class ID for the object.
    float class_confidence;   ///< Confidence on the predicted class ID.
    float bbx, bby, bbw, bbh; ///< Bounding-box for the last observation of the object in the image (in pixels).
    float xctr;    ///< X coordinate of the center of the object (in meters on the ground plane, relatively to the car
                   ///< reference point, positive/negative respectively meaning in-front-of/behind the car).
    float yctr;    ///< Y coordinate of the center of the object (in meters on the ground plane, relatively to the car
                   ///< reference point, positive/negative respectively meaning on the left/right of the car).
    float radius;  ///< Radius (size) of the object (in meters on the ground plane).
    float vx, vy;  ///< Speed of the object along the X and Y axis (relatively to the car reference point, in meters per
                   ///< second).
    float heading; ///< Angle as defined by "atan2(vy,vx)", representing the direction of motion of the object (in
                   ///< degrees). Warning: We have to remove the component of the speed of our own car to vx if we
                   ///< want this value to match the expected RelYaw from the CAN
    float azimuth; ///< Angle as defined by "atan2(yctr,xctr)" (in degrees).
    float time_to_impact;      ///< Time to impact with the object (in seconds).
    float yctr_at_impact;      ///< Prediction of the yctr of the object at the time of impact.
    bool has_collision_course; ///< Flag indicating whether the target is on a collision course (time_to_impact>=0 &&
                               ///< std::abs(yctr_at_impact)<SELF_RADIUS) or not.
    float collision_course_confidence; ///< confidence on the collision course
    enum class GroundPlanePositionStatus {
        Unknown,
        OK,
        AboveHorizon
    } ground_plane_position_status; ///< status of road plane position computation

    // Variances on the variables above
    float xctr_var = 0, yctr_var = 0, radius_var = 0;
    float vx_var = 0, vy_var = 0, heading_var = 0;
    float azimuth_var        = 0;
    float time_to_impact_var = 0, yctr_at_impact_var = 0;

    inline EventAEBTrackedObject() : Event2d() {}
    inline EventAEBTrackedObject(timestamp t, unsigned int object_id, unsigned int class_id, float class_confidence,
                                 float bbx, float bby, float bbw, float bbh) :
        Event2d(0, 0, 0, t),
        object_id(object_id),
        class_id(class_id),
        class_confidence(class_confidence),
        bbx(bbx),
        bby(bby),
        bbw(bbw),
        bbh(bbh),
        xctr(0.f),
        yctr(0.f),
        radius(0.f),
        vx(0.f),
        vy(0.f),
        heading(0.f),
        azimuth(0.f),
        time_to_impact(0.f),
        has_collision_course(false),
        collision_course_confidence(0.f),
        ground_plane_position_status(GroundPlanePositionStatus::Unknown) {}

    /// Destructor
    inline ~EventAEBTrackedObject() {}

    /// Write EventAEBTrackedObject in buffer
    inline void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer             = static_cast<RawEvent *>(buf);
        buffer->ts                   = t;
        buffer->object_id            = object_id;
        buffer->class_id             = class_id;
        buffer->class_confidence     = class_confidence;
        buffer->bbx                  = bbx;
        buffer->bby                  = bby;
        buffer->bbw                  = bbw;
        buffer->bbh                  = bbh;
        buffer->xctr                 = xctr;
        buffer->xctr_var             = xctr_var;
        buffer->yctr                 = yctr;
        buffer->yctr_var             = yctr_var;
        buffer->radius               = radius;
        buffer->radius_var           = radius_var;
        buffer->vx                   = vx;
        buffer->vx_var               = vx_var;
        buffer->vy                   = vy;
        buffer->vy_var               = vy_var;
        buffer->heading              = heading;
        buffer->heading_var          = heading_var;
        buffer->azimuth              = azimuth;
        buffer->azimuth_var          = azimuth_var;
        buffer->time_to_impact       = time_to_impact;
        buffer->time_to_impact_var   = time_to_impact_var;
        buffer->yctr_at_impact       = yctr_at_impact;
        buffer->yctr_at_impact_var   = yctr_at_impact_var;
        buffer->has_collision_course = has_collision_course;
    }

    inline static EventAEBTrackedObject read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        EventAEBTrackedObject ev;
        ev.t                    = buffer->ts;
        ev.object_id            = buffer->object_id;
        ev.class_id             = buffer->class_id;
        ev.class_confidence     = buffer->class_confidence;
        ev.bbx                  = buffer->bbx;
        ev.bby                  = buffer->bby;
        ev.bbw                  = buffer->bbw;
        ev.bbh                  = buffer->bbh;
        ev.xctr                 = buffer->xctr;
        ev.xctr_var             = buffer->xctr_var;
        ev.yctr                 = buffer->yctr;
        ev.yctr_var             = buffer->yctr_var;
        ev.radius               = buffer->radius;
        ev.radius_var           = buffer->radius_var;
        ev.vx                   = buffer->vx;
        ev.vx_var               = buffer->vx_var;
        ev.vy                   = buffer->vy;
        ev.vy_var               = buffer->vy_var;
        ev.heading              = buffer->heading;
        ev.heading_var          = buffer->heading_var;
        ev.azimuth              = buffer->azimuth;
        ev.azimuth_var          = buffer->azimuth_var;
        ev.time_to_impact       = buffer->time_to_impact;
        ev.time_to_impact_var   = buffer->time_to_impact_var;
        ev.yctr_at_impact       = buffer->yctr_at_impact;
        ev.yctr_at_impact_var   = buffer->yctr_at_impact_var;
        ev.has_collision_course = buffer->has_collision_course;
        return ev;
    }

    inline timestamp get_last_2d_update() const {
        return t;
    }

    inline timestamp get_last_3d_update() const {
        // We don't hold here the 3d update, therefore we
        // return the 2d one.
        return t;
    }

    inline void set_2d_time_update(timestamp t) {
        this->t = t;
    }

    inline void set_3d_time_update(timestamp t) {
        // Do nothing as we don't hold the 3d time update.
    }

    void debug_display(std::ostream &stream) const {
        stream << "timestamp: " << t << "   "
               << "object_id: " << object_id << "   "
               << "class_id: " << class_id << "   "
               << "class_confidence: " << class_confidence << "   "
               << "bbx: " << bbx << "   "
               << "bby: " << bby << "   "
               << "bbw: " << bbw << "   "
               << "bbh: " << bbh << "   "
               << "xctr: " << xctr << "   "
               << "yctr: " << yctr << "   "
               << "radius: " << radius << "   "
               << "vx: " << vx << "   "
               << "vy: " << vy << "   "
               << "heading: " << heading << "   "
               << "azimuth: " << azimuth << "   "
               << "time_to_impact: " << time_to_impact << "   "
               << "yctr_at_impact: " << yctr_at_impact << "   "
               << "has_collision_course: " << has_collision_course << "   "
               << "collision_course_confidence: " << collision_course_confidence << "   "
               << "xctr_var: " << xctr_var << "   "
               << "yctr_var: " << yctr_var << "   "
               << "radius_var: " << radius_var << "   "
               << "vx_var: " << vx_var << "   "
               << "vy_var: " << vy_var << "   "
               << "heading_var: " << heading_var << "   "
               << "azimuth_var: " << azimuth_var << "   "
               << "time_to_impact_var: " << time_to_impact_var << "   "
               << "yctr_at_impact_var: " << yctr_at_impact_var;
    }

    /// Structure of size 704 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        uint32_t object_id;
        uint32_t class_id;
        float class_confidence;
        float bbx;
        float bby;
        float bbw;
        float bbh;
        float xctr;
        float xctr_var;
        float yctr;
        float yctr_var;
        float radius;
        float radius_var;
        float vx;
        float vx_var;
        float vy;
        float vy_var;
        float heading;
        float heading_var;
        float azimuth;
        float azimuth_var;
        float time_to_impact;
        float time_to_impact_var;
        float yctr_at_impact;
        float yctr_at_impact_var;
        uint8_t has_collision_course;
        float collision_course_confidence;
        // ground_plane_position_status voluntarily not in RawEvent
    });
};

PROPHESEE_DEFINE_EVENT_TRAIT(EventAEBTrackedObject, 154, "AEBTrackedObject")

#endif // PROPHESEE_CORE_EVENT_AEB_TRACKED_OBJECT_H
