
#ifndef PROPHESEE_CORE_EVENT_BBOX_H
#define PROPHESEE_CORE_EVENT_BBOX_H

#include <limits>
#include "events/events.h"
#include "events/event_traits.h"

/**
 * Class representing a spatio-temporal bounding-box event.
 * The timestamp of the event (i.e. member variable 't') is by convention at the end of the bounding-box. A positive
 * bounding-box duration (i.e. member variable 'dt') means by convention a duration in the past (i.e. timestamps in
 * [t-dt+1,t] are inside the bounding-box), while a negative bounding-box duration means by convention a duration in the
 * future (i.e. timestamps in [t,t+dt-1] are inside the bounding-box). Concerning the convention about the spatial
 * position, points with u in [x,x+w[ and v in [y,y+h[ are inside the bounding-box, everything else is outside.
 */
class EventBbox : public Event2d {
public:
    enum class OrientationType : int {
        Standard,
        Back,
        Front,
        BackSide,
        FrontSide,
        Side,
        Atypical,
        NumberOfOrientationTypes
    };

    enum class SourceType : int {
        none,
        other,
        detector,
        classifier,
        tracker,
    };

    unsigned int object_id;
    float x_f_; // top left corner
    float y_f_;
    float w;
    float h;
    timestamp dt;
    unsigned int class_id;
    float confidence;
    SourceType source;

    bool occluded;
    OrientationType orientation;

    float dangerous; // Essentially, bounding boxes which are "appearing" fast (from left/right/occlusion/far-way...)

    timestamp time_to_impact; // in microseconds
    float time_to_impact_var;

    inline EventBbox() : Event2d(0, 0, 0, 0) {
        object_id          = 0;
        x_f_               = 0.f;
        y_f_               = 0.f;
        w                  = 0.f;
        h                  = 0.f;
        dt                 = 0;
        class_id           = 0;
        confidence         = 0.f;
        source             = SourceType::none;
        occluded           = false;
        orientation        = OrientationType::Atypical;
        dangerous          = 0.f;
        time_to_impact     = 0;
        time_to_impact_var = 0.f;
    }

    inline EventBbox(Event2d ev, unsigned int object_id, float x_f_, float y_f_, float w, float h, timestamp dt,
                     unsigned int class_id, float confidence, SourceType source = SourceType::none, bool occl = false,
                     OrientationType orient = OrientationType::Standard, float dang = 0.f,
                     timestamp time_to_impact = std::numeric_limits<timestamp>::max(),
                     float time_to_impact_var = std::numeric_limits<float>::max()) :
        Event2d(ev),
        object_id(object_id),
        x_f_(x_f_),
        y_f_(y_f_),
        w(w),
        h(h),
        dt(dt),
        class_id(class_id),
        confidence(confidence),
        source(source),
        occluded(occl),
        orientation(orient),
        dangerous(dang),
        time_to_impact(time_to_impact_var),
        time_to_impact_var(time_to_impact_var) {}

    /**
     * Destructor
     */
    inline ~EventBbox();

    /**
     * Function checking whether a given point is inside the bounding-box.
     */
    inline bool is_inside(int ev_x, int ev_y, timestamp ev_ts) const {
        if (dt < 0) // "pre-detected" bounding-box, i.e. bounding-box with duration in the future
            return ev_x >= x && ev_x < x + w && ev_y >= y && ev_y < y + h && ev_ts >= t && ev_ts <= t - dt + 1;
        return ev_x >= x && ev_x < x + w && ev_y >= y && ev_y < y + h && ev_ts >= t - dt + 1 && ev_ts <= t;
    }

    /**
     * Function checking whether a given subpixel point is inside the bounding-box.
     */
    inline bool is_inside_subpix(float ev_x, float ev_y, timestamp ev_ts) const {
        if (dt < 0) // "pre-detected" bounding-box, i.e. bounding-box with duration in the future
            return ev_x >= x_f_ && ev_x < x_f_ + w && ev_y >= y_f_ && ev_y < y_f_ + h && ev_ts >= t &&
                   ev_ts <= t - dt + 1;
        return ev_x >= x_f_ && ev_x < x_f_ + w && ev_y >= y_f_ && ev_y < y_f_ + h && ev_ts >= t - dt + 1 && ev_ts <= t;
    }

    /**
     * Write EventBbox in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /**
     * Read EventBbox from a buffer
     */
    inline static EventBbox read_event_v1(void *buf, const timestamp &delta_ts);
    inline static EventBbox read_event(void *buf, const timestamp &delta_ts);

    inline const float &get_bbox_x() const {
        return x_f_;
    }

    inline const float &get_bbox_y() const {
        return y_f_;
    }

    inline const float &get_bbox_w() const {
        return w;
    }

    inline const float &get_bbox_h() const {
        return h;
    }

    /// @return the intersection area between current bbox and bbox 2 (in pixel²)
    inline float intersection_area(EventBbox bbox2);
    /// @return the intersection area over curent bbox area
    /// This function will not work with a zero size bbox (w = 0 or h = 0)
    inline float overlapping_ratio(EventBbox bbox2);

    // Begin function operator<<
    friend std::ostream &operator<<(std::ostream &output, const EventBbox &e) {
        output << "EventBbox: ("
               << "t: " << e.t << "   "
               << "p: " << e.p << "   "
               << "object_id: " << e.object_id << "   "
               << "class_id: " << e.class_id << "   "
               << "x_f_: " << e.x_f_ << "   "
               << "y_f_: " << e.y_f_ << "   "
               << "w: " << e.w << "   "
               << "h: " << e.h << "   "
               << "confidence: " << e.confidence << ")";
        return output;
    }
    // End function operator<<

    /// Structure of size 192 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        float x_f_;
        float y_f_;
        unsigned char p;
        uint32_t object_id;
        float w;
        float h;
        uint32_t dt;
        unsigned char class_id;
        float confidence;
        int32_t source;
        float dangerous;
        timestamp time_to_impact;
        float time_to_impact_var;
    });
};

/**
 * Class representing a spatio-temporal bounding-box event with a frequency.
 */
class EventBboxFreq : public EventBbox {
public:
    float freq_;
    inline EventBboxFreq() : EventBbox() {}

    inline EventBboxFreq(Event2d ev, unsigned int object_id, float x_f_, float y_f_, float w, float h, timestamp dt,
                         unsigned int class_id, float confidence, float frequency, SourceType source = SourceType::none,
                         bool occl = false, OrientationType orient = OrientationType::Standard, float dang = 0.f,
                         timestamp time_to_impact = std::numeric_limits<timestamp>::max(),
                         float time_to_impact_var = std::numeric_limits<float>::max()) :
        EventBbox(ev, object_id, x_f_, y_f_, w, h, dt, class_id, confidence, source, occl, orient, dang, time_to_impact,
                  time_to_impact_var),
        freq_(frequency) {}

    /**
     * Destructor
     */
    inline ~EventBboxFreq();

    /**
     * Write EventBboxFreq in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 192 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        float x_f_;
        float y_f_;
        unsigned char p;
        uint32_t object_id;
        float w;
        float h;
        uint32_t dt;
        unsigned char class_id;
        float confidence;
        int32_t source;
        float dangerous;
        timestamp time_to_impact;
        float time_to_impact_var;
        float freq;
    });
};

EventBbox::~EventBbox() {}

void EventBbox::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer           = static_cast<RawEvent *>(buf);
    buffer->ts                 = t - origin;
    buffer->x_f_               = x_f_;
    buffer->y_f_               = y_f_;
    buffer->p                  = p;
    buffer->object_id          = object_id;
    buffer->w                  = w;
    buffer->h                  = h;
    buffer->dt                 = dt;
    buffer->class_id           = class_id;
    buffer->confidence         = confidence;
    buffer->source             = static_cast<int>(source);
    buffer->dangerous          = dangerous;
    buffer->time_to_impact     = time_to_impact;
    buffer->time_to_impact_var = time_to_impact_var;
}

EventBbox EventBbox::read_event_v1(void *buf, const timestamp &delta_ts) {
    return EventBbox::read_event(buf, delta_ts);
}

EventBbox EventBbox::read_event(void *buf, const timestamp &delta_ts) {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    return EventBbox(Event2d(buffer->x_f_, buffer->y_f_, buffer->p, buffer->ts + delta_ts), buffer->object_id,
                     buffer->x_f_, buffer->y_f_, buffer->w, buffer->h, buffer->dt, buffer->class_id, buffer->confidence,
                     static_cast<SourceType>(buffer->source), false, OrientationType::Standard, 0.f,
                     buffer->time_to_impact, buffer->time_to_impact_var);
}

float EventBbox::intersection_area(EventBbox bbox2) {
    // lets compute the intersection bbox
    float xul    = std::max(x_f_, bbox2.x_f_);               // x upper left corner
    float yul    = std::max(y_f_, bbox2.y_f_);               // y upper left corner
    float xbr    = std::min(x_f_ + w, bbox2.x_f_ + bbox2.w); // x bottom right corner
    float ybr    = std::min(y_f_ + h, bbox2.y_f_ + bbox2.h); // y bottom right corner
    float width  = std::max(0.0f, xbr - xul);
    float height = std::max(0.0f, ybr - yul);
    return (width * height);
}

float EventBbox::overlapping_ratio(EventBbox bbox2) {
    // overlap = intersection area / bbox1 area
    return (this->intersection_area(bbox2) / ((w * h) + 0.000001));
}

EventBboxFreq::~EventBboxFreq() {}

void EventBboxFreq::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer           = static_cast<RawEvent *>(buf);
    buffer->ts                 = t - origin;
    buffer->x_f_               = x_f_;
    buffer->y_f_               = y_f_;
    buffer->p                  = p;
    buffer->object_id          = object_id;
    buffer->w                  = w;
    buffer->h                  = h;
    buffer->dt                 = dt;
    buffer->class_id           = class_id;
    buffer->confidence         = confidence;
    buffer->source             = static_cast<int>(source);
    buffer->dangerous          = dangerous;
    buffer->time_to_impact     = time_to_impact;
    buffer->time_to_impact_var = time_to_impact_var;
    buffer->freq               = freq_;
}

PROPHESEE_DEFINE_EVENT_TRAIT(EventBbox, 16, "Bbox")
PROPHESEE_DEFINE_EVENT_TRAIT(EventBboxFreq, 17, "BboxFreq")

#endif // PROPHESEE_CORE_EVENT_BBOX_H
