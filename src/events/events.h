
#ifndef PROPHESEE_CORE_EVENTS_H
#define PROPHESEE_CORE_EVENTS_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <map>

#include "utils/timestamp.h"
#include "events/event2d.h"
#include "events/event_camera_motion.h"

/** \todo Comments events.h
 *
 */
using Event2d   = Prophesee::Event2d;
using timestamp = Prophesee::timestamp;

class Event2dSubPix : public Event2d {
public:
    float x_f_, y_f_;

    // Begin function Event2dSubPix returning void
    inline Event2dSubPix() : Event2d() {}
    // End function Event2dSubPix
    // Begin function Event2dSubPix returning void
    inline Event2dSubPix(float x, float y, short p, timestamp t) : Event2d(floorf(x), floorf(y), p, t) {
        x_f_ = x;
        y_f_ = y;
    }
    // End function Event2dSubPix

    /**
     * Destructor
     */
    inline ~Event2dSubPix();
    /**
     * Write Event2dSubPix in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int padding : 14;
        float x_f;
        float y_f;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float x_f;
        float y_f;
    });
};

class Event2dVec : public Event2d {
public:
    float vx_, vy_;

    // Begin function Event2dVec returning void
    inline Event2dVec() : Event2d(), vx_{0}, vy_{0} {}
    // End function Event2dVec
    // Begin function Event2dVec returning void
    inline Event2dVec(unsigned short x, unsigned short y, float vx, float vy, timestamp t) : Event2d(x, y, 0, t) {
        vx_ = vx;
        vy_ = vy;
    }
    inline Event2dVec(unsigned short x, unsigned short y, short p, float vx, float vy, timestamp t) :
        Event2d(x, y, p, t) {
        vx_ = vx;
        vy_ = vy;
    }
    inline Event2dVec(const Event2dVec &ev) : Event2dVec(ev.x, ev.y, ev.p, ev.vx_, ev.vy_, ev.t) {}
    // End function Event2dVec

    /**
     * Destructor
     */
    inline ~Event2dVec();
    /**
     * Write Event2dVec in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int padding : 14;
        float vx;
        float vy;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float vx;
        float vy;
    });

    /// Read Event from buffer
    inline static Event2dVec read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        return Event2dVec(buffer->x, buffer->y, buffer->p, buffer->vx, buffer->vy, buffer->ts + delta_ts);
    }

    inline static Event2dVec read_event_v1(void *buf, const timestamp &delta_ts) {
        RawEventV1 *buffer = static_cast<RawEventV1 *>(buf);
        return Event2dVec(buffer->x, buffer->y, buffer->p, buffer->vx, buffer->vy, buffer->ts + delta_ts);
    }
};

class Event2dEllipse : public Event2dSubPix {
public:
    float a_, b_, alpha_;

    // Begin function Event2dEllipse returning void
    inline Event2dEllipse() : Event2dSubPix() {}
    // End function Event2dEllipse
    // Begin function Event2dEllipse returning void
    inline Event2dEllipse(float x, float y, short p, float a, float b, float alpha, timestamp t) :
        Event2dSubPix(x, y, p, t) {
        a_     = a;
        b_     = b;
        alpha_ = alpha;
    }
    // End function Event2dEllipse

    /**
     * Destructor
     */
    inline ~Event2dEllipse();

    /**
     * Write Event2dEllipse in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int padding : 14;
        float x_f;
        float y_f;
        float a;
        float b;
        float alpha;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float x_f;
        float y_f;
        float a;
        float b;
        float alpha;
    });
};

class Event2dGray : public Event2d {
public:
    timestamp ts_start_;
    float gray_;
    short type_;

    // Begin function Event2dGray returning void
    inline Event2dGray() : Event2d() {}
    // End function Event2dGray
    // Begin function Event2dGray returning void
    inline Event2dGray(unsigned short x, unsigned short y, short p, timestamp t, timestamp ts_start, float gray_level,
                       short type) :
        Event2d(x, y, p, t),
        ts_start_(ts_start),
        gray_(gray_level),
        type_(type) {}
    // End function Event2dGray

    /**
     * Destructor
     */
    inline ~Event2dGray() = default;

    /**
     * Write Event2dGray in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int type : 14;
        uint32_t ts_start;
        float gray;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 1;
        unsigned int type : 3;
        uint32_t ts_start;
        float gray;
    });
};

struct Event2dGraySimple {
    unsigned short x;
    unsigned short y;
    float gray;
    timestamp t;

    inline Event2dGraySimple() {}
    inline Event2dGraySimple(unsigned short x, unsigned short y, timestamp t, float gray) :
        x(x),
        y(y),
        gray(gray),
        t(t) {}

    inline void write_event(void *buf, timestamp origin) const {}

    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        float gray;
    });
};

class EventAddr : public Event2d {
public:
    unsigned int addr_;

    // Begin function EventAddr returning void
    inline EventAddr() : Event2d() {}
    // End function EventAddr
    // Begin function EventAddr returning void
    inline EventAddr(timestamp t, unsigned int addr) : Event2d(0, 0, 0, t) {
        addr_ = addr;
    }
    // End function EventAddr

    /**
     * Destructor
     */
    inline ~EventAddr();
    /**
     * Write EventAddr in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        uint32_t addr;
    });
};

class Event3d : public Event2d {
public:
    float x_;
    float y_;
    float z_;

    inline Event3d() : Event2d() {}
    inline Event3d(Event2d ev, double x, double y, double z) : Event2d(ev), x_(x), y_(y), z_(z) {}
    // End function EventAddr

    /**
     * Destructor
     */
    inline ~Event3d();

    /**
     * Write Event3d in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        float x_;
        float y_;
        float z_;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float x_;
        float y_;
        float z_;
    });
};

class EventActivity : public Event3d {
public:
    unsigned int id_;
    unsigned int activity_;

    inline EventActivity() : Event3d() {}
    inline EventActivity(Event3d ev, unsigned int id, unsigned int activity) :
        Event3d(ev),
        id_(id),
        activity_(activity) {}

    friend std::ostream &operator<<(std::ostream &output, const EventActivity &e) {
        output << "EventActivity: (";
        output << (int)e.id_ << ", ";
        output << (float)e.x_ << ", " << (float)e.y_ << (float)e.z_ << ", ";
        output << (int)e.p << ", " << e.t << ", ";
        output << (int)e.activity_;
        output << ")";
        return output;
    }

    /**
     * Destructor
     */
    inline ~EventActivity();

    /**
     * Write event 2d in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        unsigned int ts : 32;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        float x_;
        float y_;
        float z_;
        unsigned int id_ : 32;
        unsigned int activity_ : 32;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        unsigned int ts : 32;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        float x_;
        float y_;
        float z_;
        unsigned int id_ : 32;
        unsigned int activity_ : 32;
    });
};

class EventFeature2d : public Event2d {
public:
    unsigned int id_;

    inline EventFeature2d() : Event2d() {}
    inline EventFeature2d(Event2d ev, unsigned int id) : Event2d(ev), id_(id) {}

    /**
     * Destructor
     */
    inline ~EventFeature2d();

    /**
     * Write EventFeature2d in buffer
     */
    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        uint32_t ts;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        uint32_t id_;
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        uint32_t ts;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        uint32_t id_;
    });
};

class Event2dHomography : public Event2d {
public:
    unsigned int id_;
    float a_[9];

    inline Event2dHomography() : Event2d() {}
    inline Event2dHomography(float x_grav, float y_grav, timestamp t, int TrackerId, float a11, float a12, float a21,
                             float a22, float a13, float a23, float a31, float a32, float a33) :
        Event2d(floorf(x_grav), floorf(y_grav), 0, t) {
        id_   = TrackerId;
        a_[0] = a11;
        a_[1] = a12;
        a_[2] = a13;
        a_[3] = a21;
        a_[4] = a22;
        a_[5] = a23;
        a_[4] = a21;
        a_[5] = a22;
        a_[6] = a23;
    }
    inline ~Event2dHomography();

    inline void write_event(void *buf, timestamp origin) const;

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        unsigned int ts : 32;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int padding : 14;
        unsigned int id_ : 32;
        float a_[9];
    });

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        unsigned int ts : 32;
        unsigned int x : 14;
        unsigned int y : 14;
        unsigned int p : 4;
        unsigned int id_ : 32;
        float a_[9];
    });
};

Event2dSubPix::~Event2dSubPix() {}

void Event2dSubPix::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->x_f      = x_f_;
    buffer->y_f      = y_f_;
}

Event2dVec::~Event2dVec() {}

void Event2dVec::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->vx       = vx_;
    buffer->vy       = vy_;
}

Event2dEllipse::~Event2dEllipse() {}

void Event2dEllipse::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->x_f      = x_f_;
    buffer->y_f      = y_f_;
    buffer->a        = a_;
    buffer->b        = b_;
    buffer->alpha    = alpha_;
}

void Event2dGray::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->type     = type_;
    buffer->ts_start = ts_start_;
    buffer->gray     = gray_;
}

EventAddr::~EventAddr() {}

void EventAddr::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->addr     = addr_;
}
Event3d::~Event3d() {}

void Event3d::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = (RawEvent *)buf;
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->x_       = x_;
    buffer->y_       = y_;
    buffer->z_       = z_;
}

EventActivity::~EventActivity() {}

void EventActivity::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer  = static_cast<RawEvent *>(buf);
    buffer->ts        = t - origin;
    buffer->x         = x;
    buffer->y         = y;
    buffer->p         = p;
    buffer->x_        = x_;
    buffer->y_        = y_;
    buffer->z_        = z_;
    buffer->id_       = id_;
    buffer->activity_ = activity_;
}

EventFeature2d::~EventFeature2d() {}

void EventFeature2d::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->id_      = id_;
}

Event2dHomography::~Event2dHomography() {}

void Event2dHomography::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer = static_cast<RawEvent *>(buf);
    buffer->ts       = t - origin;
    buffer->x        = x;
    buffer->y        = y;
    buffer->p        = p;
    buffer->id_      = id_;

    for (int i = 0; i < 9; i++) {
        buffer->a_[i] = a_[i];
    }
}

PROPHESEE_DEFINE_EVENT_TRAIT(Event2dSubPix, 1, "SubPix")
PROPHESEE_DEFINE_EVENT_TRAIT(Event2dVec, 2, "Vec")
PROPHESEE_DEFINE_EVENT_TRAIT(Event2dEllipse, 3, "Ellipse")
PROPHESEE_DEFINE_EVENT_TRAIT(Event2dGray, 4, "Gray")
PROPHESEE_DEFINE_EVENT_TRAIT(EventAddr, 5, "Addr")
PROPHESEE_DEFINE_EVENT_TRAIT(Event3d, 6, "3D")
PROPHESEE_DEFINE_EVENT_TRAIT(EventActivity, 7, "Activity")
PROPHESEE_DEFINE_EVENT_TRAIT(EventFeature2d, 8, "Feature2d")
PROPHESEE_DEFINE_EVENT_TRAIT(Event2dGraySimple, 9, "GraySimple")
PROPHESEE_DEFINE_EVENT_TRAIT(Event2dHomography, 19, "Homography")

#endif // PROPHESEE_CORE_EVENTS_H
