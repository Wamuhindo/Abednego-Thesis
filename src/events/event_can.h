
#ifndef PROPHESEE_CORE_EVENT_CAN_H
#define PROPHESEE_CORE_EVENT_CAN_H

#include "events/event2d.h"

namespace Prophesee {

class EventCAN : public Event2d {
public:
    EventCAN() {}
    EventCAN(timestamp ts, unsigned int id, unsigned int dlc) :
        Event2d(0, 0, 0, ts),
        id_(id),
        dlc_(dlc),
        recv_sec_(0),
        recv_usec_(0) {}
    EventCAN(const EventCAN &ev) :
        Event2d(ev),
        id_(ev.id_),
        dlc_(ev.dlc_),
        recv_sec_(ev.recv_sec_),
        recv_usec_(ev.recv_usec_) {
        data_[0] = ev.data_[0];
        data_[1] = ev.data_[1];
        data_[2] = ev.data_[2];
        data_[3] = ev.data_[3];
        data_[4] = ev.data_[4];
        data_[5] = ev.data_[5];
        data_[6] = ev.data_[6];
        data_[7] = ev.data_[7];
    }

    inline void write_event(void *buf, timestamp origin) const;

    unsigned int id_;
    unsigned int dlc_;
    long recv_sec_;
    long recv_usec_;
    unsigned char data_[8];

    FORCE_PACK(struct RawEvent {
        unsigned int ts : 32;
        unsigned int id : 32;
        unsigned int dlc : 8;
        unsigned int recv_sec : 32;
        unsigned int recv_usec : 32;
        unsigned char data[8];
    });
};

void EventCAN::write_event(void *buf, timestamp origin) const {
    RawEvent *buffer  = (RawEvent *)buf;
    buffer->ts        = t - origin;
    buffer->id        = id_;
    buffer->dlc       = dlc_;
    buffer->recv_sec  = recv_sec_;
    buffer->recv_usec = recv_usec_;
    buffer->data[0]   = data_[0];
    buffer->data[1]   = data_[1];
    buffer->data[2]   = data_[2];
    buffer->data[3]   = data_[3];
    buffer->data[4]   = data_[4];
    buffer->data[5]   = data_[5];
    buffer->data[6]   = data_[6];
    buffer->data[7]   = data_[7];
}
} // namespace Prophesee
PROPHESEE_DEFINE_EVENT_TRAIT(EventCAN, 153, "CAN")

#endif // PROPHESEE_CORE_EVENT_CAN_H
