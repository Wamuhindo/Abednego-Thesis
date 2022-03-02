
#ifndef PROPHESEE_CORE_EVENT_IMU_H
#define PROPHESEE_CORE_EVENT_IMU_H

#include <iostream>

#include "utils/struct_pack.h"
#include "utils/timestamp.h"
#include "events/event_traits.h"

namespace Prophesee {

/// \brief Class representing an IMU event
class EventIMU {
public:
    /// \brief Accelerometer x, y, and z values [g]
    float ax, ay, az;

    /// \brief Gyroscope x, y, and z values [rad/s]
    float gx, gy, gz;

    /// \brief Timestamp at which the event happened in &micro;s
    timestamp t;

    /// \brief Default constructor
    EventIMU() = default;

    /// \brief Constructor
    /// @param ax : accelerometer x value [g]
    /// @param ay : accelerometer y value [g]
    /// @param az : accelerometer z value [g]
    /// @param gx : gyroscope x value [rad/s]
    /// @param gy : gyroscope y value [rad/s]
    /// @param gz : gyroscope z value [rad/s]
    /// @param ts : timestamp of the event, in &micro;s
    inline EventIMU(float ax, float ay, float az, float gx, float gy, float gz, timestamp ts) :
        ax(ax),
        ay(ay),
        az(az),
        gx(gx),
        gy(gy),
        gz(gz),
        t(ts) {}

    /**
     * Write EventIMU in buffer
     */
    void write_event(void *buf, timestamp origin) const {
        RawEvent *buffer = (RawEvent *)buf;
        buffer->ts       = t - origin;
        buffer->ax       = ax;
        buffer->ay       = ay;
        buffer->az       = az;
        buffer->gx       = gx;
        buffer->gy       = gy;
        buffer->gz       = gz;
    }

    /**
     * Read EventIMU (old format) from buffer
     */
    static EventIMU read_event_v1(void *buf, const timestamp &delta_ts) {
        RawEventV1 *buffer = static_cast<RawEventV1 *>(buf);
        return EventIMU(buffer->ax, buffer->ay, buffer->az, buffer->gx, buffer->gy, buffer->gz, buffer->ts + delta_ts);
    }

    /**
     * Read event 2d from buffer
     */
    static EventIMU read_event(void *buf, const timestamp &delta_ts) {
        RawEvent *buffer = static_cast<RawEvent *>(buf);
        return EventIMU(buffer->ax, buffer->ay, buffer->az, buffer->gx, buffer->gy, buffer->gz, buffer->ts + delta_ts);
    }

    /**
     * Get the size of the RawEvent
     */
    static size_t get_raw_event_size() {
        return sizeof(RawEvent);
    }

    /// Structure of size 64 bits to represent one event (old format)
    FORCE_PACK(struct RawEventV1 {
        unsigned int ts : 32;
        unsigned int x : 9;
        unsigned int y : 8;
        unsigned int p : 1;
        unsigned int padding : 14;
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
    });

    /// operator<<
    friend std::ostream &operator<<(std::ostream &output, const EventIMU &e) {
        output << "EventIMU: (";
        output << e.ax << ", " << e.ay << ", " << e.az << ", " << e.gx << ", " << e.gy << ", " << e.gz << ", " << e.t
               << ", ";
        output << ")";
        return output;
    }

    /// Structure of size 64 bits to represent one event
    FORCE_PACK(struct RawEvent {
        unsigned int ts : 32;
        unsigned int x : 14; // kept for retro-compatibility but empty field
        unsigned int y : 14; // kept for retro-compatibility but empty field
        unsigned int p : 4;  // kept for retro-compatibility but empty field
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
    });
};

} /* namespace Prophesee */

PROPHESEE_DEFINE_EVENT_TRAIT(Prophesee::EventIMU, 15, "IMU")

#endif /* PROPHESEE_CORE_EVENT_IMU_H */
