#include "AP_RadarOdom.h"
#include <AP_Logger/AP_Logger_config.h>

#if HAL_RADARODOM_ENABLED && HAL_LOGGING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

void AP_RadarOdom::Write_RadarOdom(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &pos, const Vector3f &vel,
                                   const Quaternion &attitude, float posErr, float velErr, bool valid) const
{
    const struct log_RadarOdom pkt {
        LOG_PACKET_HEADER_INIT(LOG_RADARODOM_MSG),
        time_us         : AP_HAL::micros64(),
        remote_time_us  : remote_time_us,
        time_ms         : time_ms,
        pos_x           : pos.x,
        pos_y           : pos.y,
        pos_z           : pos.z,
        vel_x           : vel.x,
        vel_y           : vel.y,
        vel_z           : vel.z,
        q1              : attitude.q1,
        q2              : attitude.q2,
        q3              : attitude.q3,
        q4              : attitude.q4,
        pos_err         : posErr,
        vel_err         : velErr,
        valid           : uint8_t(valid)
    };
    AP::logger().WriteBlock(&pkt, sizeof(log_RadarOdom));
}

#endif // HAL_RADARODOM_ENABLED && HAL_LOGGING_ENABLED
