#pragma once

#include "AP_RadarOdom_config.h"

#if HAL_RADARODOM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger_config.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#define AP_RADARODOM_TIMEOUT_MS 300

class AP_RadarOdom
{
public:
    AP_RadarOdom();

    static AP_RadarOdom *get_singleton()
    {
        return _singleton;
    }

    enum class Type {
        None = 0,
        MAVLink = 1,
    };

    enum FusionMask {
        FUSE_POSITION = 1U << 0,
        FUSE_VELOCITY = 1U << 1,
    };

    void init();

    bool enabled() const { return _type != Type::None; }
    bool healthy() const;
    int8_t quality() const { return _quality; }

    uint16_t get_delay_ms() const { return MAX(0, _delay_ms); }
    const Vector3f &get_pos_offset() const { return _pos_offset; }

    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    void handle_odometry(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &pos, const Vector3f &vel,
                         const Quaternion &attitude, float posErr, float velErr, bool valid);

    static const struct AP_Param::GroupInfo var_info[];

    // MAVLink settings
    static constexpr int32_t MAVLINK_SYSTEM_TIME_INTERVAL_US = 1000000;
    static constexpr int32_t MAVLINK_ATTITUDE_QUATERNION_INTERVAL_US = 20000;
    static constexpr int32_t MAVLINK_HIGHRES_IMU_INTERVAL_US = 8333;

private:
    static AP_RadarOdom *_singleton;

    AP_Enum<Type> _type;
    AP_Int8 _fuse;
    AP_Vector3f _pos_offset;
    AP_Int16 _delay_ms;
    AP_Float _pos_noise;
    AP_Float _vel_noise;
    AP_Float _yaw_noise;
    AP_Int8 _quality_min;

    uint32_t _last_update_ms;
    int8_t _quality;

#if HAL_LOGGING_ENABLED
    void Write_RadarOdom(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &pos, const Vector3f &vel,
                         const Quaternion &attitude, float posErr, float velErr, bool valid) const;
#endif
};

namespace AP {
    AP_RadarOdom *radarodom();
};

#endif // HAL_RADARODOM_ENABLED
