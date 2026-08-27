#include "AP_RadarOdom_config.h"

#if HAL_RADARODOM_ENABLED

#include "AP_RadarOdom.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_RadarOdom::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Radar odometry connection type
    // @Description: Radar odometry connection type
    // @Values: 0:None,1:MAVLink
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 0, AP_RadarOdom, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _FUSE
    // @DisplayName: Radar odometry fusion control
    // @Description: Controls which RADAR_ODOMETRY fields are sent to the EKF external navigation input. EKF source parameters must still be set to ExternalNav for the data to be fused.
    // @Bitmask: 0:Position,1:Velocity
    // @User: Advanced
    AP_GROUPINFO("_FUSE", 1, AP_RadarOdom, _fuse, FUSE_POSITION | FUSE_VELOCITY),

    // @Param: _POS_X
    // @DisplayName: Radar odometry X position offset
    // @Description: X position of the radar odometry sensor in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Radar odometry Y position offset
    // @Description: Y position of the radar odometry sensor in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Radar odometry Z position offset
    // @Description: Z position of the radar odometry sensor in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_POS", 2, AP_RadarOdom, _pos_offset, 0.0f),

    // @Param: _DELAY_MS
    // @DisplayName: Radar odometry sensor delay
    // @Description: Radar odometry sensor delay relative to inertial measurements
    // @Units: ms
    // @Range: 0 250
    // @User: Advanced
    AP_GROUPINFO("_DELAY_MS", 3, AP_RadarOdom, _delay_ms, 10),

    // @Param: _POS_M_NSE
    // @DisplayName: Radar odometry position measurement noise
    // @Description: Radar odometry position measurement noise minimum in meters. This value is used if the sensor provides a lower noise value or no noise value.
    // @Units: m
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("_POS_M_NSE", 4, AP_RadarOdom, _pos_noise, 0.2f),

    // @Param: _VEL_M_NSE
    // @DisplayName: Radar odometry velocity measurement noise
    // @Description: Radar odometry velocity measurement noise minimum in meters per second. This value is used if the sensor provides a lower noise value or no noise value.
    // @Units: m/s
    // @Range: 0.05 5.0
    // @User: Advanced
    AP_GROUPINFO("_VEL_M_NSE", 5, AP_RadarOdom, _vel_noise, 0.1f),

    // @Param: _YAW_M_NSE
    // @DisplayName: Radar odometry yaw measurement noise
    // @Description: Radar odometry yaw measurement noise minimum in radians.
    // @Units: rad
    // @Range: 0.05 1.0
    // @User: Advanced
    AP_GROUPINFO("_YAW_M_NSE", 6, AP_RadarOdom, _yaw_noise, 0.2f),

    // @Param: _QUAL_MIN
    // @DisplayName: Radar odometry minimum quality
    // @Description: Radar odometry will only be sent to the EKF if over this quality. RADAR_ODOMETRY currently maps valid messages to 100 and invalid messages to -1.
    // @Units: %
    // @Range: -1 100
    // @User: Advanced
    AP_GROUPINFO("_QUAL_MIN", 7, AP_RadarOdom, _quality_min, 0),

    AP_GROUPEND
};

AP_RadarOdom::AP_RadarOdom()
{
    AP_Param::setup_object_defaults(this, var_info);
    _last_update_ms = 0;
    _quality = 0;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_RadarOdom must be singleton");
    }
#endif
    _singleton = this;
}

void AP_RadarOdom::init()
{
}

bool AP_RadarOdom::healthy() const
{
    return enabled() && ((AP_HAL::millis() - _last_update_ms) < AP_RADARODOM_TIMEOUT_MS);
}

bool AP_RadarOdom::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!enabled()) {
        return true;
    }

    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "not healthy");
        return false;
    }

    return true;
}

void AP_RadarOdom::handle_odometry(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &pos, const Vector3f &vel,
                                   const Quaternion &attitude, float posErr, float velErr, bool valid)
{
    if (!enabled()) {
        return;
    }

    _quality = valid ? 100 : -1;

    posErr = constrain_float(posErr, _pos_noise, 100.0f);
    velErr = constrain_float(velErr, _vel_noise, 100.0f);

    const uint8_t fuse = uint8_t(_fuse.get());
    const bool quality_ok = (_quality >= _quality_min);

    Vector3f pos_corrected = pos;
    const Vector3f pos_offset = _pos_offset;
    if (!pos_offset.is_zero()) {
        pos_corrected -= AP::ahrs().get_rotation_body_to_ned() * pos_offset;
    }

    if (valid && quality_ok && ((fuse & FUSE_POSITION) != 0)) {
        AP::ahrs().writeExtNavData(pos_corrected, attitude, posErr, _yaw_noise, time_ms, get_delay_ms(), 0);
    }

    if (valid && quality_ok && ((fuse & FUSE_VELOCITY) != 0)) {
        AP::ahrs().writeExtNavVelData(vel, velErr, time_ms, get_delay_ms());
    }

    _last_update_ms = AP_HAL::millis();

#if HAL_LOGGING_ENABLED
    Write_RadarOdom(remote_time_us, time_ms, pos, vel, attitude, posErr, velErr, valid);
#endif
}

AP_RadarOdom *AP_RadarOdom::_singleton;

namespace AP {

AP_RadarOdom *radarodom()
{
    return AP_RadarOdom::get_singleton();
}

}

#endif // HAL_RADARODOM_ENABLED
