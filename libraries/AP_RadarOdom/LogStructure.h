#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_RadarOdom_config.h"

#define LOG_IDS_FROM_RADARODOM \
    LOG_RADARODOM_MSG

// @LoggerMessage: RADO
// @Description: Radar Odometry
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: PX: Position X-axis (North-South)
// @Field: PY: Position Y-axis (East-West)
// @Field: PZ: Position Z-axis (Down-Up)
// @Field: VX: Velocity X-axis (North-South)
// @Field: VY: Velocity Y-axis (East-West)
// @Field: VZ: Velocity Z-axis (Down-Up)
// @Field: Q1: Attitude quaternion scalar
// @Field: Q2: Attitude quaternion X-axis
// @Field: Q3: Attitude quaternion Y-axis
// @Field: Q4: Attitude quaternion Z-axis
// @Field: PErr: Position estimate error
// @Field: VErr: Velocity estimate error
// @Field: V: Valid flag
struct PACKED log_RadarOdom {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float q1;
    float q2;
    float q3;
    float q4;
    float pos_err;
    float vel_err;
    uint8_t valid;
};

#if HAL_RADARODOM_ENABLED
#define LOG_STRUCTURE_FROM_RADARODOM \
    { LOG_RADARODOM_MSG, sizeof(log_RadarOdom), \
      "RADO", "QQIffffffffffffB", "TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,VX,VY,VZ,Q1,Q2,Q3,Q4,PErr,VErr,V", "sssmmmnnn----mn-", "FFC000000000000-" },
#else
#define LOG_STRUCTURE_FROM_RADARODOM
#endif
