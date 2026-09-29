#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef HAL_RADARODOM_ENABLED
#define HAL_RADARODOM_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024 && HAL_GCS_ENABLED)
#endif
