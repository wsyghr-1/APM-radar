# 雷达里程计改动说明

## 提交信息

- 提交：`bf3067412f`
- 提交标题：`feat: 增加独立雷达里程计模块`
- 提交时间：`2026-06-01 00:03:49 +0800`
- 作者：`wsyghr-1`

本次提交将原先通过 `AP_VisualOdom` 处理 `RADAR_ODOMETRY` 的逻辑拆分为独立的 `AP_RadarOdom` 模块，并把雷达里程计接入 MAVLink、EKF 外部导航输入、arm 前检查、状态上报、日志和构建系统。

## 改动范围

新增文件：

- `libraries/AP_RadarOdom/AP_RadarOdom.cpp`
- `libraries/AP_RadarOdom/AP_RadarOdom.h`
- `libraries/AP_RadarOdom/AP_RadarOdom_Logging.cpp`
- `libraries/AP_RadarOdom/AP_RadarOdom_config.h`
- `libraries/AP_RadarOdom/LogStructure.h`

修改文件：

- `modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`
- `Tools/ardupilotwaf/ardupilotwaf.py`
- `libraries/AP_Arming/AP_Arming.cpp`
- `libraries/AP_Logger/LogStructure.h`
- `libraries/AP_NavEKF/AP_NavEKF_Source.cpp`
- `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
- `libraries/AP_Vehicle/AP_Vehicle.cpp`
- `libraries/AP_Vehicle/AP_Vehicle.h`
- `libraries/GCS_MAVLink/GCS.cpp`
- `libraries/GCS_MAVLink/GCS_Common.cpp`

## 核心设计

新增 `AP_RadarOdom` 单例模块，模块由 `HAL_RADARODOM_ENABLED` 控制编译：

```cpp
#define HAL_RADARODOM_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024 && HAL_GCS_ENABLED)
```

模块当前支持 MAVLink 输入类型：

- `RADO_TYPE = 0`：关闭
- `RADO_TYPE = 1`：MAVLink `RADAR_ODOMETRY`

雷达数据进入系统后的主链路如下：

```text
MAVLink RADAR_ODOMETRY
        |
        v
GCS_MAVLINK::handle_radar_odometry()
        |
        v
AP_RadarOdom::handle_odometry()
        |
        +--> AP::ahrs().writeExtNavData()
        |
        +--> AP::ahrs().writeExtNavVelData()
        |
        +--> RADO 日志
```

## MAVLink 处理

`modules/mavlink/message_definitions/v1.0/ardupilotmega.xml` 中新增 `RADAR_ODOMETRY` 消息，消息 ID 为 `16000`：

```xml
<message id="16000" name="RADAR_ODOMETRY">
  <description>Radar Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard for aerial vehicles.</description>
  <field type="uint64_t" name="time_usec" units="us">Timestamp (UNIX Epoch time or time since system boot).</field>
  <field type="float[3]" name="p_local" units="m">UAV position in global NED frame</field>
  <field type="float[3]" name="v_local" units="m/s">UAV velocity in global NED frame</field>
  <field type="float[4]" name="q">Quaternion from body to global: w,x,y,z (1 0 0 0 is the null-rotation)</field>
  <field type="float[3]" name="v_body" units="m/s">UAV velocity in body frame</field>
  <field type="float[3]" name="p_local_std">Position standard deviation in NED frame</field>
  <field type="float[3]" name="v_local_std">Velocity standard deviation in global NED frame</field>
  <field type="float[3]" name="v_body_std">Velocity standard deviation in body frame</field>
  <field type="uint8_t" name="is_valid">Radar odom valid flag (0=invalid, 1=valid)</field>
</message>
```

`libraries/GCS_MAVLink/GCS_Common.cpp` 中新增 `RADAR_ODOMETRY` 消息处理逻辑：

- 解码 `mavlink_radar_odometry_t`
- 检查位置、速度、四元数是否为有限值
- 拒绝全 0 四元数，并对四元数归一化
- 使用 `correct_offboard_timestamp_usec_to_ms()` 将外部时间戳转换为本机时间
- 从 `p_local_std[3]` 计算位置误差 RSS
- 从 `v_local_std[3]` 计算速度误差 RSS
- 将位置、速度、姿态、误差和 `is_valid` 传给 `AP_RadarOdom`

当前飞控侧主要使用这些字段：`time_usec`、`p_local`、`v_local`、`q`、`p_local_std`、`v_local_std` 和 `is_valid`。`v_body` 与 `v_body_std` 已在消息中定义，但本次接入逻辑暂未参与 EKF 融合。

相比之前的做法，`RADAR_ODOMETRY` 不再复用 `AP_VisualOdom` 的 pose 和 speed 接口，而是走独立的雷达里程计模块。

## EKF 融合

`AP_RadarOdom::handle_odometry()` 会根据参数和数据有效性选择是否写入 EKF 外部导航输入：

- 位置和姿态：`AP::ahrs().writeExtNavData()`
- 速度：`AP::ahrs().writeExtNavVelData()`

融合开关由 `RADO_FUSE` 控制：

- bit 0：融合位置
- bit 1：融合速度
- 默认值：`3`，即位置和速度都融合

有效性判断：

- `is_valid != 0` 时质量记为 `100`
- `is_valid == 0` 时质量记为 `-1`
- 只有质量不低于 `RADO_QUAL_MIN` 时才送入 EKF

传感器位置偏移由 `RADO_POS_X/Y/Z` 配置。模块会把机体系安装偏移旋转到 NED 后，从雷达上报的位置中扣除：

```cpp
pos_corrected -= AP::ahrs().get_rotation_body_to_ned() * pos_offset;
```

`libraries/AP_NavEKF/AP_NavEKF_Source.cpp` 中也调整了 pre-arm 检查逻辑：当 EKF source 需要 ExternalNav 时，只要 `AP_VisualOdom` 或 `AP_RadarOdom` 任一模块启用即可通过外部导航可用性检查，失败信息从 `VisualOdom` 改为更通用的 `ExtNav`。

`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` 中新增雷达延迟对 EKF buffer 长度的影响，最大按 `250ms` 限制：

```cpp
maxTimeDelay_ms = MAX(maxTimeDelay_ms, MIN(radar_odom->get_delay_ms(), 250));
```

## 参数

雷达里程计参数挂在 `AP_Vehicle` 的 `RADO` 参数组下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `RADO_TYPE` | `0` | 雷达里程计输入类型，`0=None`，`1=MAVLink` |
| `RADO_FUSE` | `3` | 融合控制 bitmask，bit0 位置，bit1 速度 |
| `RADO_POS_X` | `0.0` | 雷达相对机体原点的 X 偏移，前为正，单位 m |
| `RADO_POS_Y` | `0.0` | 雷达相对机体原点的 Y 偏移，右为正，单位 m |
| `RADO_POS_Z` | `0.0` | 雷达相对机体原点的 Z 偏移，下为正，单位 m |
| `RADO_DELAY_MS` | `10` | 雷达测量相对 IMU 的延迟，单位 ms |
| `RADO_POS_M_NSE` | `0.2` | 位置测量噪声下限，单位 m |
| `RADO_VEL_M_NSE` | `0.1` | 速度测量噪声下限，单位 m/s |
| `RADO_YAW_M_NSE` | `0.2` | yaw 测量噪声下限，单位 rad |
| `RADO_QUAL_MIN` | `0` | 最低质量阈值，当前有效消息映射为 `100`，无效消息映射为 `-1` |

## 健康状态和解锁检查

`AP_RadarOdom::healthy()` 的判断条件：

- `RADO_TYPE` 已启用
- 距离最近一次雷达数据更新时间小于 `300ms`

`libraries/AP_Arming/AP_Arming.cpp` 中新增 arm 前检查：

- 如果雷达里程计关闭，则不阻止解锁
- 如果雷达里程计启用但不健康，则报错：`RadarOdom: not healthy`

`libraries/GCS_MAVLink/GCS.cpp` 中新增状态上报：

- 雷达启用后，`MAV_SYS_STATUS_SENSOR_VISION_POSITION` 会被标记为 present/enabled
- 雷达健康时，该 sensor health 会被标记为 healthy

## 日志

新增 `RADO` 日志消息，定义在 `libraries/AP_RadarOdom/LogStructure.h`，写入逻辑在 `AP_RadarOdom_Logging.cpp`。

日志字段：

| 字段 | 说明 |
| --- | --- |
| `TimeUS` | 飞控系统时间 |
| `RTimeUS` | 外部雷达系统时间 |
| `CTimeMS` | 修正后的本机时间，单位 ms |
| `PX/PY/PZ` | NED 位置 |
| `VX/VY/VZ` | NED 速度 |
| `Q1/Q2/Q3/Q4` | 姿态四元数 |
| `PErr` | 位置误差 |
| `VErr` | 速度误差 |
| `Ign` | 被忽略的融合项 bitmask |
| `V` | 原始有效标志 |
| `Q` | 质量值 |

`libraries/AP_Logger/LogStructure.h` 已接入 `LOG_STRUCTURE_FROM_RADARODOM` 和 `LOG_IDS_FROM_RADARODOM`。

## 构建接入

`Tools/ardupilotwaf/ardupilotwaf.py` 中将 `AP_RadarOdom` 加入 `COMMON_VEHICLE_DEPENDENT_LIBRARIES`，使该库参与车辆目标构建。

`libraries/AP_Vehicle/AP_Vehicle.h/.cpp` 中新增成员和初始化：

- `AP_RadarOdom radar_odom`
- 参数组前缀：`RADO`
- 启动时调用 `radar_odom.init()`

## 使用配置建议

基础启用配置：

```text
RADO_TYPE = 1
RADO_FUSE = 3
```

如果 EKF 需要融合雷达里程计，还需要把对应 EKF source 配置为 ExternalNav。`RADO_FUSE` 只控制雷达模块是否向 AHRS/EKF 写入位置或速度，不能替代 EKF source 参数配置。

建议根据实际安装和链路延迟配置：

```text
RADO_POS_X/Y/Z
RADO_DELAY_MS
RADO_POS_M_NSE
RADO_VEL_M_NSE
RADO_YAW_M_NSE
```

## 注意事项

- 当前 `RADAR_ODOMETRY` 的质量值只由 `is_valid` 映射，未使用更细粒度的质量评分。
- `RADO_QUAL_MIN` 默认是 `0`，因此有效消息 `100` 会融合，无效消息 `-1` 会被忽略。
- `RADAR_ODOMETRY` 四元数顺序按 `w, x, y, z` 构造。
- 位置和速度标准差使用三轴 RSS 合成为单个误差值，再由 `RADO_POS_M_NSE` 和 `RADO_VEL_M_NSE` 设置下限。
- 雷达启用后，如果 300ms 内没有收到有效处理过的数据，arm 前检查会失败。
