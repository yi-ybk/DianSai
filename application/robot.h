#pragma once

#include <pthread.h>
#include <stdint.h>

/** @brief 小球位置控制器的RAM在线调参区 */
typedef struct
{
    uint32_t enabled;                     /**< 0：关闭控制并回水平位，非0：启用控制 */
    float kp;                             /**< 比例增益(deg/px) */
    float ki;                             /**< 积分增益(deg/(px*s)) */
    float kd;                             /**< 速度反馈增益(deg/(px/s)) */
    float control_direction;              /**< 舵机控制方向，只使用正负号 */
    float level_angle_deg;                /**< 管道水平时的舵机角度 */
    float output_min_angle_deg;           /**< RAM软最小角度，仍受机械硬限幅保护 */
    float output_max_angle_deg;           /**< RAM软最大角度，仍受机械硬限幅保护 */
    float integral_limit_px_s;            /**< 积分绝对值上限(px*s) */
    float derivative_filter_tau_s;        /**< 测量速度低通滤波时间常数(s) */
    float deadband_px;                    /**< 位置误差死区(px) */
    float acceleration_ff_gain;           /**< 小车纵向加速度前馈增益(deg/(m/s^2))，允许用正负号修正方向 */
    float breakaway_angle_deg;            /**< 小球静止时的启动力前馈角度 */
    float breakaway_error_px;             /**< 启动力前馈的最小位置误差 */
    float breakaway_speed_px_s;            /**< 判定小球静止的速度阈值 */
    float maximum_slew_deg_per_s;          /**< 舵机指令最大变化速度 */
} BallPidRamConfig_t;

/** @brief 小球位置控制器的RAM运行状态，供CCS在线观察 */
typedef struct
{
    float error_px;
    float integral_px_s;
    float measured_speed_px_s;
    float proportional_deg;
    float integral_deg;
    float derivative_deg;
    float acceleration_feedforward_deg;
    float breakaway_feedforward_deg;
    float unsaturated_angle_deg;
    float command_angle_deg;
    float dt_s;
    uint32_t update_count;
    uint32_t reset_count;
    float chassis_acceleration_mps2;       /**< 当前送入前馈的小车指令加速度(m/s^2) */
} BallPidRamState_t;

/** @brief 循迹过程量与整圈统计，循迹启动时清零、停止后保留 */
typedef struct
{
    uint32_t active;                       /**< 1：正在记录，0：未运行或本圈已结束 */
    uint32_t mode;                         /**< 本圈运行模式 */
    uint32_t update_count;                 /**< 本圈采样次数 */
    uint32_t elapsed_ms;                   /**< 当前或最终运行时间(ms) */
    uint32_t stop_reason;                  /**< 最终停车原因，运行中为0 */

    uint32_t black_mask;                   /**< 当前8路灰度掩码 */
    uint32_t black_count;                  /**< 当前连续黑线传感器数量 */
    float normalized_error;                /**< 当前归一化循迹误差 */
    float last_nonzero_error;              /**< 最近一次非零循迹误差 */
    float line_lost_time_s;                /**< 当前连续丢线时间(s) */

    float pid_p_out;                       /**< 当前循迹P项 */
    float pid_i_out;                       /**< 当前循迹I项 */
    float pid_d_out;                       /**< 当前循迹D项 */
    float pid_output;                      /**< 当前基础循迹PID输出 */
    float base_turn_speed_radps;           /**< track模块限幅后的基础转向速度 */
    float active_turn_limit_radps;         /**< 当前模式最终转向限幅 */

    float target_forward_speed_mps;        /**< 底盘请求前进速度 */
    float target_turn_speed_radps;         /**< 底盘请求转向速度 */
    float command_forward_speed_mps;       /**< 轮速约束后实际下发前进速度 */
    float command_turn_speed_radps;        /**< 轮速约束后实际下发转向速度 */
    float measured_forward_speed_mps;      /**< 编码器估算前进速度 */
    float measured_turn_speed_radps;       /**< 编码器估算转向速度 */
    float wheel_speed_scale;               /**< 底盘轮速等比例缩放值 */

    float left_target_speed_mps;           /**< 左轮目标速度 */
    float left_measured_speed_mps;         /**< 左轮实测速度 */
    float right_target_speed_mps;          /**< 右轮目标速度 */
    float right_measured_speed_mps;        /**< 右轮实测速度 */

    float max_abs_error;                   /**< 本圈最大循迹误差绝对值 */
    float max_abs_pid_d_out;               /**< 本圈最大D项绝对值 */
    float max_abs_base_turn_speed_radps;   /**< 本圈最大基础转向速度绝对值 */
    float max_abs_command_turn_speed_radps;/**< 本圈最大实际转向命令绝对值 */
    float min_command_forward_speed_mps;   /**< 非零前进命令最小值 */
    float max_command_forward_speed_mps;   /**< 前进命令最大值 */
    float min_wheel_speed_scale;           /**< 本圈最小轮速缩放值 */
    float max_left_speed_error_mps;        /**< 左轮最大速度跟踪误差 */
    float max_right_speed_error_mps;       /**< 右轮最大速度跟踪误差 */
    float max_line_lost_time_s;            /**< 单次最长丢线时间 */

    uint32_t line_lost_active;             /**< 当前是否处于丢线状态 */
    uint32_t line_lost_event_count;        /**< 本圈进入丢线状态次数 */
    uint32_t line_lost_sample_count;       /**< 丢线采样次数 */
    uint32_t outer_sensor_sample_count;    /**< 仅最外侧传感器命中次数 */
    uint32_t large_error_sample_count;     /**< |误差|>=1的采样次数 */
    uint32_t turn_saturation_sample_count; /**< 最终转向命令触及限幅次数 */
    uint32_t max_direction_change_count;   /**< 方向切换确认计数峰值 */
    uint32_t sensor_hit_count[8];          /**< 各灰度通道本圈命中次数 */
    uint32_t black_count_histogram[9];     /**< 黑线连续宽度0..8统计 */

    uint32_t max_error_elapsed_ms;         /**< 最大误差出现时间 */
    uint32_t max_error_black_mask;         /**< 最大误差时灰度掩码 */
    float max_error_signed;                /**< 最大误差时带符号误差 */
    float max_error_forward_speed_mps;     /**< 最大误差时前进命令 */
    float max_error_turn_speed_radps;      /**< 最大误差时转向命令 */
    float final_distance_m;                /**< 当前或最终累计距离 */
} TrackDebugState_t;

#define TRACK_TRACE_CAPACITY 480U
#define TRACK_TRACE_MAGIC    0x314B5254U
#define TRACK_TRACE_VERSION  3U

#define TRACK_TRACE_STATUS_BLACK_COUNT_MASK 0x0FU
#define TRACK_TRACE_STATUS_LINE_LOST        0x10U
#define TRACK_TRACE_STATUS_OUTER_SENSOR     0x20U
#define TRACK_TRACE_STATUS_TURN_SATURATED   0x40U
#define TRACK_TRACE_STATUS_WHEEL_SCALED     0x80U

/** @brief 单个循迹轨迹采样点，采用定点数压缩以节省RAM */
typedef struct
{
    uint16_t elapsed_ms;                   /**< 相对本圈启动时刻(ms) */
    uint16_t distance_mm;                  /**< 底盘累计距离绝对值(mm) */
    uint8_t black_mask;                    /**< 8路灰度掩码 */
    uint8_t status;                        /**< 低4位为黑线宽度，高4位为状态标志 */
    int16_t normalized_error_x1000;        /**< 归一化误差乘1000 */
    int16_t control_error_x1000;           /**< 限速后的PID控制误差乘1000 */
    int16_t pid_output_mradps;             /**< 循迹PID输出乘1000 */
    int16_t command_forward_mmps;          /**< 实际前进指令(mm/s) */
    int16_t command_turn_mradps;           /**< 实际转向指令(mrad/s) */
    uint8_t raw_black_mask;                /**< 未滤波的8路灰度掩码 */
    uint8_t selected_black_mask;           /**< 本次算法选择的连续黑线段 */
    uint8_t black_run_count;               /**< 滤波掩码中的连续黑线段数量 */
    uint8_t selection_state;               /**< 高4位选择标志，低4位反向确认计数 */
} TrackTraceSample_t;

/** @brief 循迹轨迹头，描述RAM采样区的有效范围和运行状态 */
typedef struct
{
    uint32_t magic;                        /**< 固定为TRACK_TRACE_MAGIC */
    uint32_t version;                      /**< 数据格式版本 */
    uint32_t sample_size;                  /**< 单个采样点字节数 */
    uint32_t capacity;                     /**< 最大采样点数 */
    uint32_t count;                        /**< 当前有效采样点数 */
    uint32_t write_index;                  /**< 环形缓冲区下一写入位置 */
    uint32_t sample_period_ms;             /**< 采样周期(ms) */
    uint32_t mode;                         /**< 本圈运行模式 */
    uint32_t active;                       /**< 1：正在记录 */
    uint32_t complete;                     /**< 1：已正常进入停止流程 */
    uint32_t overflow;                     /**< 1：采样区已满，后续数据未记录 */
    uint32_t start_tick_ms;                /**< 启动时系统节拍 */
    uint32_t last_sample_tick_ms;          /**< 最近采样时系统节拍 */
    uint32_t stop_tick_ms;                 /**< 停止时系统节拍 */
    uint32_t capture_call_count;           /**< 记录期间控制循环调用次数 */
} TrackTraceHeader_t;

/** @brief 连续RAM轨迹存储区，可由J-Link一次性读出 */
typedef struct
{
    TrackTraceHeader_t header;
    TrackTraceSample_t samples[TRACK_TRACE_CAPACITY];
} TrackTraceStore_t;

extern volatile BallPidRamConfig_t ball_pid_ram;
extern volatile BallPidRamState_t ball_pid_state;
extern volatile TrackDebugState_t track_debug_state;
extern volatile TrackTraceStore_t track_trace_store;
extern volatile uint32_t ball_pid_reset_request;
extern volatile float ball_target;
extern volatile float ball_chassis_acceleration_mps2;
extern volatile float ball_real;
extern volatile float now_angel;

void robotInit(void);
