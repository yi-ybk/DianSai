/**
 * @file    chassis.c
 * @brief   底盘组件实现
 * @details 管理外部驱动轮对象并提供底盘前进、后退和转向速度控制，不对外暴露底层对象行为。
 */
#include "chassis.h"
#include <string.h>

#define CHASSIS_FLOAT_EPSILON 0.000001f /**< 浮点数有效性判断阈值 */

static void ChassisBindMethods(Chassis_t *chassis);
static bool ChassisConfigIsValid(const ChassisInitConfig_t *config);
static void ChassisClearVelocityCommand(Chassis_t *chassis);
static void ChassisSyncData(Chassis_t *chassis);
static float ChassisAbs(float value);

/**
 * @brief   初始化底盘对象
 * @param   chassis 底盘对象指针
 * @param   config 底盘初始化配置
 * @return  bool 成功返回true，失败返回false
 */
bool ChassisInit(Chassis_t *chassis, const ChassisInitConfig_t *config)
{
    if ((chassis == NULL) || (config == NULL))
        return false;

    ChassisBindMethods(chassis);
    if (chassis->initialized)
        return true;

    if (!ChassisConfigIsValid(config))
        return false;

    chassis->init_config = *config;
    chassis->wheels = config->wheels;
    chassis->wheel_kinematics = config->wheel_kinematics;
    memset(&chassis->data, 0, sizeof(chassis->data));
    chassis->data.wheel_count = config->wheel_count;
    chassis->data.wheel_speed_scale = 1.0f;
    chassis->initialized = true;

    if (config->auto_start)
        ChassisStart(chassis);
    else
        ChassisSyncData(chassis);

    return true;
}

/**
 * @brief   启动底盘
 * @param   chassis 底盘对象指针
 */
void ChassisStart(Chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || (!chassis->initialized))
        return;

    for (index = 0U; index < chassis->data.wheel_count; ++index)
        WheelStart(chassis->wheels[index]);

    ChassisSyncData(chassis);
}

/**
 * @brief   停止底盘
 * @param   chassis 底盘对象指针
 */
void ChassisStop(Chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || (!chassis->initialized))
        return;

    for (index = 0U; index < chassis->data.wheel_count; ++index)
        WheelStop(chassis->wheels[index]);

    ChassisClearVelocityCommand(chassis);
    ChassisSyncData(chassis);
}

/**
 * @brief   设置底盘前进和转向速度
 * @details 根据驱动轮运动学系数计算目标轮速，超出任一轮最大速度时等比例缩放全部轮速。
 * @param   chassis 底盘对象指针
 * @param   forward_speed_mps 前进速度(m/s)，负值表示后退
 * @param   turn_speed_radps 转向角速度(rad/s)，正值表示逆时针
 * @return  bool 设置成功返回true，底盘未初始化返回false
 */
bool ChassisSetVelocity(Chassis_t *chassis, float forward_speed_mps, float turn_speed_radps)
{
    float wheel_speed;
    float speed_ratio;
    float max_speed_ratio = 1.0f;
    float speed_scale;
    uint8_t index;

    if ((chassis == NULL) || (!chassis->initialized))
        return false;

    for (index = 0U; index < chassis->data.wheel_count; ++index)
    {
        wheel_speed = chassis->wheel_kinematics[index].forward_coefficient * forward_speed_mps +
                      chassis->wheel_kinematics[index].turn_coefficient_m * turn_speed_radps;
        speed_ratio = ChassisAbs(wheel_speed) /
                      chassis->wheels[index]->init_config.max_linear_speed_mps;

        if (speed_ratio > max_speed_ratio)
            max_speed_ratio = speed_ratio;
    }

    speed_scale = 1.0f / max_speed_ratio;
    for (index = 0U; index < chassis->data.wheel_count; ++index)
    {
        wheel_speed = chassis->wheel_kinematics[index].forward_coefficient * forward_speed_mps +
                      chassis->wheel_kinematics[index].turn_coefficient_m * turn_speed_radps;
        WheelSetLinearSpeed(chassis->wheels[index], wheel_speed * speed_scale);
    }

    chassis->data.target_forward_speed_mps = forward_speed_mps;
    chassis->data.target_turn_speed_radps = turn_speed_radps;
    chassis->data.command_forward_speed_mps = forward_speed_mps * speed_scale;
    chassis->data.command_turn_speed_radps = turn_speed_radps * speed_scale;
    chassis->data.wheel_speed_scale = speed_scale;
    return true;
}

/**
 * @brief   设置底盘前进速度并保持当前转向速度
 * @param   chassis 底盘对象指针
 * @param   forward_speed_mps 前进速度(m/s)，负值表示后退
 * @return  bool 设置成功返回true，底盘未初始化返回false
 */
bool ChassisSetForwardSpeed(Chassis_t *chassis, float forward_speed_mps)
{
    if ((chassis == NULL) || (!chassis->initialized))
        return false;

    return ChassisSetVelocity(chassis,
                              forward_speed_mps,
                              chassis->data.target_turn_speed_radps);
}

/**
 * @brief   设置底盘转向速度并保持当前前进速度
 * @param   chassis 底盘对象指针
 * @param   turn_speed_radps 转向角速度(rad/s)，正值表示逆时针
 * @return  bool 设置成功返回true，底盘未初始化返回false
 */
bool ChassisSetTurnSpeed(Chassis_t *chassis, float turn_speed_radps)
{
    if ((chassis == NULL) || (!chassis->initialized))
        return false;

    return ChassisSetVelocity(chassis,
                              chassis->data.target_forward_speed_mps,
                              turn_speed_radps);
}

/**
 * @brief   更新底盘控制和运行状态
 * @details 应按固定周期调用，内部更新全部驱动轮闭环控制及底盘速度反馈。
 * @param   chassis 底盘对象指针
 * @param   dt_s 更新周期(s)
 */
void ChassisUpdate(Chassis_t *chassis, float dt_s)
{
    uint8_t index;

    if ((chassis == NULL) || (!chassis->initialized))
        return;

    if (dt_s < 0.0f)
        dt_s = 0.0f;

    for (index = 0U; index < chassis->data.wheel_count; ++index)
        WheelUpdate(chassis->wheels[index], dt_s);
    chassis->data.update_count++;
    ChassisSyncData(chassis);
}

/**
 * @brief   复位底盘里程
 * @param   chassis 底盘对象指针
 */
void ChassisResetOdometry(Chassis_t *chassis)
{
    uint8_t index;

    if ((chassis == NULL) || (!chassis->initialized))
        return;

    for (index = 0U; index < chassis->data.wheel_count; ++index)
        WheelResetOdometry(chassis->wheels[index]);

    ChassisSyncData(chassis);
}

/**
 * @brief   获取底盘运行数据
 * @param   chassis 底盘对象指针
 * @param   data 数据输出目标
 */
void ChassisGetData(Chassis_t *chassis, ChassisData_t *data)
{
    if ((chassis == NULL) || (data == NULL))
        return;

    if (chassis->initialized)
        ChassisSyncData(chassis);

    memcpy(data, &chassis->data, sizeof(*data));
}

/**
 * @brief   绑定底盘对象方法
 * @param   chassis 底盘对象指针
 */
static void ChassisBindMethods(Chassis_t *chassis)
{
    if (chassis == NULL)
        return;

    chassis->init              = ChassisInit;
    chassis->start             = ChassisStart;
    chassis->stop              = ChassisStop;
    chassis->set_velocity      = ChassisSetVelocity;
    chassis->set_forward_speed = ChassisSetForwardSpeed;
    chassis->set_turn_speed    = ChassisSetTurnSpeed;
    chassis->update            = ChassisUpdate;
    chassis->reset_odometry    = ChassisResetOdometry;
    chassis->get_data          = ChassisGetData;
}

/**
 * @brief   校验底盘初始化配置
 * @param   config 底盘初始化配置
 * @return  bool 合法返回true，否则返回false
 */
static bool ChassisConfigIsValid(const ChassisInitConfig_t *config)
{
    float forward_sum = 0.0f;
    float cross_sum = 0.0f;
    float turn_sum = 0.0f;
    float determinant;
    uint8_t index;
    uint8_t compare_index;

    if ((config == NULL) || (config->wheels == NULL) ||
        (config->wheel_kinematics == NULL) || (config->wheel_count == 0U))
    {
        return false;
    }

    for (index = 0U; index < config->wheel_count; ++index)
    {
        if ((config->wheels[index] == NULL) || (!config->wheels[index]->initialized))
            return false;

        for (compare_index = index + 1U; compare_index < config->wheel_count; ++compare_index)
        {
            if (config->wheels[index] == config->wheels[compare_index])
                return false;
        }

        forward_sum += config->wheel_kinematics[index].forward_coefficient *
                       config->wheel_kinematics[index].forward_coefficient;
        cross_sum += config->wheel_kinematics[index].forward_coefficient *
                     config->wheel_kinematics[index].turn_coefficient_m;
        turn_sum += config->wheel_kinematics[index].turn_coefficient_m *
                    config->wheel_kinematics[index].turn_coefficient_m;
    }

    determinant = forward_sum * turn_sum - cross_sum * cross_sum;
    return ChassisAbs(determinant) > CHASSIS_FLOAT_EPSILON;
}

/**
 * @brief   清除底盘速度指令
 * @param   chassis 底盘对象指针
 */
static void ChassisClearVelocityCommand(Chassis_t *chassis)
{
    if (chassis == NULL)
        return;

    chassis->data.target_forward_speed_mps = 0.0f;
    chassis->data.target_turn_speed_radps = 0.0f;
    chassis->data.command_forward_speed_mps = 0.0f;
    chassis->data.command_turn_speed_radps = 0.0f;
    chassis->data.wheel_speed_scale = 1.0f;
}

/**
 * @brief   同步底盘运行数据
 * @param   chassis 底盘对象指针
 */
static void ChassisSyncData(Chassis_t *chassis)
{
    WheelData_t wheel_data;
    float forward_sum = 0.0f;
    float cross_sum = 0.0f;
    float turn_sum = 0.0f;
    float forward_feedback_sum = 0.0f;
    float turn_feedback_sum = 0.0f;
    float forward_distance_sum = 0.0f;
    float turn_distance_sum = 0.0f;
    float forward_coefficient;
    float turn_coefficient;
    float determinant;
    bool all_wheels_enabled = true;
    uint8_t index;

    if ((chassis == NULL) || (!chassis->initialized))
        return;

    for (index = 0U; index < chassis->data.wheel_count; ++index)
    {
        WheelGetData(chassis->wheels[index], &wheel_data);
        if (!wheel_data.enabled)
            all_wheels_enabled = false;

        forward_coefficient = chassis->wheel_kinematics[index].forward_coefficient;
        turn_coefficient = chassis->wheel_kinematics[index].turn_coefficient_m;
        forward_sum += forward_coefficient * forward_coefficient;
        cross_sum += forward_coefficient * turn_coefficient;
        turn_sum += turn_coefficient * turn_coefficient;
        forward_feedback_sum += forward_coefficient * wheel_data.linear_speed_mps;
        turn_feedback_sum += turn_coefficient * wheel_data.linear_speed_mps;
        forward_distance_sum += forward_coefficient * wheel_data.distance_m;
        turn_distance_sum += turn_coefficient * wheel_data.distance_m;
    }

    determinant = forward_sum * turn_sum - cross_sum * cross_sum;
    chassis->data.forward_speed_mps = 0.0f;
    chassis->data.turn_speed_radps = 0.0f;
    chassis->data.distance_m = 0.0f;
    chassis->data.turn_angle_rad = 0.0f;
    if (ChassisAbs(determinant) > CHASSIS_FLOAT_EPSILON)
    {
        chassis->data.forward_speed_mps =
            (forward_feedback_sum * turn_sum - turn_feedback_sum * cross_sum) / determinant;
        chassis->data.turn_speed_radps =
            (turn_feedback_sum * forward_sum - forward_feedback_sum * cross_sum) / determinant;
        chassis->data.distance_m =
            (forward_distance_sum * turn_sum - turn_distance_sum * cross_sum) / determinant;
        chassis->data.turn_angle_rad =
            (turn_distance_sum * forward_sum - forward_distance_sum * cross_sum) / determinant;
    }

    chassis->data.enabled = all_wheels_enabled;
}

/**
 * @brief   获取浮点数绝对值
 * @param   value 输入值
 * @return  float 绝对值
 */
static float ChassisAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}
