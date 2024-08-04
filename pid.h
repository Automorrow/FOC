#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// PID 控制器结构体
typedef struct {
    float kp;          // 比例增益
    float ki;          // 积分增益
    float kd;          // 微分增益
    float integral;    // 积分项
    float prev_error;  // 上一次误差
    float min_output;  // 最小输出值
    float max_output;  // 最大输出值
} PID_t;
typedef struct {
    float kp;
    float ki;
    float kd;
    float min_output;
    float max_output;
} PID_config_t;
/**
 * @brief 初始化 PID 控制器
 * @param pid     PID 控制器结构体指针
 * @param kp      比例增益
 * @param ki      积分增益
 * @param kd      微分增益
 * @param min_out 最小输出值
 * @param max_out 最大输出值
 */
void PID_Init(PID_t *pid, PID_config_t *config);

/**
 * @brief 更新 PID 控制器
 * @param pid     PID 控制器结构体指针
 * @param setpoint 目标值
 * @param process_value 当前过程值
 * @return         PID 输出值
 */
float PID_Update(PID_t *pid, float setpoint, float process_value);

#endif // PID_H_
