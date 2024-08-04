#include "PID.h"
#include <math.h> // for fabsf

void PID_Init(PID_t *pid, PID_config_t *config) {
    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->min_output = config->min_output;
    pid->max_output = config->max_output;
}

float PID_Update(PID_t *pid, float setpoint, float process_value) {
    float error = setpoint - process_value;
    pid->integral += error;

    // 防止积分饱和
    if (pid->integral > pid->max_output) {
        pid->integral = pid->max_output;
    } else if (pid->integral < pid->min_output) {
        pid->integral = pid->min_output;
    }

    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // 限制输出
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }

    return output;
}

