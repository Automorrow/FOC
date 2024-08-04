#include "FOC_module.h"

float FOC_module_normalize_angle(float angle) {
    angle = fmod(angle, 2 * PI);
    if (angle < 0) {
        angle += 2 * PI;
    }
    return angle;
}
float FOC_module_electric_angle_calculate(FOC_module_t *foc_module_ptr) {
    return foc_module_ptr->shaft_angle * foc_module_ptr->pole_pairs;
}

u8 FOC_module_phase_voltage_init(FOC_module_phase_t *phase_abc_ptr, void *parents_alpha_beta, u8 pwm_channel) {
    phase_abc_ptr->phase_current = 0;
    phase_abc_ptr->phase_voltage = 0;
    phase_abc_ptr->pwm_channel = pwm_channel;
    phase_abc_ptr->parents_alpha_beta = parents_alpha_beta;
    return 0;
}
u8 FOC_module_phase_voltage_set(FOC_module_phase_t *phase_abc_ptr, float phase_voltage) {
    u32 duty_value;
    FOC_module_DQ_coordinate_t *parents_dq = container_of((FOC_module_alpha_beta_coordinate_t *)(phase_abc_ptr->parents_alpha_beta), FOC_module_DQ_coordinate_t, alpha_beta);
    FOC_module_t *foc_module_ptr = container_of(parents_dq, FOC_module_t, dq);
    phase_voltage = _constrain(phase_voltage, 0.0f, foc_module_ptr->voltage_limit);
    phase_abc_ptr->phase_voltage = phase_voltage;
    duty_value = _constrain(phase_voltage * foc_module_ptr->max_duty_value / foc_module_ptr->voltage_power_supply, foc_module_ptr->min_duty_value, foc_module_ptr->max_duty_value);
    //phase_abc_ptr->phase_current = 
    PWM_SetCompare(foc_module_ptr->pwm, phase_abc_ptr->pwm_channel, duty_value);
    return 0;
}

u8 FOC_module_alpha_beta_coordinate_init(FOC_module_alpha_beta_coordinate_t *alpha_beta_ptr, u8 *pwm_channel) {
    alpha_beta_ptr->U_alpha = 0;
    alpha_beta_ptr->U_beta = 0;
    FOC_module_phase_voltage_init(alpha_beta_ptr->phase_voltage + PHASE_A, alpha_beta_ptr, pwm_channel[PHASE_A]);
    FOC_module_phase_voltage_init(alpha_beta_ptr->phase_voltage + PHASE_B, alpha_beta_ptr, pwm_channel[PHASE_B]);
    FOC_module_phase_voltage_init(alpha_beta_ptr->phase_voltage + PHASE_C, alpha_beta_ptr, pwm_channel[PHASE_C]);
    return 0;
}
u8 FOC_module_alpha_beta_coordinate_set(FOC_module_alpha_beta_coordinate_t *alpha_beta_ptr, float U_alpha, float U_beta) {
    float phase_voltage_a, phase_voltage_b, phase_voltage_c;
    FOC_module_DQ_coordinate_t *parents_dq_ptr = container_of(alpha_beta_ptr, FOC_module_DQ_coordinate_t, alpha_beta);
    FOC_module_t *foc_module_ptr = container_of(parents_dq_ptr, FOC_module_t, dq);
    alpha_beta_ptr->U_alpha = U_alpha;
    alpha_beta_ptr->U_beta = U_beta;
    //计算三相电压
    // + foc_module_ptr->voltage_power_supply / 2   // 平移曲线到供电电压的中间，避免出现负数电压
    phase_voltage_a = U_alpha + foc_module_ptr->voltage_power_supply / 2;
    phase_voltage_b = (sqrt(3) * U_beta - U_alpha) / 2 + foc_module_ptr->voltage_power_supply / 2;
    phase_voltage_c = (- U_alpha - sqrt(3) * U_beta) / 2 + foc_module_ptr->voltage_power_supply / 2;
    FOC_module_phase_voltage_set(alpha_beta_ptr->phase_voltage + PHASE_A, phase_voltage_a);
    FOC_module_phase_voltage_set(alpha_beta_ptr->phase_voltage + PHASE_B, phase_voltage_b);
    FOC_module_phase_voltage_set(alpha_beta_ptr->phase_voltage + PHASE_C, phase_voltage_c);
    return 0;
}
u8 FOC_module_DQ_coordinate_init(FOC_module_DQ_coordinate_t *dq_ptr, u8 *pwm_channel) {
    dq_ptr->U_d = 0;
    dq_ptr->U_q = 0;
    FOC_module_alpha_beta_coordinate_init(&dq_ptr->alpha_beta, pwm_channel);
    return 0;
}

u8 FOC_module_DQ_coordinate_set(FOC_module_DQ_coordinate_t *dq_ptr, float U_d, float U_q) {
    float U_alpha, U_beta;
    FOC_module_t *foc_module_ptr = container_of(dq_ptr, FOC_module_t, dq);
    //printf("%s foc_module_ptr 0x%x\n", __func__, foc_module_ptr);
    float electric_angle = FOC_module_electric_angle_calculate(foc_module_ptr);
    dq_ptr->U_d = U_d;
    dq_ptr->U_q = U_q;
    U_alpha = U_d * cosf(electric_angle) - U_q * sinf(electric_angle);
    U_beta  = U_d * sinf(electric_angle) + U_q * cosf(electric_angle);
    FOC_module_alpha_beta_coordinate_set(&dq_ptr->alpha_beta, U_alpha, U_beta);
    return 0;
}

u8 FOC_module_init(FOC_module_t *foc_module_ptr, FOC_module_config_t *foc_module_config_ptr) {
    //printf("%s foc_module_ptr 0x%x\n", __func__, foc_module_ptr);
    foc_module_ptr->pwm = foc_module_config_ptr->pwm;
    foc_module_ptr->max_duty_value = foc_module_config_ptr->max_duty_value;
    foc_module_ptr->min_duty_value = foc_module_config_ptr->min_duty_value;
    foc_module_ptr->voltage_limit = foc_module_config_ptr->voltage_limit;
    foc_module_ptr->voltage_power_supply = foc_module_config_ptr->voltage_power_supply;
    foc_module_ptr->pole_pairs = foc_module_config_ptr->pole_pairs;
    foc_module_ptr->zero_electric_angle = foc_module_config_ptr->zero_electric_angle;
    FOC_module_DQ_coordinate_init(&foc_module_ptr->dq, foc_module_config_ptr->pwm_channel);
	return 0;
}

u8 FOC_module_sensorless_SMO_get_velocity_and_angle(FOC_module_sensorless_SMO_t *SMO_ptr, float *velocity, float *angle) {

}
u8 FOC_module_sensorless_SMO_loop(FOC_module_sensorless_SMO_t *SMO_ptr) {

}
u8 FOC_module_sensorless_SMO_init(FOC_module_sensorless_SMO_t *SMO_ptr, FOC_module_sensorless_SMO_config_t *SMO_config_ptr) {

}
u8 FOC_module_sensorless_EKF_get_velocity_and_angle(FOC_module_sensorless_EKF_t *EKF_ptr, float *velocity, float *angle) {

}
u8 FOC_module_sensorless_EKF_loop(FOC_module_sensorless_EKF_t *EKF_ptr) {

}
u8 FOC_module_sensorless_EKF_init(FOC_module_sensorless_EKF_t *EKF_ptr, FOC_module_sensorless_EKF_config_t *EKF_config_ptr) {

}
u8 FOC_module_sensorless_observer_get_velocity_and_angle(FOC_module_sensorless_observer_t *sensorless_observer_ptr, float *velocity, float *angle, u32 *wait_time_us) {
    u8 ret;
    switch (sensorless_observer_ptr->type) {
        case sliding_mode_observer: {
            ret = FOC_module_sensorless_SMO_get_velocity_and_angle(&sensorless_observer_ptr->SMO, velocity, angle);
        }break;
        case extended_kalman: {
            ret = FOC_module_sensorless_EKF_get_velocity_and_angle(&sensorless_observer_ptr->EKF, velocity, angle);
        }break;
        default: return -1;
    }
    if (FOC_module_observer_return_ok == ret) {
        u32 timestamp = clock_time();
        *wait_time_us = timestamp - sensorless_observer_ptr->timestamp;
        sensorless_observer_ptr->timestamp = timestamp;
    }
    return ret;
}
u8 FOC_module_sensorless_observer_loop(FOC_module_sensorless_observer_t *sensorless_observer_ptr) {
    switch (sensorless_observer_ptr->type) {
        case sliding_mode_observer: {
            FOC_module_sensorless_SMO_loop(&sensorless_observer_ptr->SMO);
        }break;
        case extended_kalman: {
            FOC_module_sensorless_EKF_loop(&sensorless_observer_ptr->EKF);
        }break;
        default: return -1;
    }
}
u8 FOC_module_sensorless_observer_init(FOC_module_sensorless_observer_t *sensorless_observer_ptr, FOC_module_observer_type_t type, void *config_ptr){
    switch (sensorless_observer_ptr->type) {
        case sliding_mode_observer: {
            FOC_module_sensorless_SMO_init(&sensorless_observer_ptr->SMO, (FOC_module_sensorless_SMO_config_t *)config_ptr);
        }break;
        case extended_kalman: {
            FOC_module_sensorless_EKF_init(&sensorless_observer_ptr->EKF, (FOC_module_sensorless_EKF_config_t *)config_ptr);
        }break;
        default: return -1;
    }
}
/**
 * 开环控制启动FOC模块
 * 
 * 本函数用于在开环控制模式下初始化FOC模块，设定电机的目标速度，并根据时间更新电机的角度。
 * 不需要反馈控制，直接根据目标速度计算电机角度的变化。
 * 
 * @param foc_module_ptr 指向FOC模块结构体的指针，包含模块的配置和状态信息。
 * @param target_velocity 目标速度，单位为弧度/秒。
 * @return 返回0，表示函数执行成功。
 */
u8 FOC_module_open_loop(FOC_module_t *foc_module_ptr, float target_velocity) {
    float per_v = 30 * target_velocity / (PI * foc_module_ptr->KV);
    // 获取当前时间戳
    u32 tick = clock_time();
    
    // 计算时间差，并转换为秒为单位的时间间隔
    float Ts = (tick - foc_module_ptr->open_loop_timestamp) * 1e-6f;

    // 确保时间间隔在合理范围内，避免出现负值或过大的时间间隔
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // 根据目标速度和时间间隔更新电机轴的角度
    foc_module_ptr->shaft_angle = FOC_module_normalize_angle(foc_module_ptr->shaft_angle + target_velocity*Ts);

    // 在DQ坐标系中设定电压限制，初始化为0，幅值为模块的电压限制
    FOC_module_DQ_coordinate_set(&foc_module_ptr->dq, 0, foc_module_ptr->voltage_limit/12);

    // 更新开环控制的时间戳
    foc_module_ptr->open_loop_timestamp = tick;

    // 函数执行成功，返回0
    return 0;
	
}

u8 FOC_module_close_loop(FOC_module_t *foc_module_ptr, float target_velocity) {
    u8 ret;
    float angle_pid_output;
    float velocity_pid_output;
    float Uq_pid_output;
    //float Ud_pid_output;
    u32 wait_time_us;
    float prev_angle = foc_module_ptr->shaft_angle;
    ret = FOC_module_sensorless_observer_get_velocity_and_angle(&foc_module_ptr->sensorless_observer, &foc_module_ptr->shaft_velocity, &foc_module_ptr->shaft_angle, &wait_time_us);
    if (ret != FOC_module_observer_return_ok) return ret;
    angle_pid_output = PID_Update(&foc_module_ptr->angle_pid, prev_angle + target_velocity * wait_time_us * 1e-6f, foc_module_ptr->shaft_angle);
    velocity_pid_output = PID_Update(&foc_module_ptr->velocity_pid, angle_pid_output, foc_module_ptr->shaft_velocity);
    Uq_pid_output = PID_Update(&foc_module_ptr->Uq_pid, velocity_pid_output, foc_module_ptr->shaft_velocity);
    
    return 0;
}


