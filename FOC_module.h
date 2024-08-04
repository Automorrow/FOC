#ifndef __FOC_MODULE__
#define __FOC_MODULE__

#include "pt32x031.h"
#include "tick.h"
#include "lowpass_filter.h"
#include "pid.h"
#include <math.h>

#define u32 uint32_t
#define u16 uint16_t
#define u8  uint8_t
#define s32 int32_t
#define s16 int16_t
#define s8  int8_t

#ifndef PI
  #define PI               3.14159265358979f
#endif
#define _constrain(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#ifndef container_of
  #define container_of(ptr, type, member) ((type *)((char *)(ptr) - (unsigned long)(&(((type *)0)->member))))
#endif

enum {
    PHASE_A,
    PHASE_B,
    PHASE_C,
    PHASE_NUM
};
typedef struct {
    u8 pwm_channel;
    float phase_voltage;
    float phase_current;
    void *parents_alpha_beta;
}FOC_module_phase_t;
u8 FOC_module_phase_voltage_set(FOC_module_phase_t *phase_abc_ptr, float phase_voltage);
u8 FOC_module_phase_voltage_init(FOC_module_phase_t *phase_abc_ptr, void *parents_alpha_beta, u8 pwm_channel);

typedef struct FOC_module_alpha_beta_coordinate_t{
    FOC_module_phase_t phase_voltage[PHASE_NUM];
    float U_alpha;
    float U_beta;
}FOC_module_alpha_beta_coordinate_t;
u8 FOC_module_alpha_beta_coordinate_set(FOC_module_alpha_beta_coordinate_t *alpha_beta_ptr, float U_alpha, float U_beta);
u8 FOC_module_alpha_beta_coordinate_init(FOC_module_alpha_beta_coordinate_t *alpha_beta_ptr, u8 *pwm_channel);

typedef struct {
    FOC_module_alpha_beta_coordinate_t alpha_beta;
    float U_d;
    float U_q;
}FOC_module_DQ_coordinate_t;
u8 FOC_module_DQ_coordinate_set(FOC_module_DQ_coordinate_t *dq_ptr, float U_d, float U_q);
u8 FOC_module_DQ_coordinate_get(FOC_module_DQ_coordinate_t *dq_ptr, float *U_d, float *U_q);
u8 FOC_module_DQ_coordinate_init(FOC_module_DQ_coordinate_t *dq_ptr, u8 *pwm_channel, u8 *adc_channel);

typedef struct {
    float Kp;
}FOC_module_sensorless_SMO_t;
typedef struct {
    float Kp;
}FOC_module_sensorless_SMO_config_t;
u8 FOC_module_sensorless_SMO_get_velocity_and_angle(FOC_module_sensorless_SMO_t *SMO_ptr, float *velocity, float *angle);
u8 FOC_module_sensorless_SMO_loop(FOC_module_sensorless_SMO_t *SMO_ptr);
u8 FOC_module_sensorless_SMO_init(FOC_module_sensorless_SMO_t *SMO_ptr, FOC_module_sensorless_SMO_config_t *SMO_config_ptr);

typedef struct {
    float Kp;
}FOC_module_sensorless_EKF_t;
typedef struct {
    float Kp;
}FOC_module_sensorless_EKF_config_t;
u8 FOC_module_sensorless_EKF_get_velocity_and_angle(FOC_module_sensorless_EKF_t *EKF_ptr, float *velocity, float *angle);
u8 FOC_module_sensorless_EKF_loop(FOC_module_sensorless_EKF_t *EKF_ptr);
u8 FOC_module_sensorless_EKF_init(FOC_module_sensorless_EKF_t *EKF_ptr, FOC_module_sensorless_EKF_config_t *EKF_config_ptr);
typedef enum {
    sliding_mode_observer,
    extended_kalman,
}FOC_module_observer_type_t;
enum {
    FOC_module_observer_return_ok,
    FOC_module_observer_return_wait,
    FOC_module_observer_return_error,
};
typedef struct {
    float shaft_angle;
    float shaft_velocity;
    float I_alpha;
    float I_beta;
}FOC_module_sensorless_observer_return_t;
typedef struct {
    FOC_module_observer_type_t type;
    union {
        FOC_module_sensorless_SMO_t SMO;
        FOC_module_sensorless_EKF_t EKF;
    };
    u32 timestamp;
}FOC_module_sensorless_observer_t;
u8 FOC_module_sensorless_observer_get_velocity_and_angle(FOC_module_sensorless_observer_t *sensorless_observer_ptr, float *velocity, float *angle);
u8 FOC_module_sensorless_observer_loop(FOC_module_sensorless_observer_t *sensorless_observer_ptr);
u8 FOC_module_sensorless_observer_init(FOC_module_sensorless_observer_t *sensorless_observer_ptr, FOC_module_observer_type_t type, void *config_ptr);

typedef struct {
    CMSDK_PWM_TypeDef *pwm;
    u32 max_duty_value;
    u32 min_duty_value;

    float voltage_limit;//10v
    float voltage_power_supply;//12.6v
    float shaft_angle;//机械角度
    float shaft_velocity;//机械速度
    float pole_pairs;//极对数
    float open_loop_timestamp;
    float zero_electric_angle;
    u32 KV;
    float phase_resistance;
    float phase_inductance;

    FOC_module_DQ_coordinate_t dq;
    FOC_module_sensorless_observer_t observer;
    PID_t velocity_pid;
    PID_t angle_pid;
    PID_t Uq_pid;
    //PID_t Ud_pid;
}FOC_module_t;
typedef struct {
    CMSDK_PWM_TypeDef *pwm;
    u8 pwm_channel[PHASE_NUM];
    u32 max_duty_value;
    u32 min_duty_value;
    float voltage_limit;
    float voltage_power_supply;
    float pole_pairs;
    float zero_electric_angle;
    u32 KV;
    FOC_module_observer_type_t observer_type;
    union {
        FOC_module_sensorless_SMO_config_t SMO_config;
        FOC_module_sensorless_EKF_config_t EKF_config;
    };
}FOC_module_config_t;
u8 FOC_module_init(FOC_module_t *foc_module_ptr, FOC_module_config_t *foc_module_config_ptr);
u8 FOC_module_open_loop(FOC_module_t *foc_module_ptr, float target_velocity);
u8 FOC_module_close_loop(FOC_module_t *foc_module_ptr, float target_velocity);






#endif
