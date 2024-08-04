#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// PID �������ṹ��
typedef struct {
    float kp;          // ��������
    float ki;          // ��������
    float kd;          // ΢������
    float integral;    // ������
    float prev_error;  // ��һ�����
    float min_output;  // ��С���ֵ
    float max_output;  // ������ֵ
} PID_t;
typedef struct {
    float kp;
    float ki;
    float kd;
    float min_output;
    float max_output;
} PID_config_t;
/**
 * @brief ��ʼ�� PID ������
 * @param pid     PID �������ṹ��ָ��
 * @param kp      ��������
 * @param ki      ��������
 * @param kd      ΢������
 * @param min_out ��С���ֵ
 * @param max_out ������ֵ
 */
void PID_Init(PID_t *pid, PID_config_t *config);

/**
 * @brief ���� PID ������
 * @param pid     PID �������ṹ��ָ��
 * @param setpoint Ŀ��ֵ
 * @param process_value ��ǰ����ֵ
 * @return         PID ���ֵ
 */
float PID_Update(PID_t *pid, float setpoint, float process_value);

#endif // PID_H_
