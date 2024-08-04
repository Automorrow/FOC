#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

// �����˲����ṹ��
typedef struct {
    double alpha; // ƽ������
    double y_prev; // ��һ�ε����ֵ
} LowPassFilter;

// ��ʼ���˲���
LowPassFilter init_low_pass_filter(double tau, double dt);

// �����˲���
double update_low_pass_filter(LowPassFilter *filter, double input);

#endif // LOWPASS_FILTER_H