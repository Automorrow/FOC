#include "lowpass_filter.h"

// ��ʼ���˲���
LowPassFilter init_low_pass_filter(double tau, double dt) {
    double alpha = dt / (tau + dt);
    LowPassFilter filter = {alpha, 0.0}; // �����ʼ���Ϊ0
    return filter;
}

// �����˲���
double update_low_pass_filter(LowPassFilter *filter, double input) {
    double y_new = filter->alpha * input + (1 - filter->alpha) * filter->y_prev;
    filter->y_prev = y_new;
    return y_new;
}

int demo_low_pass_filter() {
    double tau = 0.1; // ʱ�䳣��
    double dt = 0.01; // ��������
    LowPassFilter my_filter = init_low_pass_filter(tau, dt);

    // ģ����������
    double input_data[] = {1.0, 1.5, 2.0, 1.8, 1.7, 1.6, 1.5, 1.4, 1.3, 1.2};
    int n_samples = sizeof(input_data) / sizeof(input_data[0]);

    // ����ÿ����������
    for (int i = 0; i < n_samples; i++) {
        double output = update_low_pass_filter(&my_filter, input_data[i]);
        printf("Input: %.2f, Output: %.2f\n", input_data[i], output);
    }

    return 0;
}