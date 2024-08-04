#include "lowpass_filter.h"

// 初始化滤波器
LowPassFilter init_low_pass_filter(double tau, double dt) {
    double alpha = dt / (tau + dt);
    LowPassFilter filter = {alpha, 0.0}; // 假设初始输出为0
    return filter;
}

// 更新滤波器
double update_low_pass_filter(LowPassFilter *filter, double input) {
    double y_new = filter->alpha * input + (1 - filter->alpha) * filter->y_prev;
    filter->y_prev = y_new;
    return y_new;
}

int demo_low_pass_filter() {
    double tau = 0.1; // 时间常数
    double dt = 0.01; // 采样周期
    LowPassFilter my_filter = init_low_pass_filter(tau, dt);

    // 模拟输入数据
    double input_data[] = {1.0, 1.5, 2.0, 1.8, 1.7, 1.6, 1.5, 1.4, 1.3, 1.2};
    int n_samples = sizeof(input_data) / sizeof(input_data[0]);

    // 处理每个输入样本
    for (int i = 0; i < n_samples; i++) {
        double output = update_low_pass_filter(&my_filter, input_data[i]);
        printf("Input: %.2f, Output: %.2f\n", input_data[i], output);
    }

    return 0;
}