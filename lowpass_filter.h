#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

// 定义滤波器结构体
typedef struct {
    double alpha; // 平滑因子
    double y_prev; // 上一次的输出值
} LowPassFilter;

// 初始化滤波器
LowPassFilter init_low_pass_filter(double tau, double dt);

// 更新滤波器
double update_low_pass_filter(LowPassFilter *filter, double input);

#endif // LOWPASS_FILTER_H