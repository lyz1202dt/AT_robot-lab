#include "leg_driver/kalman_filter.hpp"
#include <cmath>

KalmanFilter::KalmanFilter(float process_noise, 
                           float measurement_noise,
                           float initial_estimate,
                           float initial_error)
    : x_est(initial_estimate)
    , P(initial_error)
    , Q(process_noise)
    , R(measurement_noise)
    , K(0.0f)
    , initialized(false) {
}

float KalmanFilter::update(float measurement) {
    // 如果是第一次更新，使用测量值初始化
    if (!initialized) {
        x_est = measurement;
        initialized = true;
        return x_est;
    }

    // 预测步骤
    // 状态预测: x_pred = x_est (对于恒定模型)
    float x_pred = x_est;
    // 误差协方差预测: P_pred = P + Q
    float P_pred = P + Q;

    // 更新步骤
    // 计算卡尔曼增益: K = P_pred / (P_pred + R)
    K = P_pred / (P_pred + R);
    
    // 状态更新: x_est = x_pred + K * (measurement - x_pred)
    x_est = x_pred + K * (measurement - x_pred);
    
    // 误差协方差更新: P = (1 - K) * P_pred
    P = (1.0f - K) * P_pred;

    return x_est;
}

void KalmanFilter::reset(float initial_estimate) {
    x_est = initial_estimate;
    P = 1.0f;
    K = 0.0f;
    initialized = false;
}
