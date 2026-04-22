#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

/**
 * @brief 单通道卡尔曼滤波器类
 * 用于对电机力矩进行滤波处理，减少测量噪声
 */
class KalmanFilter {
public:
    /**
     * @brief 构造函数
     * @param process_noise 过程噪声协方差 Q (默认 0.001)
     * @param measurement_noise 测量噪声协方差 R (默认 0.1)
     * @param initial_estimate 初始估计值 (默认 0.0)
     * @param initial_error 初始误差协方差 (默认 1.0)
     */
    KalmanFilter(float process_noise = 0.001f, 
                 float measurement_noise = 0.1f,
                 float initial_estimate = 0.0f,
                 float initial_error = 1.0f);

    /**
     * @brief 更新卡尔曼滤波器，返回滤波后的值
     * @param measurement 测量值（原始力矩值）
     * @return 滤波后的估计值
     */
    float update(float measurement);

    /**
     * @brief 重置滤波器状态
     * @param initial_estimate 初始估计值
     */
    void reset(float initial_estimate = 0.0f);

    /**
     * @brief 获取当前估计值
     * @return 当前力矩估计值
     */
    float getEstimate() const { return x_est; }

    /**
     * @brief 设置过程噪声协方差 Q
     * @param process_noise 新的过程噪声协方差值
     */
    void setProcessNoise(float process_noise) { Q = process_noise; }

    /**
     * @brief 设置测量噪声协方差 R
     * @param measurement_noise 新的测量噪声协方差值
     */
    void setMeasurementNoise(float measurement_noise) { R = measurement_noise; }

    /**
     * @brief 获取当前 Q 值
     * @return 过程噪声协方差 Q
     */
    float getProcessNoise() const { return Q; }

    /**
     * @brief 获取当前 R 值
     * @return 测量噪声协方差 R
     */
    float getMeasurementNoise() const { return R; }

private:
    float x_est;    // 状态估计值（力矩估计）
    float P;        // 误差协方差
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float K;        // 卡尔曼增益
    bool initialized; // 是否已初始化
};

#endif // __KALMAN_FILTER_HPP__
