#ifndef __PID_H
#define __PID_H

// Không cần include HAL ở đây vì PID là module logic thuần, không dùng bất kỳ API nào của STM32
#include <stdint.h>

/**
 * @brief Cấu trúc dữ liệu cho một bộ PID độc lập
 */
typedef struct {
    // --- Hệ số PID ---
    float Kp;               // Hệ số Tỷ lệ  (Proportional)
    float Ki;               // Hệ số Tích phân (Integral)
    float Kd;               // Hệ số Vi phân (Derivative)
    float Kf;               // Hệ số Feedforward

    // --- Trạng thái nội bộ ---
    float integral;         // Giá trị tích phân cộng dồn
    float prev_error;       // Sai số của chu kỳ trước (dùng cho D-term)
    float prev_value;       // Giá trị cảm biến chu kỳ trước (Derivative on Measurement)
    float prev_setpoint;    // Giá trị setpoint chu kỳ trước
    float prev_derivative;  // Giá trị derivative chu kỳ trước (dùng cho low-pass filter)

    // --- Giới hạn ---
    float max_integral;     // Giới hạn anti-windup cho tích phân
    float max_output;       // Giới hạn lực can thiệp tối đa của PID

    // --- D-term Low-pass filter ---
    // Hệ số lọc derivative: 0.0 = chỉ lấy mẫu mới, 1.0 = không lọc
    // Khuyến nghị: 0.1f - 0.3f để giảm noise gyro
    float d_filter_alpha;
} PID_t;

/**
 * @brief Tính toán một chu kỳ PID
 * @param pid           : Con trỏ tới cấu trúc PID
 * @param setpoint      : Giá trị mục tiêu (góc mong muốn)
 * @param current_value : Giá trị thực tế đọc từ cảm biến
 * @param dt            : Thời gian chu kỳ tính bằng giây (ví dụ: 0.004f = 250Hz)
 * @return              : Lực can thiệp của PID (đã giới hạn trong max_output)
 */
float PID_Compute(PID_t *pid, float setpoint, float current_value, float dt);

/**
 * @brief Reset toàn bộ trạng thái nội bộ của bộ PID về 0
 *        Gọi hàm này khi: arm/disarm, chuyển mode bay, hoặc sau khi mất tín hiệu
 * @param pid : Con trỏ tới cấu trúc PID cần reset
 */
void PID_Reset(PID_t *pid);

#endif /* __PID_H */