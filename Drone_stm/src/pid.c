#include "pid.h"

/**
 * @brief Reset toàn bộ trạng thái nội bộ của bộ PID về 0
 *        Gọi khi arm/disarm, chuyển mode, hoặc sau khi mất tín hiệu RC
 */

void PID_Reset(PID_t *pid)
{
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->prev_value      = 0.0f; 
    pid->prev_setpoint   = 0.0f;
}

/**
 * @brief Tính toán một chu kỳ PID
 */
float PID_Compute(PID_t *pid,
                  float setpoint,
                  float current_value,
                  float dt)
{
    // Bảo vệ chia cho 0 hoặc dt bất thường
    // FIX: Phải cập nhật prev_error trước khi return để tránh D-term spike
    // ở chu kỳ kế tiếp khi dt trở lại bình thường
    if (dt <= 0.0001f)
    {
        return 0.0f;
    }

    float error = setpoint - current_value;

    // --- Mượn từ PID_CAL_YAW: Xử lý bước nhảy góc (chỉ dùng cho Angle Yaw nếu cần) ---
    // Nếu bạn điều khiển Angle Yaw, hãy uncomment đoạn dưới
    /*
    if (error > 180.0f)  error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    */

    /* ======================
       P TERM
    ====================== */
    float p_term = pid->Kp * error;

    /* ======================
       I TERM (Anti-windup)
    ====================== */
    float i_term = 0.0f;
    if (pid->Ki > 0.0f) {
        pid->integral += error * dt;
        if (pid->integral >  pid->max_integral) pid->integral =  pid->max_integral;
        if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
        i_term = pid->Ki * pid->integral;
    }

    /* ======================
       FF TERM
    ====================== */
    float ff_term = 0.0f;
    if (pid->Kf > 0.0f) {
        float setpoint_rate = (setpoint - pid->prev_setpoint) / dt;
        ff_term = pid->Kf * setpoint_rate;
    }

    /* ======================
       D TERM (Derivative on Measurement)
    ====================== */
    float d_term = 0.0f;
    if (pid->Kd > 0.0f) {
        float raw_derivative = -(current_value - pid->prev_value) / dt;
        float current_derivative = raw_derivative;
        
        if (pid->d_filter_alpha > 0.0f && pid->d_filter_alpha < 1.0f) {
            current_derivative = pid->d_filter_alpha * raw_derivative
                       + (1.0f - pid->d_filter_alpha) * pid->prev_derivative;
        }
        d_term = pid->Kd * current_derivative;
        pid->prev_derivative = current_derivative;
    }

    // Lưu lại giá trị cho chu kỳ kế tiếp
    // Cập nhật trạng thái
    pid->prev_error = error;
    pid->prev_value = current_value;
    pid->prev_setpoint = setpoint;

    /* ======================
       TOTAL OUTPUT
    ====================== */
    float output = p_term + i_term + d_term + ff_term;

    // Clamp output về giới hạn an toàn
    if (output >  pid->max_output) output =  pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;

    return output;
}
