#include "flight_controller.h"
#include "tim.h"
#include "control.h"

extern TIM_HandleTypeDef htim2;

/**
 * @brief Clamp và ghi xung PWM vào Timer 2
 *        Dùng int16_t để clamp đúng khi giá trị âm từ mixer
 */
void Motor_Outputs(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    if (m1 < 1000) m1 = 1000;
    if (m1 > 2000) m1 = 2000;
    if (m2 < 1000) m2 = 1000;
    if (m2 > 2000) m2 = 2000;
    if (m3 < 1000) m3 = 1000;
    if (m3 > 2000) m3 = 2000;
    if (m4 < 1000) m4 = 1000;
    if (m4 > 2000) m4 = 2000;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)m1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)m2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)m3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)m4);
}

/**
 * @brief Mixer Quad-X — Config B (Betaflight standard)
 *
 * MOTOR MAPPING (nhìn từ trên xuống):
 *
 *        [FL]  ^  [FR]
 *          \   |   /
 *           -------
 *          /   |   \
 *        [RL]     [RR]
 *
 *   FL (Front Left)  = TIM_CHANNEL_2 = PA1  → CCW  (ngược chiều kim đồng hồ)
 *   FR (Front Right) = TIM_CHANNEL_1 = PA0  → CW   (thuận chiều kim đồng hồ)
 *   RL (Rear Left)   = TIM_CHANNEL_3 = PA2  → CW
 *   RR (Rear Right)  = TIM_CHANNEL_4 = PA3  → CCW
 *
 * YAW LOGIC:
 *   Tăng CW (FR+RL) → drone xoay phải (+yaw)
 *   Tăng CCW (FL+RR) → drone xoay trái (-yaw)
 *   → CW motor nhận -yaw, CCW motor nhận +yaw
 */
void Flight_Controller_Update(
    uint16_t throttle,
    float    pid_roll,
    float    pid_pitch,
    float    pid_yaw
)
{
    if (throttle <= THROTTLE_IDLE_CUTOFF)  // An toàn: Ngắt mixer hoàn toàn ở ga thấp
    {
        Motor_Outputs(1000, 1000, 1000, 1000);
        return;
    }

    int16_t thr   = (int16_t)throttle;
    int16_t roll  = (int16_t)pid_roll;
    int16_t pitch = (int16_t)pid_pitch;
    int16_t yaw   = (int16_t)pid_yaw;

    //            throttle    pitch   roll   yaw
    int16_t motor_FL = thr + pitch + roll - yaw;  
    int16_t motor_FR = thr + pitch - roll + yaw;  
    int16_t motor_RL = thr - pitch + roll + yaw; 
    int16_t motor_RR = thr - pitch - roll - yaw;  

    // Tìm giá trị motor lớn nhất để cân bằng lại nếu vượt ngưỡng
    int16_t max_motor = motor_FL;
    if (motor_FR > max_motor) max_motor = motor_FR;
    if (motor_RL > max_motor) max_motor = motor_RL;
    if (motor_RR > max_motor) max_motor = motor_RR;

    // Giảm ga tổng để nhường chỗ cho PID cân bằng (Mixer Scaling)
    if (max_motor > 2000) { 
        int16_t reduction = max_motor - 2000;
        motor_FL -= reduction;
        motor_FR -= reduction;
        motor_RL -= reduction;
        motor_RR -= reduction;
    }

    // Đảm bảo motor không bao giờ thấp hơn mức tối thiểu khi đang bay
    if (motor_FL < 1000) motor_FL = 1000;
    if (motor_FR < 1000) motor_FR = 1000;
    if (motor_RL < 1000) motor_RL = 1000;
    if (motor_RR < 1000) motor_RR = 1000;

    Motor_Outputs(
        motor_FR,   // CH1 → PA0 → Front Right (CW)
        motor_FL,   // CH2 → PA1 → Front Left  (CCW)
        motor_RL,   // CH3 → PA2 → Rear Left   (CW)
        motor_RR    // CH4 → PA3 → Rear Right  (CCW)
    );
}
