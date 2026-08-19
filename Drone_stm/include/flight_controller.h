#ifndef __FLIGHT_CONTROLLER_H
#define __FLIGHT_CONTROLLER_H
 
#include "stm32f1xx_hal.h"
#include <stdint.h>
 
/**
 * @brief Clamp và ghi xung PWM trực tiếp vào thanh ghi CCR của Timer 2
 *        Dùng int16_t để clamp đúng khi giá trị âm từ mixer
 * @param m1..m4: Độ rộng xung (1000us - 2000us)
 */
void Motor_Outputs(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
 
/**
 * @brief Mixer Quad-X: trộn throttle + PID 3 trục → 4 motor
 * @param throttle  : Ga tổng (1000 - 2000)
 * @param pid_roll  : Output PID trục Roll
 * @param pid_pitch : Output PID trục Pitch
 * @param pid_yaw   : Output PID trục Yaw
 */
void Flight_Controller_Update(uint16_t throttle,
                               float pid_roll,
                               float pid_pitch,
                               float pid_yaw);
 
#endif /* __FLIGHT_CONTROLLER_H */
 
