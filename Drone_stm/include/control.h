#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include "mpu6050.h"
#include "flight_controller.h"
#include <stdint.h>

/* =========================
   THROTTLE CONFIG
========================= */
#define THROTTLE_MIN          1000
#define THROTTLE_MAX          2000
#define THROTTLE_STEP_UP        5
#define THROTTLE_STEP_DOWN      10
#define THROTTLE_INTERVAL_US 100000   // 100ms giữa mỗi bước throttle
#define THROTTLE_IDLE_CUTOFF  1200    // Dưới mức này motor dừng hẳn
#define THROTTLE_TAKEOFF_THRESHOLD 1650 // Ngưỡng bắt đầu chạy I-term (gần điểm cất cánh)

/* =========================
   SETPOINT CONFIG
========================= */
#define TARGET_PITCH_FWD     5.0f    // độ nghiêng khi FORWARD
#define TARGET_ROLL_SIDE     5.0f    // độ nghiêng khi LEFT/RIGHT
#define TARGET_YAW_RATE     30.0f    // °/s khi YAW

/* =========================
   EMERGENCY CUTOFF
========================= */
#define EMERGENCY_ANGLE     70.0f    // độ — tự động STOP khi vượt ngưỡng

/* =========================
   PID DIRECTION SIGNS
   Đổi dấu ở đây nếu drone phản ứng ngược chiều
   +1.0f = giữ nguyên, -1.0f = đảo ngược
========================= */
#define ROLL_PID_DIR    1.0f
#define PITCH_PID_DIR   1.0f
#define YAW_PID_DIR     1.0f

/* =========================
   EXTERN PID — có thể tune từ ngoài
========================= */
extern PID_t pid_angle_roll;
extern PID_t pid_angle_pitch;
extern PID_t pid_rate_roll;
extern PID_t pid_rate_pitch;
extern PID_t pid_rate_yaw;

/* =========================
   EXTERN STATE
========================= */
extern float    Target_Roll;
extern float    Target_Pitch;
extern float    Target_Yaw_Rate;
extern uint16_t Base_Throttle;
extern float    Last_Out_Roll;
extern float    Last_Out_Pitch;
extern float    Last_Out_Yaw;

/* =========================
   FUNCTION PROTOTYPES
========================= */

/**
 * @brief Khởi tạo giá trị mặc định cho control module
 */
void Control_Init(void);

/**
 * @brief Xử lý lệnh từ UART → cập nhật setpoint và throttle
 * @param cmd       : chuỗi lệnh hiện tại (FORWARD, BACK, T_UP, STOP, ...)
 * @param now_us    : thời gian hiện tại (micros) để throttle ramp
 * @param current_cmd_out : con trỏ để hàm có thể ghi "STOP" khi emergency
 */
void Control_Update_Cmd(const char *cmd, uint32_t now_us, char *current_cmd_out, uint8_t is_new_cmd);

/**
 * @brief Kiểm tra góc nguy hiểm — tự động cut motor nếu lật quá ngưỡng
 * @return 1 nếu emergency triggered (caller nên continue), 0 nếu bình thường
 */
uint8_t Control_Emergency(MPU6050_t *imu, char *current_cmd_out);

/**
 * @brief Tầng Rate PID (Inner Loop) - Chạy ở tần số cao nhất (1kHz)
 * @param imu      : Dữ liệu gyro
 * @param dt       : Thời gian chu kỳ
 * @param sp_roll  : Setpoint tốc độ góc từ tầng Angle
 * @param sp_pitch : Setpoint tốc độ góc từ tầng Angle
 */
void Control_Run_Rate_PID(MPU6050_t *imu, float dt, float sp_roll, float sp_pitch);

/**
 * @brief Reset toàn bộ PID — gọi khi STOP hoặc emergency
 */
void Control_Reset_All_PID(void);

/**
 * @brief Kiểm tra drone có đang ở trạng thái hoạt động (Armed) hay không
 * @return 1 nếu Armed, 0 nếu Disarmed/STOP
 */
uint8_t Control_Is_Armed(void);

#endif /* __CONTROL_H */
