#include "control.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* =========================
   PID INSTANCES
   Tune gains tại đây
========================= */

// Tầng 1 — Angle controller
// Output: rate setpoint (°/s) cho tầng 2
// Hạ Ki xuống 0.05 để tránh xung đột với nhiễu rung
PID_t pid_angle_roll  = {
    .Kp = 3.0f, .Ki = 0.05f, .Kd = 0.0f, .Kf = 0.0f,
    .max_integral = 120.0f, .max_output = 200.0f, // 60 độ/s là đủ để hover mượt
    .d_filter_alpha = 0.0f
};
PID_t pid_angle_pitch = {
    .Kp = 3.0f, .Ki = 0.05f, .Kd = 0.0f, .Kf = 0.0f,
    .max_integral = 120.0f, .max_output = 200.0f,
    .d_filter_alpha = 0.0f
};

// Tầng 2 — Rate controller
// Input: gyro °/s, Output: µs correction cho motor
// Đây là nơi tune chính
PID_t pid_rate_roll  = {
    .Kp = 2.5f, .Ki = 0.02f, .Kd = 0.08f, .Kf = 0.0f, // Giảm Kp để giảm biên độ dao động
    .max_integral = 150.0f, .max_output = 200.0f,    // Giảm max_output để tránh chiếm dụng Throttle
    .d_filter_alpha = 0.2f
};
PID_t pid_rate_pitch = {
    .Kp = 2.5f, .Ki = 0.02f, .Kd = 0.08f, .Kf = 0.0f,
    .max_integral = 150.0f, .max_output = 200.0f,
    .d_filter_alpha = 0.2f
};
PID_t pid_rate_yaw   = {
    .Kp = 2.8f, .Ki = 0.05f, .Kd = 0.05f, .Kf = 0.0f,
    .max_integral = 120.0f, .max_output = 200.0f,
    .d_filter_alpha = 0.1f
};

/* =========================
   STATE
========================= */
float    Target_Roll     = 0.0f;
float    Target_Pitch    = 0.0f;
float    Target_Yaw_Rate = 0.0f;
uint16_t Base_Throttle   = THROTTLE_MIN;
float    Last_Out_Roll   = 0.0f;
float    Last_Out_Pitch  = 0.0f;
float    Last_Out_Yaw    = 0.0f;

uint8_t  Pid_Active      = 1;

static uint32_t lastThrottleTime = 0;
static uint32_t lastPacketTime   = 0;
static uint8_t  controlArmed     = 0;

#define CTRL_RX_FAILSAFE_US 3000000 // Tăng lên s để tránh reset do trễ tín hiệu

static int clamp_int(int v, int min_v, int max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void control_stop(char *current_cmd_out)
{
    Target_Roll     = 0.0f;
    Target_Pitch    = 0.0f;
    Target_Yaw_Rate = 0.0f;
    Base_Throttle   = THROTTLE_MIN;
    Last_Out_Roll   = 0.0f;
    Last_Out_Pitch  = 0.0f;
    Last_Out_Yaw    = 0.0f;
    controlArmed    = 0;
    Pid_Active       = 1;
    Control_Reset_All_PID();
    Motor_Outputs(1000, 1000, 1000, 1000);
    if (current_cmd_out) {
        strncpy(current_cmd_out, "STOP", 31);
        current_cmd_out[31] = '\0';
    }

    
}

/* =========================
   CONTROL INIT
========================= */
void Control_Init(void)
{
    Target_Roll     = 0.0f;
    Target_Pitch    = 0.0f;
    Target_Yaw_Rate = 0.0f;
    Base_Throttle   = THROTTLE_MIN;
    Last_Out_Roll   = 0.0f;
    Last_Out_Pitch  = 0.0f;
    Last_Out_Yaw    = 0.0f;
    lastThrottleTime = 0;
    lastPacketTime   = 0;
    controlArmed     = 0;
    Pid_Active       = 1;
    Control_Reset_All_PID();
}

/* =========================
   RESET ALL PID
========================= */
void Control_Reset_All_PID(void)
{
    PID_Reset(&pid_angle_roll);
    PID_Reset(&pid_angle_pitch);
    PID_Reset(&pid_rate_roll);
    PID_Reset(&pid_rate_pitch);
    PID_Reset(&pid_rate_yaw);
}

uint8_t Control_Is_Armed(void)
{
    return controlArmed;
}

/**
 * @brief Hàm parse nhanh số nguyên từ chuỗi (Trả về NULL nếu chuỗi lỗi/thiếu số)
 * Tối ưu hóa: Thay thế sscanf, triệt tiêu hoàn toàn Jitter cho vòng lặp 1kHz
 */
static const char* fast_atoi(const char* p, int* res) {
    if (!p) return NULL;
    while (*p == ' ') p++;
    if (*p == '\0') return NULL; 
    
    int x = 0, f = 1;
    if (*p == '-') { f = -1; p++; }
    
    if (*p < '0' || *p > '9') return NULL; // Ký tự không hợp lệ
    
    while (*p >= '0' && *p <= '9') {
        x = x * 10 + (*p - '0');
        p++;
    }
    *res = x * f;
    return p;
}

/* =========================
   UPDATE CMD → SETPOINT
========================= */
void Control_Update_Cmd(const char *cmd, uint32_t now_us, char *current_cmd_out, uint8_t is_new_cmd)
{
    if (is_new_cmd) {
        lastPacketTime = now_us;
    }

    // 1. VÁ LỖI FAILSAFE: Đưa ra ngoài cùng để luôn luôn kiểm tra mất sóng
    if ((now_us - lastPacketTime) > CTRL_RX_FAILSAFE_US)
    {
        control_stop(current_cmd_out);
        return; // Ngắt ngay lập tức, không cho phép đọc các lệnh bên dưới
    }

    // 2. XỬ LÝ LỆNH ĐIỀU KHIỂN LIÊN TỤC (TỐI ƯU BẰNG FAST_ATOI)
    if (strncmp(cmd, "CTRL ", 5) == 0)
    {
        if (is_new_cmd) 
        {
            int throttle, roll_x100, pitch_x100, yaw_dps, arm, pid_act;
            const char* p = cmd + 5;
            
            // Chỉ cập nhật nếu gói tin chuẩn, đủ cả 6 tham số
            if ((p = fast_atoi(p, &throttle)) &&
                (p = fast_atoi(p, &roll_x100)) &&
                (p = fast_atoi(p, &pitch_x100)) &&
                (p = fast_atoi(p, &yaw_dps)) &&
                (p = fast_atoi(p, &arm)) &&
                (p = fast_atoi(p, &pid_act))) 
            {
                Base_Throttle   = (uint16_t)clamp_int(throttle, THROTTLE_MIN, THROTTLE_MAX);
                Target_Roll     = (float)clamp_int(roll_x100,  -1000, 1000) / 100.0f;
                Target_Pitch    = (float)clamp_int(pitch_x100, -1000, 1000) / 100.0f;
                Target_Yaw_Rate = (float)clamp_int(yaw_dps,    -90,   90);
                controlArmed    = arm ? 1 : 0;
                Pid_Active      = pid_act ? 1 : 0;
            }
        }
        return;
    }

    // 3. XỬ LÝ CÁC LỆNH ĐƠN (FORWARD, BACK, LEFT, RIGHT...)
    if (strcmp(cmd, "FORWARD") == 0)
    {
        controlArmed = 1;
        Target_Pitch = TARGET_PITCH_FWD;
        Target_Roll  = 0.0f;
        Target_Yaw_Rate = 0.0f;
    }
    else if (strcmp(cmd, "BACK") == 0)
    {
        controlArmed = 1;
        Target_Pitch = -TARGET_PITCH_FWD;
        Target_Roll  = 0.0f;
        Target_Yaw_Rate = 0.0f;
    }
    else if (strcmp(cmd, "LEFT") == 0)
    {
        controlArmed = 1;
        Target_Roll  = -TARGET_ROLL_SIDE;
        Target_Pitch = 0.0f;
        Target_Yaw_Rate = 0.0f;
    }
    else if (strcmp(cmd, "RIGHT") == 0)
    {
        controlArmed = 1;
        Target_Roll  = TARGET_ROLL_SIDE;
        Target_Pitch = 0.0f;
        Target_Yaw_Rate = 0.0f;
    }
    else if (strcmp(cmd, "YAW_LEFT") == 0)
    {
        controlArmed = 1;
        Target_Yaw_Rate = -TARGET_YAW_RATE;
        Target_Roll  = 0.0f;
        Target_Pitch = 0.0f;
    }
    else if (strcmp(cmd, "YAW_RIGHT") == 0)
    {
        controlArmed = 1;
        Target_Yaw_Rate = TARGET_YAW_RATE;
        Target_Roll  = 0.0f;
        Target_Pitch = 0.0f;
    }
    else if (strcmp(cmd, "T_UP") == 0)
    {
        controlArmed = 1;
        if ((now_us - lastThrottleTime) >= THROTTLE_INTERVAL_US)
        {
            if (Base_Throttle < THROTTLE_MAX)
                Base_Throttle += THROTTLE_STEP_UP;
            lastThrottleTime = now_us;
        }
    }
    else if (strcmp(cmd, "T_DOWN") == 0)
    {
        if ((now_us - lastThrottleTime) >= THROTTLE_INTERVAL_US)
        {
            if (Base_Throttle > THROTTLE_MIN)
                Base_Throttle -= THROTTLE_STEP_DOWN;
            lastThrottleTime = now_us;
        }
    }
    else if (strcmp(cmd, "HOLD") == 0)
    {
        Target_Roll     = 0.0f;
        Target_Pitch    = 0.0f;
        Target_Yaw_Rate = 0.0f;
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        control_stop(current_cmd_out);
    }
}

/* =========================
   EMERGENCY CHECK
========================= */
uint8_t Control_Emergency(MPU6050_t *imu, char *current_cmd_out)
{
    if (imu->roll  >  EMERGENCY_ANGLE || imu->roll  < -EMERGENCY_ANGLE ||
        imu->pitch >  EMERGENCY_ANGLE || imu->pitch < -EMERGENCY_ANGLE)
    {
        Base_Throttle = THROTTLE_MIN;
        controlArmed = 0; // Ngắt cờ hiệu ngay lập tức
        Motor_Outputs(1000, 1000, 1000, 1000);
        Control_Reset_All_PID();
        Target_Roll     = 0.0f;
        Target_Pitch    = 0.0f;
        Target_Yaw_Rate = 0.0f;
        // Reset cả các out trung gian để tầng Rate không bị giật
        Last_Out_Roll = 0; Last_Out_Pitch = 0; Last_Out_Yaw = 0;
        strncpy(current_cmd_out, "STOP", 31);
        return 1;
    }
    return 0;
}

/* =========================
   RUN PID
========================= */
void Control_Run_Rate_PID(MPU6050_t *imu, float dt, float sp_roll, float sp_pitch)
{
    // HOÀN TOÀN BYPASS NẾU TẮT PID TỪ GIAO DIỆN
    if (Pid_Active == 0) 
    {
        // Liên tục xóa I-term để tránh hiện tượng tích tụ sai số (windup) khi bật lại
        pid_angle_roll.integral  = 0;
        pid_angle_pitch.integral = 0;
        pid_rate_roll.integral   = 0;
        pid_rate_pitch.integral  = 0;
        pid_rate_yaw.integral    = 0;

        // Ép dữ liệu hiển thị telemetry về 0
        Last_Out_Roll  = 0.0f;
        Last_Out_Pitch = 0.0f;
        Last_Out_Yaw   = 0.0f;

        // Chỉ xuất ga thô đồng đều cho cả 4 motor
        Flight_Controller_Update(Base_Throttle, 0.0f, 0.0f, 0.0f);
        return; // Thoát sớm, không chạy đoạn tính PID bên dưới
    }

    // ── ĐOẠN CODE TÍNH PID GỐC KHI PID_ACTIVE == 1 (Giữ nguyên) ──
    if (Base_Throttle < THROTTLE_TAKEOFF_THRESHOLD) {
        pid_angle_roll.integral  = 0;
        pid_angle_pitch.integral = 0;
        pid_rate_roll.integral   = 0;
        pid_rate_pitch.integral  = 0;
        pid_rate_yaw.integral    = 0;
    }

    float out_roll  = PID_Compute(&pid_rate_roll,  sp_roll,  imu->gx, dt);
    float out_pitch = PID_Compute(&pid_rate_pitch, sp_pitch, imu->gy, dt);
    float out_yaw   = PID_Compute(&pid_rate_yaw,   Target_Yaw_Rate, imu->gz, dt);

    Last_Out_Roll  = out_roll;
    Last_Out_Pitch = out_pitch;
    Last_Out_Yaw   = out_yaw;

    Flight_Controller_Update(
        Base_Throttle,
        ROLL_PID_DIR  * out_roll,
        PITCH_PID_DIR * out_pitch,
        YAW_PID_DIR   * out_yaw
    );
}
