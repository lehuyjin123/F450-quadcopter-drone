#include "mpu6050.h"
#include <math.h>
#include <string.h>

/* =========================
   DEBUG UART
   Comment dòng dưới để tắt hoàn toàn log UART
   khi build bản release/flight
========================= */
#define MPU6050_DEBUG_UART
#ifdef MPU6050_DEBUG_UART
    extern UART_HandleTypeDef huart1;
    #define MPU_LOG(msg, len) HAL_UART_Transmit(&huart1, (uint8_t*)(msg), (len), HAL_MAX_DELAY)
#else
    #define MPU_LOG(msg, len)   // Tắt hoàn toàn, không tốn flash
#endif

/* =========================
   MPU6050 INIT
========================= */
uint8_t MPU6050_Init(MPU6050_t *pDevice, I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // Reset struct về 0
    memset(pDevice, 0, sizeof(MPU6050_t));
    pDevice->filter_alpha      = 0.6f;   // Low-pass alpha cho accel
    pDevice->angle_initialized = 0;      // Chưa khởi tạo góc

    // --- 1. Wake up: xóa SLEEP bit ---
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1,
                          1, &data, 1, 100) != HAL_OK)
    {
        MPU_LOG("MPU6050 WAKE UP FAIL\r\n", 21);
        return HAL_ERROR;
    }
    HAL_Delay(100);

    // --- 2. Set sample rate = 200 Hz ---
    // Formula: Sample Rate = Internal_Rate / (1 + SMPLRT_DIV)
    // 1000Hz / (1 + 4) = 200Hz
    data = 0x04;  // SMPLRT_DIV = 4
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV,
                          1, &data, 1, 100) != HAL_OK)
    {
        MPU_LOG("MPU6050 SMPRT FAIL\r\n", 20);
        return HAL_ERROR;
    }

    // --- 3. Set DLPF = 21 Hz bandwidth (DLPF_CFG = 4) ---
    // Giảm noise cao tần từ rung động motor
    data = 0x04;  // DLPF_CFG = 4 (21 Hz) - Giúp lọc nhiễu rung tốt hơn rất nhiều
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_CONFIG,
                          1, &data, 1, 100) != HAL_OK)
    {
        MPU_LOG("MPU6050 DLPF FAIL\r\n", 19);
        return HAL_ERROR;
    }

    // --- 4. Set Gyro FS = ±500°/s (GYRO_FS_SEL = 0) ---
    data = 0x08;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG,
                          1, &data, 1, 100) != HAL_OK)
    {
        MPU_LOG("MPU6050 GYRO CFG FAIL\r\n", 22);
        return HAL_ERROR;
    }

    // --- 5. Set Accel FS = ±2g (ACCEL_FS_SEL = 0) ---
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG,
                          1, &data, 1, 100) != HAL_OK)
    {
        MPU_LOG("MPU6050 ACCEL CFG FAIL\r\n", 23);
        return HAL_ERROR;
    }

    MPU_LOG("MPU6050 READY\r\n", 15);
    return HAL_OK;
}

/* =========================
   CALIBRATION
   Drone phải đặt phẳng, không rung trong quá trình này
========================= */
void MPU6050_Calibrate(MPU6050_t *pDevice, I2C_HandleTypeDef *hi2c)
{
    uint8_t  data[14];
    int32_t  sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t  sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t valid_count = 0;

    MPU_LOG("CALIBRATING...\r\n", 16);

    for (int i = 0; i < 1000; i++)
    {
        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                             1, data, 14, 2) == HAL_OK)
        {
            sum_ax += (int16_t)(data[0]  << 8 | data[1]);
            sum_ay += (int16_t)(data[2]  << 8 | data[3]);
            sum_az += (int16_t)(data[4]  << 8 | data[5]);
            // data[6..7] = TEMP, bỏ qua
            sum_gx += (int16_t)(data[8]  << 8 | data[9]);
            sum_gy += (int16_t)(data[10] << 8 | data[11]);
            sum_gz += (int16_t)(data[12] << 8 | data[13]);
            valid_count++;
        }
        HAL_Delay(2);
    }

    // Tránh chia cho 0 nếu I2C lỗi toàn bộ
    if (valid_count == 0) {
        MPU_LOG("CALIB FAIL: no data\r\n", 21);
        return;
    }

    pDevice->ax_offset = sum_ax / valid_count;
    pDevice->ay_offset = sum_ay / valid_count;
    // Trừ 1g (16384 LSB) trên trục Z vì khi nằm ngang az đọc ra = 1g
    pDevice->az_offset = (sum_az / valid_count) - (int32_t)ACCEL_SENSITIVITY;

    pDevice->gx_offset = sum_gx / valid_count;
    pDevice->gy_offset = sum_gy / valid_count;
    pDevice->gz_offset = sum_gz / valid_count;

    MPU_LOG("CALIB DONE\r\n", 12);
}

/* =========================
   READ ALL (Accel + Gyro, burst 14 bytes)
   Đổi tên từ MPU6050_Read_Accel → MPU6050_Read_All
   vì hàm đọc cả gyro lẫn accel
========================= */
uint8_t MPU6050_Read_All(MPU6050_t *pDevice, I2C_HandleTypeDef *hi2c)
{
    uint8_t data[14];

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                         1, data, 14, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // --- Raw ADC ---
    int16_t ax_raw = (int16_t)(data[0]  << 8 | data[1]);
    int16_t ay_raw = (int16_t)(data[2]  << 8 | data[3]);
    int16_t az_raw = (int16_t)(data[4]  << 8 | data[5]);
    // data[6..7] = TEMP, bỏ qua
    int16_t gx_raw = (int16_t)(data[8]  << 8 | data[9]);
    int16_t gy_raw = (int16_t)(data[10] << 8 | data[11]);
    int16_t gz_raw = (int16_t)(data[12] << 8 | data[13]);

    // --- Trừ offset ---
    pDevice->raw_ax = ax_raw - (int16_t)pDevice->ax_offset;
    pDevice->raw_ay = ay_raw - (int16_t)pDevice->ay_offset;
    pDevice->raw_az = az_raw - (int16_t)pDevice->az_offset;
    pDevice->raw_gx = gx_raw - (int16_t)pDevice->gx_offset;
    pDevice->raw_gy = gy_raw - (int16_t)pDevice->gy_offset;
    pDevice->raw_gz = gz_raw - (int16_t)pDevice->gz_offset;

    // --- Accel convert → g rồi low-pass filter ---
    // Dựa trên phần cứng: Y-Front, X-Right
    // Pitch (nghiêng mũi) xoay quanh trục X của chip, Roll xoay quanh trục Y của chip
    float cur_ax = (float)pDevice->raw_ax / ACCEL_SENSITIVITY; 
    float cur_ay = (float)pDevice->raw_ay / ACCEL_SENSITIVITY;
    float cur_az = (float)pDevice->raw_az / ACCEL_SENSITIVITY;

    pDevice->ax = cur_ax;
    pDevice->ay = cur_ay;
    pDevice->az = cur_az;

    // Map Gyro: Roll rate = Gyro Y, Pitch rate = Gyro X
    pDevice->gx = (float)pDevice->raw_gy / GYRO_SENSITIVITY; 
    pDevice->gy = (float)pDevice->raw_gx / GYRO_SENSITIVITY;
    pDevice->gz = (float)pDevice->raw_gz / GYRO_SENSITIVITY;

    return HAL_OK;
}

/* =========================
   CALCULATE ANGLES
   Complementary Filter: 98% Gyro + 2% Accel
========================= */
void MPU6050_Calculate_Angles(MPU6050_t *pDevice, float dt)
{
    if (pDevice->angle_initialized == 0)
    {
        // Với Y-Front, X-Right: ay liên quan Pitch, ax liên quan Roll
        pDevice->roll  = -atan2f(pDevice->ax,
                          sqrtf(pDevice->ay * pDevice->ay +
                                 pDevice->az * pDevice->az)) * RAD_TO_DEG;
        pDevice->pitch = atan2f(pDevice->ay, 
                          sqrtf(pDevice->ax * pDevice->ax +
                                 pDevice->az * pDevice->az)) * RAD_TO_DEG;
        pDevice->angle_initialized = 1;
        return;
    }

    if (dt <= 0.000001f || dt > 0.1f) return;

    // Tính toán độ lớn lực G tổng cộng (bình phương để tránh sqrtf nếu không cần thiết)
    float total_g_sq = pDevice->ax * pDevice->ax + pDevice->ay * pDevice->ay + pDevice->az * pDevice->az;

    /* 
       Nếu IMU lệch tâm, khi xoay mạnh total_g_sq sẽ biến động.
       Ta thu hẹp vùng tin tưởng Accel lại một chút (0.9G đến 1.1G) 
       để tránh Lever Arm Effect làm sai lệch góc.
       Nới rộng ngưỡng tin tưởng gia tốc lên một chút để tránh việc bỏ rơi Accel hoàn toàn khi bay.
    */
    if (total_g_sq > 0.5f && total_g_sq < 1.5f) {
        float accel_roll  = -atan2f(pDevice->ax, sqrtf(pDevice->ay * pDevice->ay + pDevice->az * pDevice->az)) * RAD_TO_DEG;
        float accel_pitch = atan2f(pDevice->ay, sqrtf(pDevice->ax * pDevice->ax + pDevice->az * pDevice->az)) * RAD_TO_DEG;

        // Điều chỉnh alpha về 0.99: Cân bằng giữa mượt mà và khả năng giữ chân trời (horizon)
        pDevice->pitch = 0.99f * (pDevice->pitch + pDevice->gy * dt) + 0.01f * accel_pitch;
        pDevice->roll  = 0.99f * (pDevice->roll  + pDevice->gx * dt) + 0.01f * accel_roll;
    } else {
        // Nếu đang rung quá mạnh (>1.5G), chỉ tin hoàn toàn vào Gyro
        pDevice->pitch += pDevice->gy * dt;
        pDevice->roll  += pDevice->gx * dt;
    }

}
