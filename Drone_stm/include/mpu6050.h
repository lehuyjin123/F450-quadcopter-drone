#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "i2c.h"
#include <stdint.h>

/* =========================
   I2C ADDRESS
   AD0 = GND → 0x68 → dịch trái 1 bit = 0xD0
========================= */
#define MPU6050_ADDR            0xD0

/* =========================
   REGISTER MAP
========================= */
#define MPU6050_REG_SMPLRT_DIV      0x19    // Sample Rate Divider
#define MPU6050_REG_CONFIG           0x1A    // DLPF config
#define MPU6050_REG_GYRO_CONFIG      0x1B    // Gyro full-scale range
#define MPU6050_REG_ACCEL_CONFIG     0x1C    // Accel full-scale range
#define MPU6050_REG_ACCEL_XOUT_H     0x3B    // Accel + Temp + Gyro (14 bytes liên tục)
#define MPU6050_REG_GYRO_XOUT_H      0x43    // Gyro X high byte
#define MPU6050_REG_PWR_MGMT_1       0x6B    // Power management

/* =========================
   SENSITIVITY CONSTANTS
   Accel FS = ±2g   → 16384 LSB/g
   Gyro  FS = ±250°/s → 131 LSB/°/s
========================= */
#define ACCEL_SENSITIVITY       16384.0f
#define GYRO_SENSITIVITY        65.5f

/* =========================
   ANGLE CONVERSION
   Tách ra header để tránh define trùng với file .c
========================= */
#define RAD_TO_DEG              57.295779513f
#define DEG_TO_RAD              0.0174532925f

/* =========================
   COMPLEMENTARY FILTER WEIGHT
   0.98 gyro + 0.02 accel
========================= */
#define COMP_FILTER_ALPHA       0.98f



/**
 * @brief Cấu trúc dữ liệu MPU6050
 */
typedef struct {
    // Raw ADC (sau khi trừ offset)
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    // Accel đã lọc low-pass (đơn vị: g)
    float ax, ay, az;

    // Gyro đã convert (đơn vị: °/s)
    float gx, gy, gz;

    // Góc ước tính từ Complementary Filter (đơn vị: °)
    float roll;
    float pitch;

    // Offset calibration
    int32_t ax_offset, ay_offset, az_offset;
    int32_t gx_offset, gy_offset, gz_offset;

    // Hệ số low-pass filter cho accel (0.0 → mượt, 1.0 → raw)
    float filter_alpha;

    // Flag đánh dấu lần đầu tính góc để bỏ qua dt bất thường
    uint8_t angle_initialized;
} MPU6050_t;

/* =========================
   FUNCTION PROTOTYPES
========================= */

/**
 * @brief Khởi tạo MPU6050: wake up, set sample rate, set DLPF, set FS range
 */
uint8_t MPU6050_Init(MPU6050_t *pDevice, I2C_HandleTypeDef *hi2c);

/**
 * @brief Calibration: drone phải đặt phẳng, không rung
 *        Lấy trung bình 1000 mẫu để tính offset
 */
void MPU6050_Calibrate(MPU6050_t *pDevice, I2C_HandleTypeDef *hi2c);

/**
 * @brief Đọc toàn bộ dữ liệu Accel + Gyro trong một burst read 14 bytes
 *        Áp dụng offset, low-pass filter cho accel, convert gyro sang °/s
 */
uint8_t MPU6050_Read_All(MPU6050_t *pDevice, I2C_HandleTypeDef *hi2c);

/**
 * @brief Tính góc Roll/Pitch bằng Complementary Filter
 *        Phải gọi sau MPU6050_Read_All() mỗi chu kỳ
 */
void MPU6050_Calculate_Angles(MPU6050_t *pDevice, float dt);

#endif /* __MPU6050_H */