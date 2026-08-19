#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "control.h"       // thay thế cho pid.h + flight_controller.h

/* =========================
   DWT TIMER (microsecond timing)
========================= */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t micros_DWT(void)
{
    // Tối ưu: Sử dụng hằng số đã biết hoặc tính toán một lần
    // Với 72MHz, SystemCoreClock / 1000000U = 72
    return DWT->CYCCNT / 72U; 
}

/* =========================
   CONFIG
========================= */
#define LOOP_PERIOD_US        1000   // 1kHz
#define TELEMETRY_INTERVAL_US 600000 // 1Hz
#define ANGLE_LOOP_DIVIDER    1      // Tầng Angle chạy ở 250Hz (1kHz / 4)

/* =========================
   GLOBALS
========================= */
MPU6050_t myIMU;
char      uart_msg[128];

extern UART_HandleTypeDef huart1;

uint8_t          rx_data;
char             rx_buffer[32];
uint8_t          rx_index        = 0;
volatile char    pending_cmd[32] = "";
volatile uint8_t cmd_ready       = 0;
char             current_cmd[32] = "STOP";

uint32_t lastLoopTime      = 0;
uint32_t lastTelemetryTime = 0;
uint32_t loopCounter       = 0;
uint32_t max_exec_time     = 0; // Biến giám sát hiệu năng (us)

// Biến trung gian để truyền Setpoint từ tầng Angle xuống tầng Rate
float rate_setpoint_roll = 0, rate_setpoint_pitch = 0;

void SystemClock_Config(void);

/* =========================
   MAIN
========================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    DWT_Init();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    // Bật LED PB14 để báo hiệu đang trong quá trình khởi động/setup
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

    HAL_Delay(200);
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    HAL_Delay(200);

    // Khởi động PWM + ARM ESC
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    Motor_Outputs(1000, 1000, 1000, 1000);
    HAL_Delay(3000);

    // Init MPU6050
    if (MPU6050_Init(&myIMU, &hi2c1) != HAL_OK)
    {
        HAL_UART_Transmit(&huart1, (uint8_t*)"MPU INIT FAIL\r\n", 15, HAL_MAX_DELAY);
        while (1);
    }
    MPU6050_Calibrate(&myIMU, &hi2c1);

    // Tắt LED PB14 báo hiệu đã setup xong, hệ thống sẵn sàng
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_UART_Transmit(&huart1, (uint8_t*)"SYSTEM READY\r\n", 14, HAL_MAX_DELAY);

    // Init control module
    Control_Init();

    lastLoopTime      = micros_DWT();
    lastTelemetryTime = micros_DWT();

    while (1)
    {
        // ── TIMING ──
        uint32_t now_us = micros_DWT();
        uint32_t elapsed = now_us - lastLoopTime;
        if (elapsed < LOOP_PERIOD_US) continue;

        uint32_t start_exec = micros_DWT(); // Bắt đầu đo thời gian xử lý

        // Tối ưu: Dùng phép nhân thay cho phép chia (1/1000000 = 0.000001)
        float dt = (float)elapsed * 0.000001f;
        lastLoopTime = now_us;

        // ── NHẬN LỆNH AN TOÀN ──
        uint8_t new_cmd_received = 0;
        if (cmd_ready)
        {
            __disable_irq();
            strncpy(current_cmd, (const char*)pending_cmd, sizeof(current_cmd) - 1);
            current_cmd[sizeof(current_cmd) - 1] = '\0';
            cmd_ready = 0;
            __enable_irq();
            new_cmd_received = 1;
        }

        // ── XỬ LÝ LỆNH → CẬP NHẬT TARGET ──
        Control_Update_Cmd(current_cmd, now_us, current_cmd, new_cmd_received);

        // ── ĐỌC CẢM BIẾN ──
        if (MPU6050_Read_All(&myIMU, &hi2c1) == HAL_OK)
        {
            // Tối ưu: Kiểm tra biến cờ hiệu nhanh hơn so sánh chuỗi strcmp rất nhiều
            if (Control_Is_Armed())
            {
                // ── KIỂM TRA KHẨN CẤP (AN TOÀN) ──
                if (Control_Emergency(&myIMU, current_cmd)) 
                {
                    // Nếu khẩn cấp, cập nhật exec_time rồi mới skip
                    uint32_t e = micros_DWT() - start_exec;
                    if (e > max_exec_time) max_exec_time = e;
                    continue;
                }

                loopCounter++;

                // Tầng Angle (Tính toán góc + PID tầng 1) chạy chậm hơn
                if (loopCounter % ANGLE_LOOP_DIVIDER == 0)
                {
                    float angle_dt = dt * ANGLE_LOOP_DIVIDER;
                    MPU6050_Calculate_Angles(&myIMU, angle_dt);

                    // Tính Setpoint cho tầng Rate dựa trên sai số góc (Stability Mode)
                    rate_setpoint_roll  = PID_Compute(&pid_angle_roll,  Target_Roll,  myIMU.roll,  angle_dt);
                    rate_setpoint_pitch = PID_Compute(&pid_angle_pitch, Target_Pitch, myIMU.pitch, angle_dt);
                }

                // Tầng Rate (PID tầng 2) + Motor luôn chạy ở 1kHz
                Control_Run_Rate_PID(&myIMU, dt, rate_setpoint_roll, rate_setpoint_pitch);
            }
            else
            {
                // Đảm bảo motor dừng hẳn khi Disarmed
                Motor_Outputs(1000, 1000, 1000, 1000);
            }
        }

        // Tính toán thời gian thực thi tối đa để đảm bảo an toàn cho vòng lặp 1000us
        uint32_t exec_time = micros_DWT() - start_exec;
        if (exec_time > max_exec_time) max_exec_time = exec_time;

        // ── TELEMETRY LUÔN CHẠY (Dù STOP hay không) ──
        if ((micros_DWT() - lastTelemetryTime) >= TELEMETRY_INTERVAL_US)
        {
            uint32_t m1 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
            uint32_t m2 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
            uint32_t m3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
            uint32_t m4 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);

            int len = snprintf(uart_msg, sizeof(uart_msg),
                                "%.2f %.2f %.2f %lu %lu %lu %lu %u %.1f %.1f %.0f %.0f %.0f %.0f\n",
                                myIMU.roll, myIMU.pitch, myIMU.gz,
                                m1, m2, m3, m4,
                                Base_Throttle,
                                Target_Roll, Target_Pitch, Target_Yaw_Rate,
                                Last_Out_Roll, Last_Out_Pitch, Last_Out_Yaw);

            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_msg, (uint16_t)len);
            lastTelemetryTime = micros_DWT();
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
        }
    }
}

/* =========================
   UART RX ISR
========================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    if (rx_data == '\n' || rx_data == '\r')
    {
        if (rx_index > 0)
        {
            rx_buffer[rx_index] = '\0';
            strncpy((char*)pending_cmd, rx_buffer, sizeof(pending_cmd) - 1);
            pending_cmd[sizeof(pending_cmd) - 1] = '\0';
            cmd_ready = 1;
            rx_index  = 0;
        }
    }
    else
    {
        if (rx_index < sizeof(rx_buffer) - 1)
            rx_buffer[rx_index++] = rx_data;
        else
            rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
