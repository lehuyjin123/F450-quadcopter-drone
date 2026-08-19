# F450-quadcopter-drone
This repo is about my final for the embedded system in my uni. 
Even tho the whole project was done in just a month, from designing PCB to programming and controlling the drone.
But I did manage to made it fly, well...hover at the designated height and self-balancing to be exact.
The drone controlled using your phone which connected the to esp32 wifi on the drone, so no download needed on the phone. Just put the correct IP address on your browser.

Flight Controller:
STM32F103C8T6 (Blue Pill) — main controller

Communication:
ESP32 — WiFi bridge (AP mode, WebSocket)

Sensor:
MPU6050 (module GY-521) — IMU (gyro + accelerometer)

Frame:
Frame F450
Landing legs with 2 carbon

Motors & ESC:
4x ESC XXD 30A (Yellow)
4x Brushless Motor 950kv
4X 9045 propellers

Programming:
UART: PA9(TX)→ESP32 RX0, PA10(RX)→ESP32 TX0


Battery: 3s 3300mah
Was able to hover continuously around 30mins

<img width="1024" height="1536" alt="ChatGPT Image Jun 15, 2026, 11_46_46 PM" src="https://github.com/user-attachments/assets/67dd6096-971a-4d09-99b6-f7cd66f774bd" />
