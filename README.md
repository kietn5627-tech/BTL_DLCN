# Hệ thống đo lường đa cảm biến dùng STM32F103C8T6 và GUI C#

## 1. Giới thiệu

Đây là đồ án xây dựng **hệ thống đo lường đa cảm biến** với vi điều khiển **STM32F103C8T6**, firmware viết bằng **Keil MDK + STM32CubeMX/HAL**, và phần mềm giám sát trên máy tính viết bằng **C# WinForms**.

Hệ thống thu thập dữ liệu từ 4 cảm biến:

- **Thermistor**:
- **Laser**: 
- **Potentiometer**:
- **Ultra sonic**:

Dữ liệu được xử lý trên MCU, sau đó truyền theo thời gian thực lên GUI qua **UART (COM Port)**.  
Mục tiêu của dự án là thiết kế một hệ đo có khả năng:

- đọc nhiều loại cảm biến trên cùng một nền tảng MCU
- chuẩn hóa dữ liệu về cùng một luồng xử lý
- hỗ trợ hiệu chuẩn cơ bản cho từng cảm biến
- hiển thị dữ liệu realtime trên GUI
- dễ mở rộng, dễ debug, phù hợp với bài tập lớn và demo thực nghiệm

---

## 2. Hướng triển khai

### MCU firmware
- Vi điều khiển: **STM32F103C8T6**
- IDE/Compiler: **Keil MDK**
- Cấu hình ngoại vi: **STM32CubeMX + HAL**
- Kiến trúc phần mềm: **loop + scheduler**
- Dùng **interrupt + DMA** ở các vị trí cần thiết
- **Không dùng FreeRTOS**

### Phần đọc cảm biến
- **Thermistor**: ADC1_IN0 - PA0
- **Potentiometer**: ADC2_IN1 - PA1
- **Laser**: PB8 → I2C1_SCL, PB8 → I2C1_SCL (module VL53L0X)
- **Encoder**: TRIG → PB6 (output), ECHO → PB7 (Input capture - TIM4_CH2) (HC_SR04 module)

### Giao tiếp với GUI
- **UART qua COM port**
---
