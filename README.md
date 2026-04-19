# Hệ thống đo lường đa cảm biến dùng STM32F103C8T6 và GUI C#

## 1. Giới thiệu

Đây là đồ án xây dựng **hệ thống đo lường đa cảm biến** với vi điều khiển **STM32F103C8T6**, firmware viết bằng **Keil MDK + STM32CubeMX/HAL**, và phần mềm giám sát trên máy tính viết bằng **C# WinForms**.

Hệ thống thu thập dữ liệu từ 4 cảm biến:

- **Thermistor**: đo nhiệt độ
- **Loadcell**: đo lực/khối lượng
- **Potentiometer**: đo góc hoặc vị trí
- **Incremental Encoder**: đo vị trí/góc quay

Dữ liệu được xử lý trên MCU, sau đó truyền theo thời gian thực lên GUI qua **UART (COM Port)**.  
Mục tiêu của dự án là thiết kế một hệ đo có khả năng:

- đọc nhiều loại cảm biến trên cùng một nền tảng MCU
- chuẩn hóa dữ liệu về cùng một luồng xử lý
- hỗ trợ hiệu chuẩn cơ bản cho từng cảm biến
- hiển thị dữ liệu realtime trên GUI
- dễ mở rộng, dễ debug, phù hợp với bài tập lớn và demo thực nghiệm

---

## 2. Hướng triển khai đã chọn

### MCU firmware
- Vi điều khiển: **STM32F103C8T6**
- IDE/Compiler: **Keil MDK**
- Cấu hình ngoại vi: **STM32CubeMX + HAL**
- Kiến trúc phần mềm: **super loop + scheduler mềm**
- Dùng **interrupt + DMA** ở các vị trí cần thiết
- **Không dùng FreeRTOS**

### Phần đọc cảm biến
- **Thermistor**: ADC + DMA
- **Potentiometer**: ADC + DMA
- **Loadcell**: HX711
- **Encoder**: Timer Encoder Mode

### Giao tiếp với GUI
- **UART qua COM port**
- Giai đoạn đầu dùng **ASCII packet** để đơn giản hóa việc debug và kiểm thử

---

## 3. Kiến trúc hệ thống

```text
[Cảm biến]
   |-- Thermistor ------> ADC + DMA
   |-- Potentiometer ---> ADC + DMA
   |-- Loadcell --------> HX711
   |-- Encoder ---------> Timer Encoder Mode
                |
                v
        [STM32F103C8T6]
        - lọc dữ liệu
        - chuyển đổi đơn vị
        - hiệu chuẩn
        - phát hiện lỗi cơ bản
        - đóng gói khung truyền UART
                |
                v
        [UART / COM Port]
                |
                v
         [GUI C# WinForms]
         - hiển thị realtime
         - vẽ biểu đồ
         - hiệu chuẩn
         - gửi lệnh điều khiển