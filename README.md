# xe điều khiển sử dụng stm32, nrf24l01, joystick

stm32f411 (tranmitter) : thực hiện lấy mẫu adc đọc joystick, truyền giá trị adc qua nrf24l01.
stm32f103 (receiver) : thu giá trị adc, xử lí giá trị adc để truyền vào thanh ghi CCR PWM để điều khiển tốc độ động cơ.
