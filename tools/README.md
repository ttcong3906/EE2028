# tools/

## sim/: kiểm tra logic firmware trên máy tính

Chạy `Core/Src/samplecode.c` trên PC với HAL giả (đồng hồ, cảm biến, nút, USART1, UART4 nối tay cầm, 5-way, LED matrix, OLED, flash), không cần board.

```
bash tools/sim/run_tests.sh        # trong WSL, cần gcc
```

- `sim.c` gồm hơn 130 bài kiểm tra theo spec v3.3. Sửa firmware xong thì chạy lại để chắc không làm hỏng chỗ khác.
- `simrt.c` chạy firmware theo thời gian thực trên một cổng serial ảo (pty) để gõ lệnh như TeraTerm. Cách nối được ghi ở đầu file.
