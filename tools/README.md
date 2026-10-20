# tools/

## dashboard.py: Command Centre (thay OLED)

Chạy bằng **Python trên Windows** (không chạy trong WSL), vì cổng COM của board nằm phía Windows.

```
pip install pyserial
python tools\dashboard.py                    # chọn cổng COM ở thanh trên rồi bấm Connect
python tools\dashboard.py --drone COM5 --ctl COM7
python tools\dashboard.py --demo             # drone giả, không cần board
```

- Mỗi cổng COM chỉ một chương trình mở được: **đóng TeraTerm trước**. Tab *Console* của dashboard hiện mọi dòng thô và gõ lệnh được như TeraTerm.
- Khi kết nối, dashboard gửi `DASH ON` để drone gửi thêm `[HUD]`/`[MAP]`. Khi đóng, dashboard gửi `DASH OFF`.
- Tay cầm (board 2, đã gõ `ROLE CONTROLLER` một lần) là tùy chọn. Dashboard chuyển `[CTL] TILT` thành `AIM`, `BTN SINGLE` thành `LOCK`, còn `BTN DOUBLE` thành `POWER NEXT` (ở BATTLE) hoặc `RESCUE` (ở ATTACKED). Dashboard cũng gửi `LINK <mode>` mỗi 250 ms để LED2 trên tay cầm nháy theo drone.
- Phím tắt khi ô lệnh không được chọn: mũi tên = AIM, `L` = LOCK, `P` = POWER NEXT, Space = CHARGE, `R` = RESCUE.
- Muốn biết cổng nào là board nào: Device Manager → Ports → "STMicroelectronics STLink Virtual COM Port".

## sim/: kiểm tra logic firmware trên máy tính

Chạy `Core/Src/samplecode.c` trên PC với HAL giả (đồng hồ, cảm biến, nút, UART, 5-way, IR, flash), không cần board.

```
bash tools/sim/run_tests.sh        # trong WSL, cần gcc
```

- `sim.c` gồm hơn 100 bài kiểm tra theo spec v3.2. Sửa firmware xong thì chạy lại để chắc không làm hỏng chỗ khác.
- `simrt.c` chạy firmware theo thời gian thực trên một cổng serial ảo (pty), dùng để thử dashboard với firmware thật trên Linux. Cách nối được ghi ở đầu file.
