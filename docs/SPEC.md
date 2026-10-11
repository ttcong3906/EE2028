# EE2028 A2 — The Last of EE2028: Spec chốt

Tài liệu này là "hợp đồng" giữa bạn và mình: code viết đúng theo đây, muốn đổi hành vi thì sửa file này trước.
Những mục đánh dấu **[CẦN CHỐT]** là chỗ đề bài để ngỏ, đang để giá trị mặc định đề xuất.

---

## 1. Phần cứng

| Thiết bị | Kết nối | Ghi chú |
|---|---|---|
| Board | B-L4S5I-IOT01A (STM32L4S5VI) | |
| LED2 (xanh lá) | PB14 | `BSP_LED_*` |
| User button B1 | PC13, EXTI13, cạnh xuống | Chung IRQ `EXTI15_10` |
| HTS221 (T, H) / LPS22HB (P) / LSM6DSL (A, G) / LIS3MDL (M) | I2C2 nội bộ qua BSP | |
| LSM6DSL INT1 | PD11, EXTI11, cạnh lên | Dùng cho phát hiện lật (6D) |
| UART tới PC | USART1 PB6/PB7 (ST-LINK VCP) | 115200 8N1 |
| OLED SSD1306 128x64 | I2C1 PB8 (D15 SCL) / PB9 (D14 SDA), addr 0x3C | Ngoài, dùng lại từ năm ngoái |
| LED matrix 8x8 HT16K33 | Cùng I2C1, addr 0x70 | Ngoài, dùng lại từ năm ngoái |
| Buzzer thụ động | PB1 (D6), PWM TIM3_CH4 | Ngoài, dùng lại từ năm ngoái |

---

## 2. Trạng thái hệ thống

```
            power on
               │
               ▼
        ┌─────────────┐   double    ┌─────────────┐
        │   STANDBY   │ ──────────► │   BATTLE    │◄─────────┐
        │             │ ◄────────── │             │          │ double (rescue)
        └─────────────┘   double    └──────┬──────┘          │ hoặc lệnh RESCUE
                                           │ lật ngửa ≥ 300 ms│
                                           ▼                  │
                                    ┌─────────────┐           │
                                    │  ATTACKED   │───────────┘
                                    │ (Last of …) │
                                    └──────┬──────┘
                                           │ hết 10 s
                                           ▼
                                    ┌─────────────┐
                                    │ TERMINATED  │  (không thoát được, chỉ reset)
                                    └─────────────┘
```

---

## 3. Nút bấm

- Ngắt EXTI13 chỉ ghi thời điểm và đếm số lần nhấn. Không UART, không I2C trong ISR.
- **Debounce:** bỏ qua cạnh đến trong vòng **50 ms** sau cạnh được chấp nhận trước đó.
- **Double press:** lần nhấn thứ hai đến **< 500 ms** sau lần đầu. Xử lý ngay khi nhận lần thứ hai.
- **Single press:** sau lần nhấn đầu, đủ **500 ms** mà không có lần thứ hai. Vì vậy single luôn trễ khoảng 0.5 s (đúng theo định nghĩa của đề).
- Lần nhấn thứ ba sau một double sẽ bắt đầu một chuỗi mới.

| Trạng thái | Single | Double |
|---|---|---|
| STANDBY | Bỏ qua, in `[EVT] Single press ignored` **[CẦN CHỐT]** | Sang BATTLE |
| BATTLE | Nạp laser +3/10 | Về STANDBY |
| ATTACKED | Bỏ qua | Được cứu, về BATTLE |
| TERMINATED | Bỏ qua (UART đã tắt) | Bỏ qua |

---

## 4. Hành vi từng trạng thái

### STANDBY
- Khi vào: gửi `Entering STANDBY MODE` một lần.
- Mỗi **1 s**: đọc cả 6 cảm biến.
  - Tất cả trong ngưỡng: gửi 1 dòng `[TLM]`.
  - Có cảm biến vượt ngưỡng: **không** gửi `[TLM]`, thay bằng một dòng `[WARN]` cho mỗi cảm biến vi phạm.
- LED2: sáng liên tục.
- Không kiểm tra lật trong STANDBY.

### BATTLE (không bị tấn công)
- Khi vào: gửi `Entering BATTLE MODE` và một dòng `[STAT]`.
- **Không gửi telemetry định kỳ** **[CẦN CHỐT]**, chỉ gửi sự kiện (nạp, bắn, đổi mode).
- LED2: nháy **1 Hz** (đảo trạng thái mỗi 500 ms).
- Laser: xem mục 5.
- Kiểm tra lật: xem mục 6.

### ATTACKED ("The Last of EE2028")
- Khi vào: gửi `[EVT] Drone flipped! Entering The Last of EE2028`, bắt đầu đếm 10 s.
- Mỗi **1 s** (lần đầu gửi ngay): `[ATTACK] Drone Was Attacked! Rescue in N s`.
- LED2: nháy **2 Hz** (đảo trạng thái mỗi 250 ms).
- Double press hoặc lệnh UART `RESCUE` trong 10 s: gửi `[EVT] Rescued!`, về BATTLE. Năng lượng laser giữ nguyên.
- Sau khi được cứu, phát hiện lật chỉ **kích hoạt lại khi drone đã lật về tư thế bình thường**, để tránh vừa cứu xong lại bị tấn công ngay.
- Hết 10 s: chuyển TERMINATED.

### TERMINATED
1. Gửi `[EVT] No rescue. Drone self-destructing. Cutting all communication. Goodbye.`
2. OLED hiện màn hình cuối, LED matrix hiện dấu X, buzzer chơi giai điệu kết thúc (khoảng 2 s).
3. Tắt LED2, `HAL_UART_DeInit`, tắt mọi ngắt, cho MCU vào **Standby mode** (low-power). OLED và LED matrix vẫn giữ hình cuối vì chúng có RAM riêng.
4. Chỉ thoát được bằng nút RESET.

---

## 5. Laser gun

- Năng lượng lưu theo đơn vị **phần mười**, kiểu số nguyên `0..10`.
- Single press trong BATTLE: `+3`, rồi nếu `>= 5` thì **tự bắn**, `-5`.
- Chuỗi từ 0: 0 → 3 → 6 (bắn) → 1 → 4 → 7 (bắn) → 2 → 5 (bắn) → 0. Không bao giờ vượt 7/10, nhưng vẫn kẹp tối đa 10 cho chắc.
- Năng lượng **giữ nguyên** khi về STANDBY và khi bị tấn công.
- Đếm tổng số phát đã bắn để hiện lên OLED.
- Thông báo:
  - `[EVT] LASER charged: 3/10 -> 6/10`
  - `[EVT] LASER FIRED! Zombie down. Energy 6/10 -> 1/10`

---

## 6. Phát hiện lật ngửa

- **Chiều quy ước** **[CẦN CHỐT]**:
  - Đề viết "Z+ hướng xuống đất khi bay", tức board úp mặt xuống mới là bình thường. Khi đó bình thường `Az ≈ −9.8`, bị lật `Az ≈ +9.8`.
  - Đề xuất: bình thường là **board nằm ngửa trên bàn** (`Az ≈ +9.8`), lật úp xuống là bị tấn công. Demo tự nhiên hơn.
  - Code có một `#define FLIP_SIGN` để đổi giữa hai cách trong một dòng.
- **Cơ chế:**
  - Chức năng nhận diện hướng **6D** có sẵn trong LSM6DSL, ngưỡng góc 60°, báo ngắt qua INT1 (PD11, EXTI11) khi hướng thay đổi.
  - LSM6DSL là IMU chứa cả gyroscope lẫn accelerometer. Lúc demo có thể giải thích dòng "gyroscope detects upside-down" trong đề là chip IMU đó phát hiện.
  - Ngắt chỉ bật cờ. Main loop xác nhận bằng cách đọc `Az`, và phải **giữ tư thế lật ≥ 300 ms** mới vào ATTACKED, để lắc mạnh không bị nhận nhầm.
  - Dự phòng: trong BATTLE vẫn đọc `Az` mỗi 100 ms, phòng khi lỡ ngắt.
- **Ngưỡng:** "lật" khi thành phần trọng lực trên trục Z theo chiều ngược lại lớn hơn `0.5 g` (khoảng nghiêng hơn 120° so với bình thường).

---

## 7. Cảm biến, đơn vị và ngưỡng

| Ký hiệu | Đại lượng gửi đi | Đơn vị | Ngưỡng thấp | Ngưỡng cao | Lý do vật lý |
|---|---|---|---|---|---|
| T | Nhiệt độ | `C` | 10.0 | 35.0 | Khoảng hoạt động an toàn của pin Li-ion và điện tử; ngoài khoảng này hiệu suất pin giảm |
| P | Áp suất | `hPa` | 950.0 | 1040.0 | 950 hPa ≈ 540 m trên mực nước biển (trần bay); trên 1040 hPa gần mức kỷ lục ở mực nước biển, coi là lỗi cảm biến |
| H | Độ ẩm tương đối | `%RH` | 20.0 | 80.0 | Trên 80 % dễ đọng sương trên mạch; dưới 20 % dễ tĩnh điện |
| A | Độ lớn gia tốc √(x²+y²+z²) | `m/s^2` | 3.0 | 15.0 | < 0.3 g là rơi tự do; > 1.5 g là va chạm hoặc cơ động gắt (cảm biến đặt dải ±2 g) |
| G | Độ lớn vận tốc góc | `dps` | — | 300.0 | Quay > 300 °/s là mất kiểm soát |
| M | Độ lớn từ trường | `uT` | 10.0 | 100.0 | Từ trường Trái Đất 25–65 µT; > 100 µT là nhiễu từ mạnh, la bàn sai |

- Đổi đơn vị:
  - A: `mg × 9.80665 / 1000`.
  - G: `mdps / 1000` (BSP đã nhân độ nhạy, **không** nhân thêm 0.07).
  - M: `mGauss / 10 = µT`.
- Các ngưỡng là giá trị khởi điểm, chỉnh lại theo đo thực tế sau lần test đầu. Có thể đổi lúc chạy bằng lệnh `SET`, tiện khi TA muốn xem vi phạm.
- **Hiển thị thêm (chỉ OLED):** độ cao tương đối, tính từ áp suất lúc khởi động `P0`:
  `h = 44330 × (1 − (P/P0)^0.1903)` m.

---

## 8. UART: định dạng tin nhắn

Mỗi dòng có tiền tố `[TAG]` để người đọc trên TeraTerm vẫn dễ hiểu, còn dashboard Python tách được không cần đoán. Kết thúc bằng `\r\n`.

### Board → PC

| Tag | Khi nào | Ví dụ |
|---|---|---|
| (không tag) | Đổi mode, đúng chữ đề yêu cầu | `Entering STANDBY MODE` |
| `[TLM]` | STANDBY, mỗi 1 s, khi bình thường | `[TLM] T: 25.31 C, P: 1008.52 hPa, H: 61.20 %RH, A: 9.79 m/s^2, G: 1.25 dps, M: 45.30 uT` |
| `[WARN]` | STANDBY, mỗi 1 s, mỗi dòng một vi phạm | `[WARN] T: 36.20 C, exceeds threshold of 35.00 C` / `[WARN] H: 15.10 %RH, below threshold of 20.00 %RH` |
| `[EVT]` | Sự kiện | `[EVT] LASER FIRED! Zombie down. Energy 6/10 -> 1/10` |
| `[ATTACK]` | ATTACKED, mỗi 1 s | `[ATTACK] Drone Was Attacked! Rescue in 7 s` |
| `[STAT]` | Khi đổi mode và khi có lệnh `STATUS` | `[STAT] MODE: BATTLE, LASER: 6/10, SHOTS: 2, ALT: 3.2 m` |
| `[CMD]` | Trả lời lệnh | `[CMD] OK T_HIGH = 30.00` / `[CMD] ERR unknown command 'FOO'` |

### PC → Board (UART hai chiều)

Nhận bằng ngắt, từng byte vào ring buffer; main loop ghép dòng khi gặp `\r` hoặc `\n`. Không phân biệt hoa thường. Board không echo từng ký tự; nếu dùng TeraTerm thì bật *Local echo*.

| Lệnh | Tác dụng | Hợp lệ ở |
|---|---|---|
| `HELP` | Liệt kê lệnh | Mọi trạng thái (trừ TERMINATED) |
| `STATUS` | Gửi một dòng `[STAT]` | Mọi trạng thái |
| `THRESH` | In toàn bộ ngưỡng hiện tại | Mọi trạng thái |
| `SET <KEY> <value>` | Đổi ngưỡng; KEY ∈ `T_LOW T_HIGH P_LOW P_HIGH H_LOW H_HIGH A_LOW A_HIGH G_HIGH M_LOW M_HIGH` | Mọi trạng thái |
| `MODE STANDBY` / `MODE BATTLE` | Giống double press | STANDBY / BATTLE |
| `CHARGE` | Giống single press | BATTLE |
| `RESCUE` | Cứu drone | ATTACKED |
| `MUTE ON` / `MUTE OFF` | Tắt hoặc bật buzzer | Mọi trạng thái |

Lệnh không hợp lệ ở trạng thái hiện tại thì trả `[CMD] ERR not allowed in <MODE>`.

### Truyền đi
- TX dùng ring buffer cộng `HAL_UART_Transmit_IT`, không chặn main loop.

---

## 9. OLED 128x64 (21 ký tự × 8 dòng, font 5x7)

Vẽ vào framebuffer 1 KB trong RAM, chỉ đẩy những dòng thay đổi lên OLED. Cập nhật khi có sự kiện, cộng thêm mỗi 1 s ở STANDBY và ATTACKED.

```
STANDBY                 BATTLE                  ATTACKED                TERMINATED
+---------------------+ +---------------------+ +---------------------+ +---------------------+
|== STANDBY MODE ==   | |=== BATTLE MODE ===  | |!! UNDER ATTACK !!   | |                     |
|T 25.3C  H 61.2%     | |                     | |                     | |    DRONE  LOST      |
|P 1008.5hPa          | |LASER                | |  RESCUE WINDOW      | |                     |
|ALT   +3.2 m         | |[######----]  6/10   | |       7 s           | |  COMMS  OFFLINE     |
|A  9.79 m/s2         | |                     | |                     | |                     |
|G  1.25 dps          | |SHOTS FIRED: 2       | |DOUBLE-PRESS TO      | |  press RESET to     |
|M  45.3 uT           | |                     | |      RESCUE         | |     reboot          |
|STATUS: ALL NORMAL   | |1x CHARGE 2x STANDBY | |                     | |                     |
+---------------------+ +---------------------+ +---------------------+ +---------------------+
```

- Dòng cuối STANDBY khi có vi phạm: `WARN: T HIGH, M HIGH`.

---

## 10. LED matrix 8x8

| Lúc | Hình |
|---|---|
| STANDBY, bình thường | Vòng tròn "radar" |
| STANDBY, có vi phạm | Dấu chấm than |
| BATTLE | Thanh năng lượng: số cột sáng = năng lượng (0–7); một chấm ở cột 5 hàng trên cùng đánh dấu ngưỡng bắn |
| Khi bắn | Hiệu ứng tia laser quét ngang, 5 khung × 60 ms, rồi về thanh năng lượng |
| ATTACKED | Dấu chấm than nháy 2 Hz, cùng nhịp LED2 |
| Vừa được cứu | Mặt cười trong 1 s |
| TERMINATED | Dấu X |

---

## 11. Buzzer (PWM TIM3_CH4, không chặn)

| Sự kiện | Âm thanh |
|---|---|
| Nạp laser | Bíp ngắn 1 kHz, 50 ms |
| Bắn | "Piu": quét 2 kHz → 500 Hz trong 200 ms |
| Đổi mode | Hai tiếng bíp ngắn |
| ATTACKED | Bíp 1.5 kHz, 125 ms, cùng nhịp 2 Hz với LED |
| Được cứu | Ba nốt đi lên |
| TERMINATED | Giai điệu đi xuống (lấy từ năm ngoái) |

`MUTE ON` tắt toàn bộ âm thanh.

---

## 12. Kiến trúc phần mềm

- Siêu vòng lặp không chặn, mọi định thời dựa trên `HAL_GetTick()` qua `elapsed(since, ms)`.
- ISR chỉ ghi cờ hoặc dữ liệu vào biến `volatile` hay ring buffer:
  - EXTI13: nút bấm.
  - EXTI11: LSM6DSL INT1.
  - USART1: RX từng byte, TX xong một đoạn.
- Không gọi I2C, `HAL_Delay` hay UART blocking trong ISR.
- `sprintf("%f")` cần bật `-u _printf_float` trong linker.
- Cách chia file **[CẦN CHỐT]**: tùy yêu cầu nộp bài.

## 13. Bảng thời gian

| Hạng mục | Giá trị |
|---|---|
| Đọc cảm biến và gửi TLM/WARN | 1000 ms |
| Debounce nút | 50 ms |
| Cửa sổ double press | < 500 ms |
| LED BATTLE / ATTACKED | đảo mỗi 500 ms / 250 ms |
| Kiểm tra lật (dự phòng) | 100 ms |
| Giữ lật để xác nhận | 300 ms |
| Tin `[ATTACK]` | 1000 ms |
| Cửa sổ cứu hộ | 10000 ms |

## 14. Tình huống biên

- Vào BATTLE khi board đang lật: sau 300 ms vào ATTACKED (đúng spec).
- Double press ở STANDBY lúc đang có WARN: vẫn sang BATTLE.
- Nhấn nút trong lúc nhạc hoặc hiệu ứng đang chạy: không ảnh hưởng, vì mọi thứ đều không chặn.
- I2C ngoài (OLED, LED matrix) không cắm: hệ thống vẫn chạy; lỗi I2C bị bỏ qua sau timeout ngắn.
