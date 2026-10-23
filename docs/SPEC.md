# EE2028 A2 — The Last of EE2028: Spec v4 (đã chốt 23/10)

Tài liệu này là "hợp đồng" của nhóm: code viết đúng theo đây, muốn đổi hành vi thì sửa file này trước.
Firmware nằm trong **một file `Core/Src/main.c`**, chạy trên **một board**. Không dùng màn hình laptop để hiển thị: mọi thứ hiện trên OLED và LED matrix, laptop chỉ chạy TeraTerm *(đề)*. Lịch sử quyết định ở mục 18.

## Tổng quan

| Thành phần | Vai trò |
|---|---|
| Board B-L4S5I-IOT01A | Toàn bộ yêu cầu của đề |
| **LED matrix** | **Bản đồ**: sương mù trinh sát ở STANDBY, bản đồ bắn zombie ở BATTLE, chữ SOS khi bị tấn công |
| **OLED** | **Chữ và số**: cảm biến, kết quả sonar, năng lượng laser, điểm, đếm ngược cứu hộ, hộp đen |
| **Buzzer** | **Âm thanh và nhạc**: tiếng vọng sonar, tiếng bắn, nhịp tim nhanh dần theo mức nguy hiểm, nhạc chiến thắng / kỷ lục / kết thúc, SOS |
| **5-way switch** | **Điều khiển**: lái drone trinh sát, dời tâm ngắm, chuyển trang OLED, đổi công suất |

**Câu chuyện chung:** drone đi qua đúng vòng đời mà đề cho:
**trinh sát** ở STANDBY (6 cảm biến, lái drone qua vùng sương mù, dùng sonar dò ổ zombie, tìm nơi ở cho người) → **chiến đấu** ở BATTLE (ổ đã tìm thấy hiện ra trên bản đồ, ngắm bằng 5-way, bắn theo đợt, ghi điểm) → **bị hạ** ở The Last of EE2028 (SOS, chờ cứu) → **được cứu** hoặc **tự hủy** (cắt liên lạc, hộp đen được giữ lại để điều tra).

Ký hiệu *(đề)* = yêu cầu bắt buộc của đề bài. **Drone không có module ngoài vẫn phải chạy đúng toàn bộ phần *(đề)*.**

---

## 1. Phần cứng

| Thiết bị | Kết nối | Vai trò |
|---|---|---|
| Board | B-L4S5I-IOT01A (STM32L4S5VI), clock 4 MHz MSI mặc định | |
| LED2 (xanh lá) | PB14 | Báo mode *(đề)* |
| User button B1 | PC13, EXTI13, cả hai cạnh | Nút chính *(đề)* |
| HTS221 (T, H), LPS22HB (P), LSM6DSL (A, G), LIS3MDL (M) | I2C2 nội bộ qua BSP | 6 cảm biến *(đề)*, phát hiện lật |
| LSM6DSL INT1 | PD11, EXTI11 | Ngắt 6D khi đổi hướng |
| UART tới PC | USART1 PB6/PB7 (ST-LINK VCP), 115200 8N1 | TeraTerm *(đề)* |
| Red LED Matrix 8x8 (HT16K33) | Cổng Grove I2C, addr 0x70 | Bản đồ |
| OLED 128x64 (SSD1306/SSD1315) | Cổng Grove I2C, addr 0x3C | Màn hình chữ |
| 5-way switch | Cổng Grove I2C, addr 0x03 | Điều khiển |
| Buzzer | Cổng Grove **D6** (PB1, TIM3_CH4 PWM) | Âm thanh |

**Bus I2C1 (PB8/PB9) dùng chung, đổi tốc độ theo từng thiết bị:** OLED và matrix chạy **100 kHz**, 5-way chạy ở tốc độ mà lần dò tìm ra (**25 kHz** với module của nhóm; ở 50/100 kHz module này không trả lời được). Đổi tốc độ chỉ là ghi lại thanh ghi timing.

---

## 2. Trạng thái hệ thống *(đề)*

```
  power on ─► BOOT ─► STANDBY ◄──double──► BATTLE ◄────────────┐
                     (trinh sát)          (chiến đấu)          │ double / RESCUE
                                              │ lật ≥ 300 ms   │
                                              ▼                │
                                          ATTACKED ────────────┘
                                     (The Last of EE2028)
                                              │ hết 10 s
                                              ▼
                                         TERMINATED  (chỉ RESET mới thoát)
```

---

## 3. Nút bấm B1 *(đề: phải đọc bằng ngắt)*

- Ngắt EXTI13 bắt cả cạnh nhấn và cạnh nhả, chỉ ghi thời điểm. Không UART, không I2C trong ISR.
- **Debounce:** một lần nhấn chỉ được tính khi nút đã nhả ổn định ≥ 30 ms và cách lần nhấn trước ≥ 50 ms. Giữ nút bao lâu cũng chỉ là một lần.
- **Double press:** lần thứ hai < 500 ms sau lần đầu, xử lý ngay. **Single press:** đủ 500 ms không có lần thứ hai.

| Trạng thái | Single | Double |
|---|---|---|
| STANDBY | **Sonar ping** + báo cáo trinh sát (đề: "or create your own operation"), mục 7 | Sang BATTLE *(đề)* |
| BATTLE | Nạp laser +3/10 *(đề)* | Về STANDBY *(đề)* |
| ATTACKED | Bỏ qua *(đề)* | Được cứu, về BATTLE *(đề)* |
| TERMINATED | Bỏ qua | Bỏ qua |

---

## 4. Cấu hình

| Mục | Mặc định | Đổi bằng |
|---|---|---|
| **Im lặng** | Tắt | `MUTE ON\|OFF` (SOS vẫn kêu) |
| **Kỷ lục điểm** | 0 | Tự lưu khi phá kỷ lục; `HISCORE RESET` để xóa |

- Cố định trong code: drone **#3**, mặt chip hướng lên là bình thường.
- Lưu vào **trang flash áp chót** (có mã kiểm tra), reset hay mất điện không mất. Mỗi lệnh tự lưu, trả `[CMD] OK … (saved)`.
- Chỉ đổi được ở **STANDBY**.

---

## 5. Khởi động

| Bước | Làm gì | Matrix / OLED / buzzer | UART |
|---|---|---|---|
| 1 | Đọc cấu hình | | `=== Drone #3 online === silent: OFF` |
| 2 | Chờ 300 ms cho các module Grove khởi động, gỡ kẹt bus I2C nếu cần, dò module (5-way ở 25 kHz, kiểm tra mã ID) | | |
| 3 | Báo kết quả | OLED: trang **SYSTEM CHECK** 2.5 s (cảm biến, matrix, 5-way, hộp đen, kỷ lục) | `[EVT] System check: sensors OK, LED matrix OK, OLED OK, 5-way OK` |
| 4 | Lưu `P0` (trung bình 10 lần đọc) làm mốc độ cao 0 m | | `[EVT] Altitude zero at 1008.52 hPa` |
| 5 | Nếu flash còn hộp đen của lần bay trước | | `[EVT] Black box of Drone #3 recovered (18 records). Type BLACKBOX to read` |
| 6 | Tạo khu vực trinh sát đầu tiên, vào STANDBY | Matrix: số hiệu 1 s; tiếng khởi động | `Entering STANDBY MODE` *(đề)* |

---

## 6. STANDBY: trinh sát

| Thành phần | Làm gì | Matrix / OLED / buzzer | UART |
|---|---|---|---|
| 6 cảm biến *(đề)* | Đọc mỗi 1 s, so ngưỡng (mục 13) | OLED trang 1 | `[TLM] T: … M: …` **đúng 6 trường**, hoặc `[WARN] …` *(đề)* |
| LED2 *(đề)* | Sáng liên tục | | |
| **Bản đồ trinh sát** | Lái drone qua sương mù, quét 3x3 (mục 7) | Matrix | `[EVT] Scouting: nest found at (5,2) (2/3)` |
| **Sonar** (PB single) | Ping, nghe tiếng vọng (mục 7) | Vòng sóng trên matrix; ping + tiếng vọng; OLED `ECHO: 3 CELLS NE` | `[EVT] Sonar ping 1/3: echo from 3 cells NE` + `[REPORT] …` |
| **Tìm nơi ở** | Nhiệt độ 20–32 °C **và** độ ẩm 40–70 %RH liên tục 3 s (đề: humidity "used to find new human habitats") | Matrix: ngôi nhà 2 s; chuông | `[EVT] Possible human habitat found: T 26.1 C, H 58 %RH` / `[EVT] Habitat conditions lost` |
| 5-way 4 hướng | Lái drone trinh sát 1 ô (giữ thì đi liên tục) | | |
| 5-way giữa **nhấn ngắn** | Sang trang OLED kế tiếp (1 → 2 → 3 → 4 → 1) | | |
| 5-way giữa **giữ 1 s** | Đặt lại mốc độ cao 0 m tại chỗ  | Hai tiếng bíp | `[EVT] Altitude zero reset at 1008.40 hPa` |

- Lật board ở STANDBY không có tác dụng. Ở STANDBY không có nhạc nền.
- Khoảng tìm nơi ở nới rộng hơn chuẩn tiện nghi ASHRAE (khoảng 30–60 %RH) để demo được trong phòng lab có máy lạnh ở Singapore.

**Các trang OLED ở STANDBY** (21 ký tự × 8 dòng):

| Trang | Nội dung |
|---|---|
| 1 Sensors | T, H, P, A, G, M kèm đơn vị, độ cao. Dòng cuối: cảnh báo đang có (`WARN: HHI MHI`) hoặc `STATUS: ALL NORMAL` |
| 2 Scouting | Số ổ đã tìm x/3, số ping còn lại x/3, số ô đã quét x/64, vị trí drone, tiếng vọng gần nhất, nơi ở |
| 3 Mission | Điểm hiện tại, kỷ lục, thời gian bay, zombie đã hạ, số phát bắn, số lần tìm được nơi ở |
| 4 Black box | Tổng kết chuyến gần nhất (hoặc của drone đã tự hủy), lật từng sự kiện bằng 5-way **trái/phải** (trên trang này trái/phải không lái drone) |

Dòng cuối mỗi trang hiện sự kiện vừa xảy ra trong 2 s (`NEST FOUND 2/3`, `ECHO: 3 CELLS NE`, …).

---

## 7. Trinh sát: bản đồ và sonar

**Bản đồ:**
- Matrix 8x8 là vùng phía trước drone, **cùng tọa độ với bản đồ BATTLE** (hàng 0 xa nhất, hàng 7 gần nhất).
- Mỗi lần vào STANDBY là một **khu vực mới**: toàn bộ phủ sương mù, giấu ngẫu nhiên **3 ổ zombie** ở hàng 0–5, có **3 lần ping**. Ngẫu nhiên được gieo từ nhiễu cảm biến nên mỗi lần bật nguồn khác nhau.
- Drone trinh sát xuất phát ở (3,7). Di chuyển 1 ô mỗi lần bằng 5-way.
- Mỗi khi tới một ô, **vùng 3x3 quanh drone** được quét. Thấy ổ thì báo và bíp; tìm đủ 3 ổ thì `[EVT] Scouting complete: all 3 nests located` kèm giai điệu.

| Trên matrix | Nghĩa |
|---|---|
| Sáng liên tục | Sương mù (chưa quét) |
| Tắt | Đã quét, trống |
| Nháy nhanh 4 Hz | Ổ zombie đã tìm thấy |
| Nháy chậm 1 Hz | Drone trinh sát |

**Sonar** (PB single hoặc lệnh `PING`):
1. Matrix vẽ **vòng sóng** lan ra từ drone (4 khung × 90 ms), buzzer kêu "ping".
2. Sonar dò **ổ chưa tìm thấy gần nhất**. Khoảng cách `d` tính theo số ô đi chéo được (`max(|dx|, |dy|)`, 1–7), hướng làm tròn về 8 hướng (N là phía hàng 0).
3. **Tiếng vọng** đến sau `150 + 100·d` ms, cao độ `2600 − 250·d` Hz: ổ càng gần thì vọng càng sớm và càng cao.
4. Cùng lúc tiếng vọng, matrix lóe **một vạch từ drone theo hướng đó, dài `d` ô** trong 0.7 s; OLED và UART báo `ECHO: 3 CELLS NE`.
5. Sonar **không mở ô nào**: muốn xác nhận phải lái drone tới.
6. Hết ping: `[EVT] Sonar: no pings left in this area`, tiếng ù trầm. Đã tìm đủ 3 ổ: `no echo`.
7. Mỗi lần ping cũng gửi `[REPORT] Drone #3: flight 02:13, alt +2.1 m, area scanned 37/64, nests found 2/3, pings left 1/3, score 60, habitat found 2x, zombies down 12, shots 6`.

**Liên kết với BATTLE:**
- Vào BATTLE, mỗi ping còn dư thành **+1/10 năng lượng laser** (`[EVT] Unused sonar pings: +2/10 laser energy`). Đủ năng lượng thì bắn ngay theo đúng luật tự bắn của đề.
- Điểm trinh sát xem mục 11.

---

## 8. BATTLE: chiến đấu

| Thành phần | Làm gì | Matrix / OLED / buzzer | UART |
|---|---|---|---|
| Khi vào | Đổi ping dư thành năng lượng, bắt đầu **đợt 1** từ khu vực vừa trinh sát (mục 9) | Đoạn nhạc vào trận | `Entering BATTLE MODE` *(đề)* + `[STAT]` + `[EVT] Wave 1 incoming: 3 outbreaks (2 scouted nests, bigger)` |
| LED2 *(đề)* | Nháy 1 Hz | | |
| PB single *(đề)* | Nạp +3/10, đủ năng lượng thì tự bắn (mục 10) | Bíp nạp; tia + vùng nổ, piuuu / pew / BOOM | `[EVT] LASER charged: 3/10 -> 6/10` |
| 5-way 4 hướng | Dời tâm ngắm 1 ô, giữ thì chạy liên tục | Tâm ngắm nháy 4 Hz | |
| 5-way giữa ngắn | Nhảy tới ô nhiễm gần nhất | | `[EVT] Target locked at (3,2)` |
| 5-way giữa **giữ 1 s** | Đổi công suất: RULE 5/10 → 1 → 2 → 3 → 4 → 5 → RULE | | `[EVT] Weapon: power 3/10, cooldown 3 s` |
| **Nhạc nhịp tim** | Mục 12 | Buzzer | |
| Accel *(đề)* | Phát hiện lật theo kiểu lắp (mục 14) | | → ATTACKED |
| PB double *(đề)* | Chốt điểm, về STANDBY, **bản đồ bị xóa**, khu vực trinh sát mới | | `Entering STANDBY MODE` |

**OLED ở BATTLE:** dòng 0 đợt và số mục tiêu (`WAVE 2   TARGETS 11` / `CLEARED!`), dòng 1–2 thanh năng lượng 10 ô cỡ lớn, dòng 3 năng lượng + công suất hoặc cooldown còn lại, dòng 4 tâm ngắm (`TARGET!` nếu đang nhắm trúng ô nhiễm), dòng 5 **điểm và hệ số nhân** (`SCORE 1230 x3`), dòng 6 sự kiện (`HIT! 5 CELLS +150`, `MISSED!`, `AREA CLEARED!`, `AREA OVERRUN!`), dòng 7 hướng dẫn nút.

BATTLE không gửi telemetry định kỳ, chỉ gửi sự kiện.

---

## 9. Bản đồ lây nhiễm và các đợt

- Ô sáng = khu bị zombie chiếm. Drone ở giữa cạnh dưới.
- **Đợt 1** dùng đúng 3 ổ của khu vực trinh sát: ổ **đã tìm thấy** hiện thành **cụm chữ thập 5 ô** (nhiều zombie hơn), ổ chưa tìm thấy chỉ là 1 ô. Tâm ngắm khóa sẵn vào ổ đã tìm thấy gần nhất (`[EVT] Target locked on scouted nest at (3,4)`).
- **Đợt n ≥ 2** rải **2 + n** ổ ngẫu nhiên (tối đa 12) ở hàng 0–4.
- Trong đợt, mỗi khoảng thời gian một ô nhiễm lây sang một ô kề chưa nhiễm. Khoảng này bắt đầu 3 s ở đợt 1, mỗi đợt nhanh hơn 0.25 s, nhanh nhất 1 s.
- **Dọn sạch bản đồ** → `[EVT] Wave 2 cleared! AREA CLEARED`, nhạc chiến thắng, matrix hiện dấu tích. 2 s sau đợt kế tiếp bắt đầu.
- Thua một đợt không dẫn tới ATTACKED. Theo đề, drone chỉ bị hạ khi bị lật.
- **Quá 75 % (≥ 48/64 ô):** chỉ báo `[EVT] AREA OVERRUN (52/64)`.
- Bị tấn công thì bản đồ dừng lây. Được cứu thì tiếp tục đúng đợt đó.
- Không có 5-way: tâm ngắm tự nhảy tới ô nhiễm gần nhất trước mỗi phát bắn.

---

## 10. Laser gun

- Năng lượng theo đơn vị **1/10**, số nguyên 0–10. Mỗi single press **+3/10** *(đề)*.
- **Chế độ bắn:**
  - **RULE** (mặc định, đúng đề): mỗi phát `p = 5`, đủ năng lượng là tự bắn ngay, **không cooldown** *(đề)*.
  - **SEL** (5-way giữa giữ 1 s, đi vòng RULE → 1 → … → 5 → RULE): `p` = 1–5. Sau mỗi phát có **cooldown `p` giây**, riêng `p = 1` không có. Trong lúc cooldown vẫn nạp được. Hết cooldown mà đủ năng lượng thì tự bắn ngay.
- Tự bắn khi năng lượng ≥ `p`, mỗi lần **một phát**, trừ `p`. Năng lượng thừa để dành, tối đa 10/10.
- Trong 2 s nghỉ sau AREA CLEARED bản đồ trống nên không bắn (nạp vẫn được). Đợt mới vừa xuất hiện mà đủ năng lượng thì bắn ngay.
- Mỗi lần vào BATTLE (chuyến xuất kích mới) chế độ bắn về **RULE**. Được cứu thì giữ nguyên chế độ đang dùng.

| `p` | Vùng nổ | Âm thanh |
|---|---|---|
| 1 | 1 ô | "piuuu": cao, ngắn |
| 2–3 | Chữ thập 5 ô | "pew" |
| 4–5 | Vuông 3x3 | "BOOM": ù trầm 400 → 80 Hz, có rung |

- Thông báo: `[EVT] LASER FIRED (5/10, BOOM) at (3,2): 7 cells cleared, +140 points (x2). Energy 6/10 -> 1/10. Zombies down: 19`; trượt: `… missed, no zombies there …`; hết cooldown: `[EVT] Laser ready`.
- Matrix: tia quét từ drone tới tâm ngắm (4 khung × 50 ms), rồi vùng nổ nháy 150 ms.

---

## 11. Điểm và kỷ lục

| Hành động | Điểm |
|---|---|
| Tìm thấy một ổ khi trinh sát | +20 |
| Tìm đủ 3 ổ | +30, cộng thêm **+20 cho mỗi ping còn dư** |
| Mỗi ô zombie bị dọn | +10 × **hệ số nhân** |
| Dọn sạch đợt `n` | +50 × `n` |

- **Hệ số nhân (combo):** phát bắn trúng liên tiếp tăng hệ số: phát trúng đầu ×1, phát thứ hai liên tiếp ×2, thứ ba ×3, từ thứ tư trở đi ×4. Bắn trượt thì về ×1.
- **Một ván** = một khu vực trinh sát + chuyến BATTLE ngay sau nó. Điểm về 0 khi bắt đầu khu vực mới.
- **Chốt điểm** khi về STANDBY hoặc khi tự hủy: `[EVT] Mission score: 1230 (best 870)`. Phá kỷ lục thì `[EVT] NEW HIGH SCORE: 1230 (was 870)`, lưu vào flash, OLED `NEW HIGH SCORE!` và đoạn nhạc kỷ lục (khi tự hủy thì chỉ lưu và báo, không phát nhạc).
- Lệnh `HISCORE RESET` (STANDBY) xóa kỷ lục.

---

## 12. Âm thanh và nhạc

| Lúc | Buzzer |
|---|---|
| Khởi động | Tiếng khởi động ngắn |
| STANDBY: tìm thấy ổ / tìm đủ 3 ổ / nơi ở | Bíp báo động / giai điệu / hai nốt chuông |
| STANDBY: sonar | "ping" rồi **tiếng vọng** (cao độ và độ trễ theo khoảng cách, mục 7) |
| Vào BATTLE | Đoạn nhạc vào trận (4 nốt) |
| **BATTLE: nhịp tim** | Hai tiếng "thình thịch" trầm lặp lại khi buzzer rảnh. Chu kỳ theo số ô nhiễm: **1200 ms** khi bản đồ gần trống, nhanh dần tới **350 ms** khi ≥ 48 ô. Tắt trong lúc nghỉ giữa hai đợt |
| BATTLE: nạp / bắn | Bíp ngắn / piuuu, pew, BOOM |
| Dọn sạch đợt | Nhạc chiến thắng |
| Phá kỷ lục | Đoạn nhạc kỷ lục |
| ATTACKED | **Morse SOS liên tục, kêu cả khi `MUTE ON`** |
| Được cứu | Ba nốt đi lên |
| TERMINATED | Giai điệu kết thúc |

- Hiệu ứng (bắn, nạp, …) luôn ngắt nhịp tim; nhịp tim chỉ chạy khi buzzer đang rảnh. Không gì ngắt được SOS.
- `MUTE ON` tắt mọi âm thanh trừ SOS.
- Bộ phát âm thanh không chặn: một chuỗi nốt (tần số đầu, tần số cuối, thời gian, có rung hay không) chạy dần trong vòng lặp chính.

---

## 13. Cảm biến, đơn vị và ngưỡng *(đề)*

| Ký hiệu | Đại lượng | Đơn vị | Thấp | Cao | Lý do vật lý |
|---|---|---|---|---|---|
| T | Nhiệt độ | `C` | 10.0 | 35.0 | Khoảng hoạt động an toàn của pin Li-ion và điện tử |
| P | Áp suất | `hPa` | 950.0 | 1040.0 | 950 hPa ≈ 540 m (trần bay); trên 1040 hPa gần kỷ lục ở mực nước biển, coi là lỗi |
| H | Độ ẩm tương đối | `%RH` | 20.0 | 80.0 | Trên 80 % dễ đọng sương trên mạch; dưới 20 % dễ tĩnh điện |
| A | Độ lớn gia tốc √(x²+y²+z²) | `m/s^2` | 3.0 | 15.0 | < 0.3 g là rơi tự do; > 1.5 g là va chạm (cảm biến đặt dải ±2 g) |
| G | Độ lớn vận tốc góc | `dps` | — | 300.0 | Quay > 300 °/s là mất kiểm soát |
| M | Độ lớn từ trường | `uT` | 10.0 | 100.0 | Từ trường Trái Đất 25–65 µT; > 100 µT là có nam châm hoặc vật nhiễm từ ở gần |

- Đổi đơn vị: A `mg × 9.80665 / 1000`; G `mdps / 1000`; M `mGauss / 10 = µT`.
- **Độ cao** (OLED, báo cáo): mốc 0 m là `P0` lúc khởi động, `h = 44330 × (1 − (P/P0)^0.1903)` m. Lọc trung bình trượt rồi làm tròn theo bậc 0.1 hPa (≈ 0.85 m) tính từ P0, có trễ 0.06 hPa để không nhảy. Dòng `[TLM]` vẫn gửi áp suất thật.

---

## 14. Phát hiện lật *(đề)*

- Board nằm ngửa, mặt chip hướng lên: `Az ≈ +9.8 m/s²`. Lật úp: `Az ≈ −9.8`.
- Chức năng **6D** trong LSM6DSL (ngưỡng 60°) báo ngắt INT1 khi hướng thay đổi. Main loop xác nhận bằng `Az` và phải **giữ lật ≥ 300 ms**. Dự phòng: đọc `Az` mỗi 100 ms.
- "Lật" khi trọng lực trên trục Z theo chiều ngược lại lớn hơn 0.5 g.
- LSM6DSL là IMU chứa cả gyroscope và accelerometer. Đó là cách hiểu câu "gyroscope detects upside-down" của đề: gyro chỉ đo tốc độ quay nên không tự biết được úp hay ngửa.
- Sau khi được cứu, drone phải được lật ngửa lại một lần mới có thể bị tấn công tiếp.

---

## 15. ATTACKED và TERMINATED *(đề)*

**ATTACKED:**
- LED2 nháy 2 Hz *(đề)*. Khi vào: `[EVT] Drone #3 flipped! The Last of EE2028`. Mỗi 1 s: `[ATTACK] Drone #3 Was Attacked! Rescue in 7 s` *(đề)*.
- Buzzer phát **Morse SOS** liên tục, kêu cả khi im lặng. Matrix hiện chữ S-O-S theo nhịp.
- OLED: `!! UNDER ATTACK !!`, **số giây còn lại cỡ chữ lớn**, `DOUBLE-PRESS TO RESCUE`.
- **Được cứu** khi double press *(đề)* hoặc lệnh `RESCUE`. Báo `[EVT] Drone #3 rescued!`, về BATTLE. Matrix mặt cười 1 s, ba nốt đi lên.
- Single press và 5-way bị bỏ qua.

**TERMINATED** (hết 10 s không ai cứu):
1. Chốt điểm (mục 11), ghi hộp đen vào flash (mục 16).
2. `[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.`
3. Matrix dấu X; OLED `DRONE #3 LOST / COMMS OFFLINE / BLACK BOX SAVED` (OLED tự giữ hình này sau khi MCU tắt); giai điệu kết thúc.
4. Tắt USART1, LED2, các ngắt, MCU vào Standby mode.
5. Chỉ nút **RESET** đen (hoặc rút nguồn) mới thoát được.

---

## 16. Hộp đen (black box)

- Ghi **từng chuyến xuất kích**: bắt đầu khi vào BATTLE (xóa bản ghi cũ trong RAM và flash), tới khi về STANDBY hoặc tự hủy. Được cứu vẫn là cùng chuyến.
- **32 sự kiện gần nhất** (bộ đệm vòng): đợt bắt đầu/dọn xong, nạp, bắn, đổi công suất, quá 75 %, bị lật, được cứu, về STANDBY, tự hủy.
- **Bản tổng kết:** số đợt đã dọn, số phát bắn, số ô đã dọn, thời gian bay, lý do kết thúc (`RETURNED` / `RESCUE TIMEOUT`).
- Khi tự hủy: ghi **một lần** vào **trang flash cuối cùng** (có mã kiểm tra), sống sót qua RESET và mất điện.
- Xem ở STANDBY: lệnh `BLACKBOX` (in tổng kết rồi từng sự kiện) hoặc OLED trang 4.

---

## 17. UART và lệnh

**Drone → PC:** tiền tố `[TLM]`, `[WARN]`, `[ATTACK]`, `[EVT]`, `[STAT]`, `[REPORT]`, `[BBOX]`, `[CMD]`. Thông báo đổi mode giữ đúng chữ của đề. Kết thúc `\r\n`.

**PC → Drone:** gõ trên TeraTerm. Drone tự echo (TeraTerm **tắt** Local echo). Backspace xóa, ESC hủy dòng. Có dòng thông báo đến giữa lúc đang gõ thì dòng đang gõ được vẽ lại.

| Lệnh | Tác dụng | Hợp lệ ở |
|---|---|---|
| `HELP`, `STATUS`, `THRESH` | Hướng dẫn, trạng thái (có điểm và kỷ lục), ngưỡng | Mọi lúc |
| `SET <T\|P\|H\|A\|G\|M>_<LOW\|HIGH> <v>` | Đổi ngưỡng | Mọi lúc |
| `MODE STANDBY\|BATTLE`, `CHARGE`, `RESCUE`, `PING` | Như các thao tác nút | Theo mode |
| `MUTE ON\|OFF`, `HISCORE RESET` | Cấu hình, tự lưu flash | STANDBY |
| `BLACKBOX` | In hộp đen | STANDBY |

Lái, ngắm, khóa mục tiêu và đổi công suất chỉ làm bằng 5-way.

---

## 18. Kiến trúc, thời gian

- Siêu vòng lặp không chặn, định thời bằng `HAL_GetTick()`. ISR chỉ ghi cờ hoặc dữ liệu (EXTI13 nút, EXTI11 IMU, USART1 RX/TX).
- OLED dùng framebuffer: chữ được so với bản cũ, chỉ trang nào đổi mới được gửi, mỗi vòng lặp gửi tối đa một trang (128 byte ≈ 12 ms ở 100 kHz).
- Module ngoài không có thì bỏ qua, phần *(đề)* vẫn chạy đúng. Ở STANDBY, module chưa thấy được dò lại mỗi 2 s (cắm sau vẫn nhận). 5-way hoặc OLED không trả lời 10 lần liên tiếp thì coi như đã rút. Màn hình cuối lúc tự hủy chỉ thử gửi một số lần giới hạn, nên thiếu OLED vẫn tự hủy bình thường.
- Nếu nhiều lần bấm nút dồn lại trong một vòng lặp, chỉ xử lý đến lần đổi mode đầu tiên.
- Đổi tốc độ I2C chỉ ghi lại thanh ghi `TIMINGR` (tắt `PE`, ghi, bật lại), không khởi tạo lại cả I2C1.
- Bus I2C bị kẹt (SDA bị giữ thấp sau một lần timeout) thì tự gỡ: 9 xung SCL + STOP rồi khởi tạo lại I2C1, và thử lại giao dịch đó một lần.
- 5-way chỉ trả lời ở **25 kHz**; OLED và matrix chạy 100 kHz. Firmware đổi tốc độ bus theo từng module.
- Mã ID của 5-way là một số 32-bit little-endian `(VID << 16) | PID`, tức các byte `02 00 86 28`. Lúc dò: thấy địa chỉ 0x03 trả lời thì đọc ID (thử 2 lần) và so khớp.

| Hạng mục | Giá trị |
|---|---|
| Cảm biến + TLM/WARN | 1000 ms |
| Debounce nút / cửa sổ double | 30–50 ms / < 500 ms |
| LED BATTLE / ATTACKED | đảo mỗi 500 / 250 ms |
| Giữ lật để xác nhận / cửa sổ cứu hộ | 300 ms / 10 s |
| 5-way: đọc / giữ | 30 ms / 1 s |
| OLED: vẽ lại / dòng sự kiện | 100 ms / 2 s |
| Sonar: vòng sóng / tiếng vọng / vạch hướng | 4 × 90 ms / 150 + 100·d ms / 0.7 s |
| Nhịp tim | 1200 ms → 350 ms theo số ô nhiễm |
| Lây nhiễm / nghỉ giữa hai đợt | 3 s → 1 s / 2 s |
