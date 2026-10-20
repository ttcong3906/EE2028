# EE2028 A2 — The Last of EE2028: Spec v3.3 (đã chốt 20/10)

Tài liệu này là "hợp đồng" của nhóm: code viết đúng theo đây, muốn đổi hành vi thì sửa file này trước.
Firmware nằm trong **một file `Core/Src/main.c`** (cả drone lẫn tay cầm, chọn bằng cấu hình `ROLE`). Không dùng màn hình laptop để hiển thị: mọi thứ hiện trên OLED và LED matrix, laptop chỉ chạy TeraTerm *(đề)*. Lịch sử quyết định ở mục 17.

## Tổng quan

| Thành phần | Là gì | Phần cứng |
|---|---|---|
| **Drone** (board 1) | Toàn bộ yêu cầu của đề + tính năng thêm | **LED matrix, OLED, buzzer, 5-way switch** (đúng 4 module ngoài) |
| **Tay cầm phi công** (board 2) | Nghiêng để lái drone trinh sát / ngắm, nút để khóa mục tiêu / đổi công suất / gọi cứu hộ | Không module ngoài. **Nối dây trực tiếp** với drone qua UART4, cắm USB chỉ để lấy điện |

**Câu chuyện chung:** drone đi qua đúng vòng đời mà đề cho:
**trinh sát** ở STANDBY (6 cảm biến, lái drone qua vùng sương mù để tìm ổ zombie, tìm nơi ở cho người) → **chiến đấu** ở BATTLE (các ổ đã tìm thấy hiện ra trên bản đồ lây nhiễm, ngắm bằng 5-way hoặc tay cầm) → **bị hạ** ở The Last of EE2028 (SOS, chờ cứu) → **được cứu** hoặc **tự hủy** (cắt liên lạc, hộp đen được giữ lại để điều tra).

Hai màn hình chia việc: **LED matrix là bản đồ** (sương mù trinh sát, bản đồ lây nhiễm, chữ SOS), **OLED là chữ và số** (cảm biến, năng lượng laser, đếm ngược cứu hộ).

Ký hiệu *(đề)* = yêu cầu bắt buộc của đề bài. **Drone đứng một mình (không tay cầm, không module ngoài) vẫn phải chạy đúng toàn bộ phần *(đề)*.**

---

## 1. Phần cứng

| Thiết bị | Kết nối | Vai trò |
|---|---|---|
| Board ×2 | B-L4S5I-IOT01A (STM32L4S5VI), clock 4 MHz MSI mặc định | Drone + tay cầm |
| LED2 (xanh lá) | PB14 | Báo mode *(đề)* |
| User button B1 | PC13, EXTI13, cả hai cạnh | Nút chính *(đề)* |
| HTS221 (T, H), LPS22HB (P), LSM6DSL (A, G), LIS3MDL (M) | I2C2 nội bộ qua BSP | 6 cảm biến *(đề)*, phát hiện lật; trên tay cầm: đo nghiêng |
| LSM6DSL INT1 | PD11, EXTI11 | Ngắt 6D khi đổi hướng |
| UART tới PC | USART1 PB6/PB7 (ST-LINK VCP), 115200 8N1 | TeraTerm *(đề)* |
| Red LED Matrix 8x8 (HT16K33) | Cổng Grove I2C, addr 0x70 | Bản đồ trinh sát / bản đồ lây nhiễm / biểu tượng |
| OLED 128x64 (SSD1306/SSD1315) | Cổng Grove I2C, addr 0x3C | Màn hình chữ: các trang STANDBY, BATTLE, đếm ngược, hộp đen |
| 5-way switch | Cổng Grove I2C, addr 0x03 | Lái drone trinh sát, ngắm bắn, chuyển trang OLED, đổi công suất |
| Buzzer | Cổng Grove **D6** (PB1, TIM3_CH4 PWM) | Âm thanh |
| Dây nối tay cầm | Cổng Grove **UART** trên shield của drone = UART4: PA0 TX (D1), PA1 RX (D0). Tay cầm dùng cùng chân. **Bắt chéo** TX→RX, RX→TX, nối GND | Tay cầm phi công |

**Bus I2C1 (PB8/PB9) dùng chung, đổi tốc độ theo từng thiết bị:** OLED và matrix chạy **100 kHz**, 5-way chạy ở tốc độ mà lần dò tìm ra (**25 kHz** với module của nhóm; ở 50/100 kHz module này không trả lời được). Đổi tốc độ chỉ là ghi lại thanh ghi timing.

---

## 2. Trạng thái hệ thống *(đề)*

```
  power on ─► BOOT ─► STANDBY ◄──double──► BATTLE ◄────────────┐
                     (trinh sát)          (chiến đấu)          │ double / RESCUE / tay cầm
                                              │ lật ≥ 300 ms   │
                                              ▼                │
                                          ATTACKED ────────────┘
                                     (The Last of EE2028)
                                              │ hết 10 s
                                              ▼
                                         TERMINATED  (chỉ RESET mới thoát)
```

---

## 3. Nút bấm B1 trên drone *(đề: phải đọc bằng ngắt)*

- Ngắt EXTI13 bắt cả cạnh nhấn và cạnh nhả, chỉ ghi thời điểm. Không UART, không I2C trong ISR.
- **Debounce:** một lần nhấn chỉ được tính khi nút đã nhả ổn định ≥ 30 ms và cách lần nhấn trước ≥ 50 ms. Giữ nút bao lâu cũng chỉ là một lần.
- **Double press:** lần thứ hai < 500 ms sau lần đầu, xử lý ngay. **Single press:** đủ 500 ms không có lần thứ hai.

| Trạng thái | Single | Double |
|---|---|---|
| STANDBY | Gửi **báo cáo trinh sát** (đề cho phép tự chọn) | Sang BATTLE *(đề)* |
| BATTLE | Nạp laser +3/10 *(đề)* | Về STANDBY *(đề)* |
| ATTACKED | Bỏ qua *(đề)* | Được cứu, về BATTLE *(đề)* |
| TERMINATED | Bỏ qua | Bỏ qua |

---

## 4. Cấu hình

| Mục | Mặc định | Đổi bằng |
|---|---|---|
| **Vai trò board** | `DRONE` | `ROLE DRONE\|CONTROLLER` (đổi xong tự khởi động lại) |
| **Số hiệu drone** | **3** (Drone #3) | `ID <1..9>` |
| **Kiểu lắp** | Mặt chip hướng lên là bình thường | `MOUNT CHIPUP\|ZDOWN` (`ZDOWN` = Z+ hướng xuống đất, đúng chữ đề) |
| **Im lặng** | Tắt | `MUTE ON\|OFF` (SOS vẫn kêu) |

- Lưu vào **trang flash áp chót** (có mã kiểm tra), reset hay mất điện không mất. Mỗi lệnh tự lưu, trả `[CMD] OK … (saved)`.
- Chỉ đổi được ở **STANDBY**.

---

## 5. Khởi động (drone)

| Bước | Làm gì | Matrix / OLED / buzzer | UART |
|---|---|---|---|
| 1 | Đọc cấu hình | | `=== Drone #3 online === mount: chip-up, silent: OFF` |
| 2 | Chờ 300 ms cho các module Grove khởi động, gỡ kẹt bus I2C nếu cần, dò module | | `[EVT] 5-way at 0x03: 25 kHz, stop + read` |
| 3 | Báo kết quả | OLED: trang **SYSTEM CHECK** 2.5 s | `[EVT] System check: sensors OK, LED matrix OK, OLED OK, 5-way OK` |
| 4 | Lưu `P0` (trung bình 10 lần đọc) làm mốc độ cao 0 m | | `[EVT] Altitude zero at 1008.52 hPa` |
| 5 | Nếu flash còn hộp đen của lần bay trước | | `[EVT] Black box of Drone #3 recovered (18 records). Type BLACKBOX to read` |
| 6 | Tạo khu vực trinh sát đầu tiên, vào STANDBY | Matrix: số hiệu 1 s; tiếng khởi động | `Entering STANDBY MODE` *(đề)* |

---

## 6. STANDBY: trinh sát

| Thành phần | Làm gì | Matrix / OLED / buzzer | UART |
|---|---|---|---|
| 6 cảm biến *(đề)* | Đọc mỗi 1 s, so ngưỡng (mục 11) | OLED trang 1 | `[TLM] T: … M: …` **đúng 6 trường**, hoặc `[WARN] …` *(đề)* |
| LED2 *(đề)* | Sáng liên tục | | |
| **Bản đồ trinh sát** | Mục 7 | Matrix | `[EVT] Scouting: nest found at (5,2) (2/3)` |
| **Tìm nơi ở** | Nhiệt độ 20–32 °C **và** độ ẩm 40–70 %RH liên tục 3 s (đề: humidity "used to find new human habitats") | Matrix: ngôi nhà 2 s; chuông | `[EVT] Possible human habitat found: T 26.1 C, H 58 %RH` / `[EVT] Habitat conditions lost` |
| 5-way 4 hướng / tay cầm nghiêng | Lái drone trinh sát 1 ô (giữ thì đi liên tục) | | |
| 5-way giữa **nhấn ngắn** | Sang trang OLED kế tiếp (1 → 2 → 3 → 4 → 1) | | |
| 5-way giữa **giữ 1 s** | Đặt lại mốc độ cao 0 m tại chỗ (như lệnh `HOME`) | Hai tiếng bíp | `[EVT] Altitude zero reset at 1008.40 hPa` |
| PB single | **Báo cáo trinh sát** | Bíp | `[REPORT] Drone #3: flight 02:13, alt +2.1 m, area scanned 37/64, nests found 2/3, habitat found 2x, zombies down 12, shots 6` |

- Lật board ở STANDBY không có tác dụng.
- Khoảng tìm nơi ở nới rộng hơn chuẩn tiện nghi ASHRAE (khoảng 30–60 %RH) để demo được trong phòng lab có máy lạnh ở Singapore.

**Các trang OLED ở STANDBY** (21 ký tự × 8 dòng):

| Trang | Nội dung |
|---|---|
| 1 Sensors | T, H, P, A, G, M kèm đơn vị, độ cao. Dòng cuối: cảnh báo đang có (`WARN: HHI MHI`) hoặc `STATUS: ALL NORMAL` |
| 2 Scouting | Số ổ đã tìm x/3, số ô đã quét x/64, vị trí drone trinh sát, độ cao, trạng thái nơi ở |
| 3 Mission | Thời gian bay, zombie đã hạ, số phát bắn, số lần tìm được nơi ở, tay cầm có kết nối không |
| 4 Black box | Tổng kết chuyến gần nhất (hoặc của drone đã tự hủy), lật từng sự kiện bằng 5-way **trái/phải** (trên trang này trái/phải không lái drone) |

Dòng cuối mỗi trang hiện sự kiện vừa xảy ra trong 2 s (`NEST FOUND 2/3`, `REPORT SENT`, …).

---

## 7. Bản đồ trinh sát (STANDBY)

- Matrix 8x8 là vùng phía trước drone, **cùng tọa độ với bản đồ BATTLE** (hàng 0 xa nhất, hàng 7 gần nhất).
- Mỗi lần vào STANDBY là một **khu vực mới**: toàn bộ phủ sương mù, giấu ngẫu nhiên **3 ổ zombie** ở hàng 0–5. Ngẫu nhiên được gieo từ nhiễu cảm biến nên mỗi lần bật nguồn khác nhau.
- Drone trinh sát xuất phát ở (3,7). Di chuyển 1 ô mỗi lần bằng 5-way, tay cầm hoặc lệnh `AIM <dx> <dy>`.
- Mỗi khi tới một ô, **vùng 3x3 quanh drone** được quét. Thấy ổ thì báo và bíp; tìm đủ 3 ổ thì `[EVT] Scouting complete: all 3 nests located` kèm giai điệu.

| Trên matrix | Nghĩa |
|---|---|
| Sáng liên tục | Sương mù (chưa quét) |
| Tắt | Đã quét, trống |
| Nháy nhanh 4 Hz | Ổ zombie đã tìm thấy |
| Nháy chậm 1 Hz | Drone trinh sát |

---

## 8. BATTLE: chiến đấu

| Thành phần | Làm gì | Matrix / OLED / buzzer | UART |
|---|---|---|---|
| Khi vào | Bắt đầu **đợt 1** từ khu vực vừa trinh sát (mục 9) | | `Entering BATTLE MODE` *(đề)* + `[STAT]` + `[EVT] Wave 1 incoming: 3 outbreaks (2 scouted nests, bigger)` |
| LED2 *(đề)* | Nháy 1 Hz | | |
| PB single *(đề)* | Nạp +3/10, đủ năng lượng thì tự bắn (mục 10) | Bíp nạp; tia + vùng nổ, piuuu / pew / BOOM | `[EVT] LASER charged: 3/10 -> 6/10` |
| 5-way 4 hướng / tay cầm nghiêng | Dời tâm ngắm 1 ô, giữ thì chạy liên tục | Tâm ngắm nháy 4 Hz | |
| 5-way giữa ngắn / tay cầm nhấn 1 lần | Nhảy tới ô nhiễm gần nhất | | `[EVT] Target locked at (3,2)` |
| 5-way giữa **giữ 1 s** / tay cầm nhấn đúp | Đổi công suất: RULE 5/10 → 1 → 2 → 3 → 4 → 5 → RULE | | `[EVT] Weapon: power 3/10, cooldown 3 s` |
| Accel *(đề)* | Phát hiện lật theo kiểu lắp (mục 11) | | → ATTACKED |
| PB double *(đề)* | Về STANDBY, **bản đồ bị xóa**, khu vực trinh sát mới | | `Entering STANDBY MODE` |

**OLED ở BATTLE:** dòng 0 đợt và số mục tiêu (`WAVE 2   TARGETS 11` / `CLEARED!`), dòng 1–2 thanh năng lượng 10 ô cỡ lớn, dòng 3 năng lượng + công suất hoặc cooldown còn lại, dòng 4 tâm ngắm (`TARGET!` nếu đang nhắm trúng ô nhiễm), dòng 5 zombie đã hạ + số phát bắn, dòng 6 sự kiện (`HIT! 5 CELLS CLEARED`, `MISSED!`, `AREA CLEARED!`, `AREA OVERRUN!`), dòng 7 hướng dẫn nút.

BATTLE không gửi telemetry định kỳ, chỉ gửi sự kiện.

---

## 9. Bản đồ lây nhiễm và các đợt

- Ô sáng = khu bị zombie chiếm. Drone ở giữa cạnh dưới.
- **Đợt 1** dùng đúng 3 ổ của khu vực trinh sát: ổ **đã tìm thấy** hiện thành **cụm chữ thập 5 ô** (nhiều zombie hơn), ổ chưa tìm thấy chỉ là 1 ô. Tâm ngắm khóa sẵn vào ổ đã tìm thấy gần nhất (`[EVT] Target locked on scouted nest at (3,4)`).
- **Đợt n ≥ 2** rải **2 + n** ổ ngẫu nhiên (tối đa 12) ở hàng 0–4.
- Trong đợt, mỗi khoảng thời gian một ô nhiễm lây sang một ô kề chưa nhiễm. Khoảng này bắt đầu 3 s ở đợt 1, mỗi đợt nhanh hơn 0.25 s, nhanh nhất 1 s.
- **Dọn sạch bản đồ** → `[EVT] Wave 2 cleared! AREA CLEARED`, tiếng chiến thắng, matrix hiện dấu tích. 2 s sau đợt kế tiếp bắt đầu.
- Thua một đợt không dẫn tới ATTACKED. Theo đề, drone chỉ bị hạ khi bị lật.
- **Quá 75 % (≥ 48/64 ô):** chỉ báo `[EVT] AREA OVERRUN (52/64)`.
- Bị tấn công thì bản đồ dừng lây. Được cứu thì tiếp tục đúng đợt đó.
- Không có 5-way và tay cầm: tâm ngắm tự nhảy tới ô nhiễm gần nhất trước mỗi phát bắn.

---

## 10. Laser gun

- Năng lượng theo đơn vị **1/10**, số nguyên 0–10. Mỗi single press **+3/10** *(đề)*.
- **Chế độ bắn:**
  - **RULE** (mặc định, đúng đề): mỗi phát `p = 5`, đủ năng lượng là tự bắn ngay, **không cooldown** *(đề)*.
  - **SEL** (5-way giữa giữ 1 s, tay cầm nhấn đúp, hoặc lệnh `POWER 1..5`; `POWER RULE` để về đúng đề): `p` = 1–5. Sau mỗi phát có **cooldown `p` giây**, riêng `p = 1` không có. Trong lúc cooldown vẫn nạp được. Hết cooldown mà đủ năng lượng thì tự bắn ngay.
- Tự bắn khi năng lượng ≥ `p`, mỗi lần **một phát**, trừ `p`. Năng lượng thừa để dành, tối đa 10/10.
- Mỗi lần vào BATTLE (chuyến xuất kích mới) chế độ bắn về **RULE**. Được cứu thì giữ nguyên chế độ đang dùng.

| `p` | Vùng nổ | Âm thanh |
|---|---|---|
| 1 | 1 ô | "piuuu": cao, ngắn |
| 2–3 | Chữ thập 5 ô | "pew" |
| 4–5 | Vuông 3x3 | "BOOM": ù trầm 400 → 80 Hz, có rung |

- Thông báo: `[EVT] LASER FIRED (5/10, BOOM) at (3,2): 7 cells cleared. Energy 6/10 -> 1/10. Zombies down: 19`; trượt: `… missed, no zombies there …`; hết cooldown: `[EVT] Laser ready`.
- Matrix: tia quét từ drone tới tâm ngắm (4 khung × 50 ms), rồi vùng nổ nháy 150 ms.

---

## 11. Phát hiện lật *(đề)*

- **Kiểu lắp** theo cấu hình `MOUNT` (mục 4).
- Chức năng **6D** trong LSM6DSL (ngưỡng 60°) báo ngắt INT1 khi hướng thay đổi. Main loop xác nhận bằng `Az` và phải **giữ lật ≥ 300 ms**. Dự phòng: đọc `Az` mỗi 100 ms.
- "Lật" khi trọng lực trên trục Z theo chiều ngược lại lớn hơn 0.5 g.
- LSM6DSL là IMU chứa cả gyroscope và accelerometer. Đó là cách hiểu câu "gyroscope detects upside-down" của đề: gyro chỉ đo tốc độ quay nên không tự biết được úp hay ngửa.
- Sau khi được cứu, drone phải được lật ngửa lại một lần mới có thể bị tấn công tiếp.

---

## 12. Cảm biến, đơn vị và ngưỡng *(đề)*

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

## 13. ATTACKED và TERMINATED *(đề)*

**ATTACKED:**
- LED2 nháy 2 Hz *(đề)*. Khi vào: `[EVT] Drone #3 flipped! The Last of EE2028`. Mỗi 1 s: `[ATTACK] Drone #3 Was Attacked! Rescue in 7 s` *(đề)*.
- Buzzer phát **Morse SOS** liên tục, kêu cả khi im lặng. Matrix hiện chữ S-O-S theo nhịp.
- OLED: `!! UNDER ATTACK !!`, **số giây còn lại cỡ chữ lớn**, `DOUBLE-PRESS TO RESCUE`.
- **Được cứu** khi: double press trên drone *(đề)*, lệnh `RESCUE`, hoặc **nhấn đúp trên tay cầm** (`[EVT] Lab rescue team dispatched by the pilot`). Báo `[EVT] Drone #3 rescued!`, về BATTLE. Matrix mặt cười 1 s, ba nốt đi lên.
- Single press, 5-way, nghiêng tay cầm bị bỏ qua.

**TERMINATED** (hết 10 s không ai cứu):
1. Ghi hộp đen vào flash (mục 14).
2. `[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.`
3. Matrix dấu X; OLED `DRONE #3 LOST / COMMS OFFLINE / BLACK BOX SAVED` (OLED tự giữ hình này sau khi MCU tắt); giai điệu kết thúc.
4. Báo `LINK TERMINATED` cho tay cầm, rồi tắt UART4 và USART1, tắt LED2, tắt các ngắt, MCU vào Standby mode.
5. Chỉ nút **RESET** đen (hoặc rút nguồn) mới thoát được.

---

## 14. Hộp đen (black box)

- Ghi **từng chuyến xuất kích**: bắt đầu khi vào BATTLE (xóa bản ghi cũ trong RAM và flash), tới khi về STANDBY hoặc tự hủy. Được cứu vẫn là cùng chuyến.
- **32 sự kiện gần nhất** (bộ đệm vòng): đợt bắt đầu/dọn xong, nạp, bắn, đổi công suất, quá 75 %, bị lật, được cứu, về STANDBY, tự hủy.
- **Bản tổng kết:** số đợt đã dọn, số phát bắn, số ô đã dọn, thời gian bay, lý do kết thúc (`RETURNED` / `RESCUE TIMEOUT`).
- Khi tự hủy: ghi **một lần** vào **trang flash cuối cùng** (có mã kiểm tra), sống sót qua RESET và mất điện.
- Xem ở STANDBY: lệnh `BLACKBOX` (in tổng kết rồi từng sự kiện) hoặc OLED trang 4.

---

## 15. Tay cầm phi công (board 2, `ROLE CONTROLLER`)

- Không cắm module ngoài. Nối dây với drone (mục 1). USB của tay cầm chỉ để lấy điện; mở TeraTerm vào tay cầm thì thấy những gì nó gửi (`[CTL] TILT 1 0`, …) và dùng được `HELP`, `STATUS`, `ROLE DRONE`.
- Khởi động: `=== Pilot controller online ===`.
- **Giao thức trên dây** (115200 8N1, mỗi dòng kết thúc `\n`):

| Thao tác trên tay cầm | Tay cầm gửi | Drone làm |
|---|---|---|
| Nghiêng > 20° theo một hướng (nhả dưới 12°; giữ thì lặp mỗi 150 ms) | `TILT <dx> <dy>` | STANDBY: lái drone trinh sát. BATTLE: dời tâm ngắm |
| Nhấn 1 lần | `BTN SINGLE` | BATTLE: khóa mục tiêu gần nhất |
| Nhấn đúp | `BTN DOUBLE` | BATTLE: đổi công suất. ATTACKED: **đội cứu hộ** cứu drone |
| (tự động mỗi 1 s) | `ALIVE` | Biết tay cầm còn nối |
| | ← `LINK <mode>` mỗi 250 ms | LED2 của tay cầm theo mode của drone |

- Drone báo `[EVT] Pilot controller connected` khi nhận dòng đầu tiên, `[EVT] Pilot controller lost` khi 3 s không nhận gì. OLED trang 3 và màn BATTLE hiện `PILOT LINKED`.
- **LED2 của tay cầm:** sáng liên tục = STANDBY, nháy 1 Hz = BATTLE, nháy 2 Hz = ATTACKED, tắt = TERMINATED hoặc mất `LINK` quá 2 s.
- Nút của tay cầm dùng chung bộ debounce và phân biệt single/double như drone.
- Không có tay cầm thì drone vẫn đủ chức năng (5-way hoặc tự ngắm).

---

## 16. UART và lệnh

**Drone → PC:** tiền tố `[TLM]`, `[WARN]`, `[ATTACK]`, `[EVT]`, `[STAT]`, `[REPORT]`, `[BBOX]`, `[CMD]`. Thông báo đổi mode giữ đúng chữ của đề. Kết thúc `\r\n`.

**PC → Drone:** gõ trên TeraTerm. Drone tự echo (TeraTerm **tắt** Local echo). Backspace xóa, ESC hủy dòng. Có dòng thông báo đến giữa lúc đang gõ thì dòng đang gõ được vẽ lại.

| Lệnh | Tác dụng | Hợp lệ ở |
|---|---|---|
| `HELP`, `STATUS`, `THRESH` | Hướng dẫn, trạng thái, ngưỡng | Mọi lúc |
| `SET <T\|P\|H\|A\|G\|M>_<LOW\|HIGH> <v>` | Đổi ngưỡng | Mọi lúc |
| `MODE STANDBY\|BATTLE`, `CHARGE`, `RESCUE` | Như các thao tác nút | Theo mode |
| `AIM <dx> <dy>` | STANDBY: lái drone trinh sát. BATTLE: dời tâm ngắm | STANDBY, BATTLE |
| `LOCK`, `POWER RULE\|1..5\|NEXT` | Khóa mục tiêu, chế độ bắn | BATTLE |
| `MUTE ON\|OFF`, `ID <1..9>`, `MOUNT CHIPUP\|ZDOWN`, `ROLE DRONE\|CONTROLLER` | Cấu hình, tự lưu flash | STANDBY |
| `HOME`, `BLACKBOX` | Đặt lại mốc độ cao, in hộp đen | STANDBY |
| `I2CSCAN`, `I2CDUMP`, `I2CSPEED`, `FIVEWAY`, `INPUTS`, `MAG` | **Chỉ dùng lúc phát triển** (`ENABLE_DEV_TOOLS 0` trước khi nộp) | Mọi lúc |

---

## 17. Kiến trúc, thời gian, chia việc, nhật ký

- Siêu vòng lặp không chặn, định thời bằng `HAL_GetTick()`. ISR chỉ ghi cờ hoặc dữ liệu (EXTI13 nút, EXTI11 IMU, USART1 RX/TX, UART4 RX).
- OLED dùng framebuffer: chữ được so với bản cũ, chỉ trang nào đổi mới được gửi, mỗi vòng lặp gửi tối đa một trang (128 byte ≈ 12 ms ở 100 kHz).
- Module ngoài hoặc tay cầm không có thì bỏ qua, phần *(đề)* vẫn chạy đúng. Ở STANDBY, module chưa thấy được dò lại mỗi 2 s (cắm sau vẫn nhận). 5-way không trả lời 10 lần liên tiếp thì coi như đã rút.
- Bus I2C bị kẹt (SDA bị giữ thấp sau một lần timeout) thì tự gỡ: 9 xung SCL + STOP rồi khởi tạo lại I2C1, và thử lại giao dịch đó một lần.
- Mã ID của 5-way là một số 32-bit little-endian `(VID << 16) | PID`, tức các byte `02 00 86 28`.

| Hạng mục | Giá trị |
|---|---|
| Cảm biến + TLM/WARN | 1000 ms |
| Debounce nút / cửa sổ double | 30–50 ms / < 500 ms |
| LED BATTLE / ATTACKED | đảo mỗi 500 / 250 ms |
| Giữ lật để xác nhận / cửa sổ cứu hộ | 300 ms / 10 s |
| 5-way: đọc / giữ | 30 ms / 1 s |
| OLED: vẽ lại / dòng sự kiện | 100 ms / 2 s |
| Lây nhiễm / nghỉ giữa hai đợt | 3 s → 1 s / 2 s |
| Tay cầm: lặp nghiêng / `ALIVE` / `LINK` / mất kết nối | 150 ms / 1 s / 250 ms / 3 s |

**Chia việc:**

| Người | Phần |
|---|---|
| A | Firmware drone: phần *(đề)*, LED matrix, buzzer, 5-way, trinh sát + bản đồ + laser, hộp đen, cấu hình flash |
| B | OLED (driver + các trang), tay cầm (`ROLE CONTROLLER`) và dây nối UART4 ở cả hai phía |
| Cả hai | Giai đoạn đầu (UART + console); thống nhất giao thức dây nối (mục 15) trước khi tách ra |

**Nhật ký quyết định:**

| Ngày | Quyết định |
|---|---|
| 12/10 | Mặt chip hướng lên là bình thường (đổi được bằng `MOUNT`); BATTLE không gửi telemetry định kỳ; chỉ nộp `main.c` |
| 13/10 | `[TLM]` đúng 6 trường; câu chuyện theo vòng đời nhiệm vụ; SOS khi ATTACKED, kêu cả khi im lặng; single press STANDBY = báo cáo trinh sát |
| 13/10 | Hộp đen theo từng chuyến; thoát TERMINATED bằng RESET; độ cao mốc `P0`, bậc 0.1 hPa |
| 13/10 | Bắn một phát mỗi lần; vùng nổ và âm thanh theo công suất; cooldown `p` giây (trừ `p = 1`) chỉ ở chế độ chọn công suất |
| 17/10 | Chiến đấu theo đợt; hộp đen có tổng kết |
| 20/10 | Chỉ 4 module ngoài. Bỏ WiFi và phần hai drone phối hợp |
| 20/10 | Drone mặc định **#3** (đổi bằng `ID 1..9`); vai trò board bằng lệnh `ROLE` lưu flash; cấu hình bằng lệnh, lưu flash |
| 20/10 | Bỏ chế độ TRAINING. I2C chạy đúng chuẩn; 5-way chỉ chạy ở 25 kHz, ID đọc theo little-endian |
| 20/10 | **Bỏ la bàn** (cần hiệu chuẩn, dễ nhiễu từ) → **bản đồ trinh sát**: sương mù, 3 ổ ẩn, quét 3x3, ổ đã tìm thấy thành cụm 5 ô ở đợt 1 |
| 20/10 | **Không dùng màn hình laptop** → bỏ dashboard Python, **OLED thay IR**. Bộ module: LED matrix, OLED, buzzer, 5-way. Tay cầm **nối dây trực tiếp** qua UART4. 5-way giữa nhấn ngắn = chuyển trang OLED. Bus I2C đổi tốc độ theo thiết bị |
