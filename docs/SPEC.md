# EE2028 A2 — The Last of EE2028: Spec v2.2

Tài liệu này là "hợp đồng" giữa bạn và mình: code viết đúng theo đây, muốn đổi hành vi thì sửa file này trước.
Toàn bộ code nằm trong **một file `Core/Src/main.c`**. Lịch sử quyết định ở mục 17.

**Câu chuyện chung:** mỗi module gắn vào một giai đoạn trong vòng đời nhiệm vụ mà đề đã cho sẵn:
xuất kích (DIP switch cấu hình drone trong đội) → **trinh sát** ở STANDBY (6 cảm biến, la bàn về phòng lab, tìm nơi ở cho người, nghe tiếng zombie) → **chiến đấu** ở BATTLE (bản đồ lây nhiễm, ngắm bằng 5-way, chọn công suất laser) → **bị hạ** ở The Last of EE2028 (SOS, chờ cứu) → **tự hủy** (cắt liên lạc, hộp đen được giữ lại để điều tra).

Ký hiệu *(đề)* = yêu cầu bắt buộc của đề bài. Phần còn lại là tính năng thêm.

---

## 1. Phần cứng

| Thiết bị | Kết nối | Vai trò |
|---|---|---|
| Board | B-L4S5I-IOT01A (STM32L4S5VI), clock 4 MHz MSI mặc định | |
| LED2 (xanh lá) | PB14 | Báo mode *(đề)* |
| User button B1 | PC13, EXTI13, cả hai cạnh | Nút chính *(đề)* |
| HTS221 (T, H), LPS22HB (P), LSM6DSL (A, G), LIS3MDL (M) | I2C2 nội bộ qua BSP | 6 cảm biến *(đề)*, la bàn, phát hiện lật |
| LSM6DSL INT1 | PD11, EXTI11 | Ngắt 6D khi đổi hướng |
| UART tới PC | USART1 PB6/PB7 (ST-LINK VCP), 115200 8N1 | Command Centre *(đề)* |
| OLED 0.96" 128x64 | Cổng Grove I2C, addr 0x3C | Buồng lái |
| Red LED Matrix 8x8 (HT16K33) | Cổng Grove **I2C**, addr 0x70 | Bản đồ / la bàn |
| Buzzer | Cổng Grove **D6** (PB1, TIM3_CH4 PWM) | Âm thanh |
| IR reflective sensor | Cổng Grove **A0** (PC5, input số) | Tránh va chạm / zombie áp sát |
| Light sensor v1.2 | Cổng Grove **A1** (PC4, ADC1_IN13) | Tìm nơi ở cho người |
| Sound sensor | Cổng Grove **A2** (PC3, ADC1_IN4) | Nghe tiếng zombie gầm |
| 5-way switch | Cổng Grove I2C, addr 0x03 | Chuyển trang OLED / ngắm bắn |
| 6-pos DIP switch | Cổng Grove I2C, addr 0x04 (sau khi đổi, mục 15) | Cấu hình drone / chọn công suất |

Bốn cổng I2C trên shield nối chung một bus (PB8/PB9).

---

## 2. Trạng thái hệ thống

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
| STANDBY | Gửi **báo cáo trinh sát** (đề cho phép tự chọn) | Sang BATTLE *(đề)* |
| BATTLE | Nạp laser +3/10 *(đề)* | Về STANDBY *(đề)* |
| ATTACKED | Bỏ qua *(đề)* | Được cứu, về BATTLE *(đề)* |
| TERMINATED | Bỏ qua | Bỏ qua |

---

## 4. DIP switch: cấu hình drone và chế độ bắn

Ý nghĩa của DIP switch **phụ thuộc trạng thái**. Đọc mỗi 200 ms.

**Ở STANDBY: cấu hình drone** (chỉ chỉnh được ở STANDBY)

| SW1–SW3 | SW4 | SW5 | SW6 |
|---|---|---|---|
| Số hiệu drone 0–7 (SW1 = bit 0) | Kiểu lắp: tắt = mặt chip hướng lên là bình thường; bật = Z+ hướng xuống đất (đúng chữ đề) | Im lặng (tắt buzzer, trừ SOS) | **Khóa cấu hình**: bật thì SW1–SW5 không đổi cấu hình nữa |

**Ở BATTLE: chế độ bắn** (cấu hình đã được giữ nguyên từ STANDBY)

| SW6 | Chế độ bắn |
|---|---|
| Tắt | **Đúng đề**: mỗi phát 5/10, bắn liên tục, không cooldown. SW1–SW5 **không có tác dụng gì** |
| Bật | **Chọn công suất**: công tắc **số cao nhất** đang bật trong SW1–SW5 là công suất (SW1 = 1/10 … SW5 = 5/10, không công tắc nào bật = 5/10), có cooldown (mục 9) |

- **Vì sao SW6 là khóa cấu hình ở STANDBY:** cùng một công tắc mang hai nghĩa. Nếu bạn gạt SW1–SW5 để chọn công suất trong BATTLE rồi về STANDBY, các vị trí đó sẽ bị đọc thành số hiệu và chế độ im lặng mới, gây đổi cấu hình ngoài ý muốn. SW6 bật thì cấu hình bị khóa, nên chỉnh công suất thoải mái mà không ảnh hưởng gì. Muốn chỉnh cấu hình thì về STANDBY, gạt SW6 tắt, chỉnh SW1–SW5.
- Đổi cấu hình ở STANDBY thì báo `[EVT] Config: drone #5, mount chip-up, silent ON` và OLED cập nhật.
- Đổi chế độ bắn ở BATTLE thì báo `[EVT] Weapon: rule mode 5/10` hoặc `[EVT] Weapon: power 3/10, cooldown 3 s`.
- **Không đọc được DIP switch** (chưa cắm, sai địa chỉ): số hiệu 0, chip hướng lên, không im lặng, chế độ đúng đề.

---

## 5. Khởi động

| Module | Làm gì | Hiển thị | UART |
|---|---|---|---|
| UART | | | `=== Drone #3 online === mount: chip-up, silent: OFF` |
| I2C | Quét bus | | `[EVT] I2C bus: 0x03 0x04 0x3C 0x70` |
| Pressure | Lưu `P0` (trung bình 10 lần đọc) làm mốc **độ cao 0 m** | | `[EVT] Altitude zero at 1008.52 hPa` |
| Magnetometer | Lưu hướng hiện tại làm **hướng về phòng lab** | | `[EVT] Lab bearing registered: 237 deg` |
| Hộp đen | Nếu flash còn hộp đen của lần bay trước | | `[EVT] Black box of Drone #3 recovered (18 records). Type BLACKBOX to read` |
| OLED | **Trang kiểm tra hệ thống** trong 2.5 s đầu: SENSORS, LED MATRIX, 5-WAY, DIP, MIC/LIGHT là `OK` / `NOT FOUND` / `FAIL`, BLACK BOX là `FOUND` / `EMPTY` | | `[EVT] System check: sensors OK, OLED OK, LED matrix OK, mic/light OK, DIP OK, 5-way NOT FOUND` |
| Micro | Đo mức ồn nền (mục 6b) | | `[EVT] Mic ambient baseline …` |
| LED matrix | Hiện số hiệu drone 1 s | Chữ số | |
| Buzzer | Tiếng khởi động ngắn | | |
| | Vào STANDBY | | `Entering STANDBY MODE` *(đề)* |

---

## 6. STANDBY: trinh sát

| Module | Làm gì | Hiển thị | UART |
|---|---|---|---|
| 6 cảm biến *(đề)* | Đọc mỗi 1 s, so ngưỡng (mục 11) | OLED trang 1 | `[TLM] T: … M: …` đúng 6 trường, hoặc `[WARN] …` khi vượt ngưỡng *(đề)* |
| LED2 *(đề)* | Sáng liên tục | | |
| Magnetometer + accel | **La bàn về phòng lab**: hướng có bù nghiêng | LED matrix: mũi tên chỉ về phòng lab, xoay theo khi xoay board. OLED: `HDG 237 SW` | |
| Light + T + H | **Tìm nơi ở cho người**: 20–32 °C, 40–70 %RH, ánh sáng ≥ 30 %, thỏa liên tục 3 s | OLED: `HABITAT: FOUND`. LED matrix: hình ngôi nhà 2 s. Buzzer: hai nốt chuông | `[EVT] Possible human habitat found: T 26.1 C, H 58 %RH, light 62 %`; khi mất: `[EVT] Habitat conditions lost` |
| Sound | **Nghe tiếng gầm** ("moans of the infected"), có chống nghe nhầm buzzer (mục 6b) | OLED: `ZOMBIES HEARD!` 2 s. LED matrix: mặt zombie 1 s. Buzzer bíp | `[EVT] Zombie moans detected (noise 63 %). Recommend BATTLE MODE` |
| IR | **Tránh va chạm** | OLED: `OBSTACLE AHEAD`. LED matrix: dấu `!`. Buzzer bíp | `[EVT] Obstacle at close range, holding position` |
| 5-way | Lên/xuống: đổi trang OLED (mục 13) | | |
| PB single | **Báo cáo trinh sát** gửi về Command Centre qua UART: tóm tắt chuyến trinh sát tới lúc đó | OLED: `REPORT SENT` 1 s | `[REPORT] Drone #3: alt +2.1 m, hdg 237, habitat found 2x, moans heard 3x, zombies down 12` |

- Sự kiện sound và IR có thời gian chờ 3 s để không lặp.

### 6b. Chống micro nghe nhầm tiếng buzzer

Buzzer nằm cạnh micro, tiếng BOOM hay báo động có thể bị tính thành "tiếng zombie gầm" rồi sinh thêm ổ nhiễm. Vì vậy:
1. **Bỏ qua micro** trong lúc buzzer đang kêu và **200 ms** sau khi nó tắt.
2. **Đo mức ồn nền** lúc khởi động (trước mọi âm thanh của board): trung bình 5 cửa sổ 100 ms. In ra `[EVT] Mic ambient baseline 12 %, roar trigger 37 %`.
3. **Ngưỡng kích hoạt** = nền + 25 % (lệnh `SET NOISE` đổi được).
4. **Trễ phục hồi:** sau một lần báo, mức ồn phải xuống dưới nền + 10 % thì mới báo lần tiếp theo. Một tiếng ồn kéo dài chỉ tính một lần.
- Lật board trong STANDBY: không có tác dụng.

---

## 7. BATTLE: chiến đấu

| Module | Làm gì | Hiển thị | UART |
|---|---|---|---|
| Khi vào | Bắt đầu **đợt 1** trên bản đồ mới (mục 8) | | `Entering BATTLE MODE` *(đề)* + `[STAT]` + `[EVT] Wave 1 incoming: 3 outbreaks` |
| LED2 *(đề)* | Nháy 1 Hz | | |
| LED matrix | **Bản đồ lây nhiễm** (mục 8) + tâm ngắm | | |
| OLED | **Buồng lái** (mục 13) | | |
| PB single *(đề)* | Nạp +3/10, đủ năng lượng thì tự bắn (mục 9) | | `[EVT] LASER charged: 3/10 -> 6/10` |
| 5-way | 4 hướng: dời tâm ngắm 1 ô, giữ nghiêng thì chạy liên tục. Giữa: nhảy tới ô nhiễm gần nhất | Tâm ngắm nhấp nháy | |
| Sound | Tiếng gầm → **ổ nhiễm mới** ở một ô mép bản đồ (không tạo trong lúc nghỉ giữa hai đợt) | OLED dòng 7: `MOANS: NEW OUTBREAK` | `[EVT] Zombie moans: new outbreak at (0,5)` |
| IR | Vật áp sát → ô ngay trước drone (hàng 7, cột 3–4) bị nhiễm, nhấp nháy. Buzzer báo động | OLED dòng 7: `RADAR: ZOMBIE CLOSE!` (ưu tiên cao nhất) | `[EVT] ZOMBIE AT CLOSE RANGE!` |
| Accel *(đề)* | Phát hiện lật theo kiểu lắp ở SW4 (mục 10) | | → ATTACKED |
| PB double *(đề)* | Về STANDBY, **bản đồ bị xóa** | | `Entering STANDBY MODE` |

BATTLE không gửi telemetry định kỳ, chỉ gửi sự kiện.

---

## 8. Bản đồ lây nhiễm và các đợt (LED matrix trong BATTLE)

- 8x8 là khu vực phía trước drone nhìn từ trên xuống. Drone ở giữa cạnh dưới. Hàng 0 xa nhất, hàng 7 gần nhất. Ô sáng = khu bị zombie chiếm.
- **Chiến đấu theo đợt:**
  - Đợt *n* rải **2 + n** ổ nhiễm (tối đa 12) ở nửa xa của bản đồ (hàng 0–4): đợt 1 có 3 ổ, đợt 5 có 7 ổ.
  - Trong đợt, mỗi khoảng thời gian một ô nhiễm lây sang một ô kề chưa nhiễm. Khoảng này bắt đầu 3 s ở đợt 1, mỗi đợt nhanh hơn 0.25 s, nhanh nhất 1 s.
  - **Dọn sạch bản đồ** → `[EVT] Wave 2 cleared! AREA CLEARED`, tiếng chiến thắng, LED matrix hiện dấu tích ✓, OLED `WAVE 2  CLEARED!`. **2 s** sau đợt kế tiếp bắt đầu.
  - Thua một đợt không dẫn tới ATTACKED. Theo đề, drone chỉ bị hạ khi bị lật.
- **Tâm ngắm:** nhấp nháy 4 Hz. Ô dưới tâm ngắm đang nhiễm thì OLED báo `TARGET!`.
- **Quá 75 % (≥ 48/64 ô):** chỉ báo `[EVT] AREA OVERRUN (52/64)` và OLED `OVERRUN!`. Báo lại khi xuống dưới 75 % rồi vượt lần nữa.
- Rời BATTLE (về STANDBY) thì bản đồ bị xóa. Vào lại BATTLE thì bắt đầu lại từ đợt 1.
- Khi bị tấn công thì bản đồ dừng lây lan. Được cứu thì tiếp tục đúng đợt đó.

---

## 9. Laser gun

- Năng lượng tính theo đơn vị **1/10**, số nguyên 0–10. Mỗi single press **+3/10** *(đề)*.
- **Công suất mỗi phát** `p`:
  - SW6 tắt (chế độ đúng đề): `p = 5` *(đề)*.
  - SW6 bật (chỉ có tác dụng ở BATTLE): `p` = 1–5 theo DIP switch (mục 4).
- **Tự bắn** khi năng lượng ≥ `p` *(đề)*. Mỗi lần tự bắn chỉ **một phát**, trừ `p`. Năng lượng thừa để dành, tối đa 10/10.
- **Cooldown** (chỉ khi SW6 bật): sau mỗi phát, chờ `p` giây mới được bắn tiếp. Riêng `p = 1` không có cooldown. Trong lúc cooldown vẫn nạp được. Hết cooldown mà đủ năng lượng thì tự bắn ngay vào vị trí tâm ngắm lúc đó.
  - Chế độ đúng đề (SW6 tắt) **không có cooldown**, vì đề yêu cầu đủ năng lượng là bắn ngay.
- **Vùng nổ** quanh tâm ngắm, các ô nhiễm trong vùng bị dọn sạch:

| `p` | Vùng nổ | Âm thanh |
|---|---|---|
| 1 | 1 ô | "piuuu": cao, ngắn, trong |
| 2–3 | Chữ thập 5 ô | "pew": trung |
| 4–5 | Vuông 3x3, 9 ô | "BOOM": tiếng ù trầm quét 400 → 80 Hz, có rung |

- Thông báo:
  - `[EVT] LASER charged: 3/10 -> 6/10`
  - `[EVT] LASER FIRED (5/10, BOOM) at (3,2): 7 cells cleared. Energy 6/10 -> 1/10. Zombies down: 19`
  - `[EVT] LASER FIRED (1/10) at (3,2): missed, no zombies there. Energy 3/10 -> 2/10`
  - Hết cooldown: `[EVT] Laser ready` kèm tiếng tách nhỏ.
- LED matrix: tia quét từ drone tới tâm ngắm (4 khung × 50 ms), rồi vùng nổ nháy sáng 150 ms trước khi tắt.

---

## 10. Phát hiện lật *(đề)*

- **Kiểu lắp** theo SW4: mặc định (tắt) mặt chip hướng lên là bình thường (`Az ≈ +9.8 m/s²`). Bật SW4 thì đảo chiều, đúng nghĩa đen "Z+ hướng xuống đất".
- Chức năng **6D** trong LSM6DSL (ngưỡng 60°) báo ngắt INT1 khi hướng thay đổi. Main loop xác nhận bằng `Az` và phải **giữ tư thế lật ≥ 300 ms**. Dự phòng: đọc `Az` mỗi 100 ms.
- "Lật" khi trọng lực trên trục Z theo chiều ngược lại lớn hơn 0.5 g.
- LSM6DSL là IMU chứa cả gyroscope và accelerometer. Đó là cách hiểu câu "gyroscope detects upside-down" của đề: gyro chỉ đo tốc độ quay nên không tự biết được úp hay ngửa.
- Sau khi được cứu, drone phải được lật ngửa lại một lần mới có thể bị tấn công tiếp.

---

## 11. Cảm biến, đơn vị và ngưỡng *(đề)*

| Ký hiệu | Đại lượng | Đơn vị | Thấp | Cao | Lý do vật lý |
|---|---|---|---|---|---|
| T | Nhiệt độ | `C` | 10.0 | 35.0 | Khoảng hoạt động an toàn của pin Li-ion và điện tử |
| P | Áp suất | `hPa` | 950.0 | 1040.0 | 950 hPa ≈ 540 m (trần bay); trên 1040 hPa gần kỷ lục ở mực nước biển, coi là lỗi |
| H | Độ ẩm tương đối | `%RH` | 20.0 | 80.0 | Trên 80 % dễ đọng sương trên mạch; dưới 20 % dễ tĩnh điện |
| A | Độ lớn gia tốc √(x²+y²+z²) | `m/s^2` | 3.0 | 15.0 | < 0.3 g là rơi tự do; > 1.5 g là va chạm (cảm biến đặt dải ±2 g) |
| G | Độ lớn vận tốc góc | `dps` | — | 300.0 | Quay > 300 °/s là mất kiểm soát |
| M | Độ lớn từ trường | `uT` | 10.0 | 100.0 | Từ trường Trái Đất 25–65 µT; > 100 µT là nhiễu từ, la bàn sai |

- Đổi đơn vị: A `mg × 9.80665 / 1000`; G `mdps / 1000`; M `mGauss / 10 = µT`.
- **Độ cao** (chỉ OLED và báo cáo), mốc 0 m là `P0` lúc khởi động: `h = 44330 × (1 − (P/P0)^0.1903)` m. Gần mặt đất, 0.1 hPa ≈ 0.85 m.
  - Áp suất đọc về luôn dao động vài phần trăm hPa. Để số không nhảy: lọc trung bình trượt, rồi **làm tròn theo bậc 0.1 hPa tính từ P0**. Chỉ nhảy sang bậc mới khi giá trị đã lọc lệch khỏi bậc hiện tại hơn 0.06 hPa (có trễ, để không nhảy qua lại giữa hai bậc). Đứng yên thì độ cao đứng yên, nâng board lên khoảng 1 m mới đổi, mỗi bậc ≈ 0.85 m.
  - Áp suất khí quyển cũng tự trôi theo thời tiết (khoảng 0.1–0.5 hPa mỗi giờ), nên sau lâu độ cao có thể lệch vài mét. Lệnh `HOME` đặt lại mốc 0 m.
  - Dòng `[TLM]` vẫn gửi áp suất thật (P), đúng yêu cầu của đề.
- **Khoảng tìm nơi ở** (khác ngưỡng an toàn): 20–32 °C, 40–70 %RH, ánh sáng ≥ 30 %. Chuẩn tiện nghi ASHRAE hẹp hơn (khoảng 30–60 %RH); ta nới rộng để demo được trong phòng lab có máy lạnh ở Singapore.

---

## 12. La bàn về phòng lab

- Hướng (heading) = `atan2` của từ trường ngang, có **bù nghiêng** bằng accelerometer (roll/pitch), đơn vị độ 0–359, theo kiểu lắp ở SW4.
- **Hướng về phòng lab** = hướng của drone lúc khởi động (drone được đăng ký ở phòng lab, mũi hướng về phòng lab). Lệnh `HOME` đăng ký lại cả hướng về lab lẫn mốc độ cao 0 m.
- LED matrix ở STANDBY: mũi tên 8 hướng chỉ `lab_bearing − heading`.
- **Hiệu chuẩn** (nhiễu từ cứng của board làm la bàn lệch): lệnh `CALIB`. Xoay board nằm ngang đủ 1–2 vòng trong 10 s, board lấy min/max của trục X, Y làm offset. OLED hiện `CALIBRATING… ROTATE`. Kết quả dùng tới lần reset sau.

---

## 13. OLED 128x64 (21 ký tự × 8 dòng, có font chữ lớn 2x)

**Ưu tiên hiển thị theo tình huống:** ATTACKED → vật / zombie áp sát → cảnh báo cảm biến → hiệu ứng bắn (trúng, trượt, AREA CLEARED) → thông báo thường. Một thông báo đang hiện không bị thông báo kém quan trọng hơn ghi đè. Khi bị lật, mọi hiệu ứng bắn trên LED matrix và OLED nhường chỗ ngay.

**Khởi động:** trang kiểm tra hệ thống (mục 5) trong 2.5 s.

**STANDBY:** 5-way lên/xuống đổi trang (4 trang). Hướng la bàn và thống kê nằm ở các trang phụ.

```
Trang 1: cảm biến         Trang 2: điều hướng        Trang 3: nhiệm vụ          Trang 4: hộp đen
+---------------------+   +---------------------+   +---------------------+   +---------------------+
|DRONE#3 STANDBY  1/4 |   |DRONE#3 STANDBY  2/4 |   |DRONE#3 STANDBY  3/4 |   |DRONE#3 STANDBY  4/4 |
|T 25.3C  H 61.2%     |   |HDG 237 SW           |   |HABITAT FOUND: 2x    |   |BLACK BOX: LOST DRONE|
|P 1008.5hPa          |   |LAB BEARING 012 N    |   |MOANS HEARD:   3x    |   |WAVES 3  SHOTS 12    |
|ALT   +3.2 m         |   |TURN  +135 deg       |   |ZOMBIES DOWN: 12     |   |CELLS CLEARED 28     |
|A  9.79 m/s2         |   |ALT   +3.2 m         |   |SHOTS FIRED:   6     |   |END: RESCUE TIMEOUT  |
|G  1.25 dps          |   |LIGHT 62%            |   |FLIGHT 03:12         |   |EVT 05/18 t=41.2s    |
|M  45.3 uT           |   |NOISE  8% (BASE 6)   |   |                     |   |FIRE P5 (3,2) x7     |
|STATUS: ALL NORMAL   |   |HABITAT: SEARCHING   |   |1x REPORT 2x BATTLE  |   |<> EVENTS  ^v PAGES  |
+---------------------+   +---------------------+   +---------------------+   +---------------------+
```
- Dòng cuối trang 1 theo ưu tiên: `OBSTACLE AHEAD` → `WARN: T HI, M HI` → `REPORT SENT` / thông báo thường → `STATUS: ALL NORMAL`.
- Trang 4: 5-way trái/phải lướt từng sự kiện của hộp đen.

**BATTLE (màn hình hành động):**
```
+---------------------+
|WAVE 2   TARGETS  5  |   ← lúc nghỉ: WAVE 2   CLEARED!
|[■][■][■][■][■][■][ ]|   ← thanh năng lượng lớn, 2 dòng, 10 ô
|[ ][ ][ ][ ]         |
|ENERGY  6/10 P5 BOOM |   ← lúc cooldown: ENERGY  6/10 COOL3.2
|AIM (3,2)    TARGET! |
|DOWN 12   WAVES CLR 1|
|RADAR: CLEAR         |   ← ZOMBIE CLOSE! > HIT!/MISSED!/AREA CLEARED! > OVERRUN!
|1x CHARGE 2x STANDBY |
+---------------------+
```

**ATTACKED:** `!! UNDER ATTACK !!`, `DRONE #3  RESCUE IN`, **số giây cứu hộ chữ lớn 2x**, `DOUBLE-PRESS TO RESCUE`, `SOS BEACON ON`.
**TERMINATED:** `DRONE #3 LOST`, `COMMS OFFLINE`, `BLACK BOX SAVED`, `END: RESCUE TIMEOUT`, `press RESET`.

Framebuffer 1 KB trong RAM, mỗi vòng lặp chỉ đẩy một trang (page) thay đổi.

---

## 14. LED matrix, buzzer, ATTACKED, TERMINATED

| Trạng thái | LED matrix | Buzzer |
|---|---|---|
| Khởi động | Số hiệu drone 1 s | Tiếng khởi động |
| STANDBY | Mũi tên la bàn; sự kiện: nhà / zombie / `!` trong 1–2 s | Chuông nơi ở, bíp cảnh báo |
| BATTLE | Bản đồ lây nhiễm + tâm ngắm | Nạp: bíp ngắn; bắn: piuuu / pew / BOOM; báo động khi IR |
| ATTACKED | Chữ S-O-S theo nhịp Morse | **Morse SOS liên tục, kêu cả khi im lặng** |
| Được cứu | Mặt cười 1 s | Ba nốt đi lên |
| TERMINATED | Dấu X | Giai điệu kết thúc |

**ATTACKED** *(đề)*: LED2 nháy 2 Hz. Khi vào: `[EVT] Drone #3 flipped! The Last of EE2028`. Mỗi 1 s: `[ATTACK] Drone #3 Was Attacked! Rescue in 7 s`. Double press hoặc lệnh `RESCUE` trong 10 s: `[EVT] Drone #3 rescued!`, về BATTLE. Single press, 5-way, IR, sound, light đều bị bỏ qua.

**TERMINATED** *(đề)*:
1. Ghi hộp đen vào flash (mục 15).
2. `[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.`
3. OLED, LED matrix, buzzer như bảng trên. Tắt LED2, tắt UART và các ngắt, MCU vào Standby mode. Chỉ RESET thoát được.
4. **Thoát TERMINATED:** chỉ bằng nút **RESET** đen trên board (hoặc rút cắm nguồn). Board khởi động lại từ đầu, vào STANDBY như một drone mới xuất kích. Đề yêu cầu "terminate the program and cut off all communication", nên không có lệnh hay nút nào khác đánh thức drone được.
5. **Chế độ huấn luyện** (lệnh `TRAINING ON`): thay vì cắt liên lạc, báo `[EVT] TRAINING: drone #3 would self-destruct now. Restarting in 3 s` rồi tự khởi động lại. Hộp đen vẫn được ghi.

---

## 15. Hộp đen (black box)

- Hộp đen ghi **từng chuyến xuất kích**: bắt đầu khi **vào BATTLE** (xóa sạch bản ghi cũ, cả trong RAM lẫn trong flash), ghi tới khi về STANDBY hoặc tự hủy.
- Ghi **32 sự kiện gần nhất** vào RAM (bộ đệm vòng). Mỗi bản ghi gồm thời điểm, loại sự kiện, mode và vài số liệu: vị trí bắn và số ô dọn, công suất, mức ồn, `Az` lúc bị lật…
- Sự kiện được ghi: vào BATTLE, bắt đầu và dọn xong từng đợt, nạp, bắn, tiếng gầm, IR, quá 75 %, đổi chế độ bắn, bị lật, được cứu, về STANDBY, tự hủy.
- **Bản tổng kết chuyến bay** lưu kèm (không bị mất khi bộ đệm vòng quay vòng): số đợt đã dọn, số phát bắn, số ô đã dọn, thời gian bay, **lý do kết thúc** (`RETURNED` = về STANDBY, `RESCUE TIMEOUT` = tự hủy). Được cứu thì vẫn là cùng một chuyến.
- **Khi tự hủy**, bộ đệm cùng số hiệu drone được ghi **một lần** vào **trang flash cuối cùng** (cách xa vùng code ≈ 90 KB trong 2 MB flash), có mã kiểm tra. Nhờ vậy nó sống sót qua RESET và mất điện.
- Lệnh `BLACKBOX` chỉ dùng ở **STANDBY**. Ở TERMINATED thì UART đã bị cắt nên không nhận lệnh được; cách đọc là nhấn RESET, board vào STANDBY, rồi gõ `BLACKBOX`.
  - Nếu flash có hộp đen của drone đã tự hủy: in hộp đen đó. Lúc khởi động cũng báo `[EVT] Black box of Drone #3 recovered (18 records). Type BLACKBOX to read`.
  - Nếu không: in chuyến BATTLE gần nhất đang có trong RAM (chuyến kết thúc bình thường).
- Xem trên OLED: STANDBY trang 4, 5-way trái/phải lướt từng sự kiện (mục 13).
- Ví dụ:
  `[BBOX] LAST MISSION: waves cleared 3, shots fired 12, cells cleared 28, flight 95 s, END: RESCUE TIMEOUT`
  `[BBOX] t=41.2s BATTLE  FIRE p=5 at (3,2), 7 cells cleared`
  `[BBOX] t=44.8s BATTLE  FLIPPED Az=-9.7 m/s^2`
  `[BBOX] t=54.8s ATTACKED  SELF-DESTRUCT (no rescue)`

---

## 16. UART

### Board → PC
Mỗi dòng có tiền tố `[TAG]`, kết thúc `\r\n`. Các thông báo đổi mode giữ đúng chữ của đề.

| Tag | Khi nào |
|---|---|
| (không tag) | `Entering STANDBY MODE`, `Entering BATTLE MODE` *(đề)* |
| `[TLM]` | STANDBY mỗi 1 s, **đúng 6 trường** *(đề)* |
| `[WARN]` | STANDBY mỗi 1 s, mỗi dòng một vi phạm *(đề)* |
| `[ATTACK]` | ATTACKED mỗi 1 s *(đề)* |
| `[EVT]` | Sự kiện |
| `[STAT]` | Đổi mode và lệnh `STATUS` |
| `[REPORT]` | Báo cáo trinh sát (single press STANDBY) |
| `[BBOX]` | Nội dung hộp đen |
| `[CMD]` | Trả lời lệnh |

### PC → Board
Board tự echo và vẽ lại dòng đang gõ khi có tin nhắn mới (TeraTerm **tắt** Local echo). Backspace xóa, ESC hủy dòng.

| Lệnh | Tác dụng | Hợp lệ ở |
|---|---|---|
| `HELP`, `STATUS`, `THRESH` | Hướng dẫn, trạng thái, ngưỡng | Mọi lúc |
| `SET <T\|P\|H\|A\|G\|M>_<LOW\|HIGH> <v>` | Đổi ngưỡng an toàn | Mọi lúc |
| `SET NOISE <pct>` | Mức ồn coi là tiếng gầm | Mọi lúc |
| `MODE STANDBY` / `MODE BATTLE` | Như double press | STANDBY / BATTLE |
| `CHARGE` | Như single press | BATTLE |
| `RESCUE` | Cứu drone | ATTACKED |
| `MUTE ON\|OFF` | Tắt hoặc bật buzzer (SOS vẫn kêu) | Mọi lúc |
| `HOME` | Đăng ký lại hướng về phòng lab và mốc độ cao 0 m | STANDBY |
| `CALIB` | Hiệu chuẩn la bàn 10 s | STANDBY |
| `TRAINING ON\|OFF` | Chế độ huấn luyện (mục 14) | Mọi lúc |
| `BLACKBOX` | Đọc hộp đen (mục 15) | STANDBY |
| `I2CSCAN`, `I2CDUMP`, `I2CADDR`, `INPUTS` | **Chỉ dùng lúc phát triển** (`ENABLE_DEV_TOOLS 0` trước khi nộp). `INPUTS` in trạng thái thô của DIP và 5-way | Mọi lúc |

---

## 17. Kiến trúc, thời gian, tình huống biên

- Siêu vòng lặp không chặn, định thời bằng `HAL_GetTick()`. ISR chỉ ghi cờ hoặc dữ liệu (EXTI13 nút, EXTI11 IMU, USART1 RX/TX).
- ADC chạy trực tiếp bằng thanh ghi (module ADC của HAL tắt trong conf, mà chỉ nộp `main.c`).
- Module ngoài không cắm hoặc không trả lời thì bỏ qua, hệ thống vẫn chạy đúng đề.

| Hạng mục | Giá trị |
|---|---|
| Cảm biến + TLM/WARN | 1000 ms |
| Debounce nút / cửa sổ double | 30–50 ms / < 500 ms |
| LED BATTLE / ATTACKED | đảo mỗi 500 / 250 ms |
| Giữ lật để xác nhận | 300 ms |
| Cửa sổ cứu hộ | 10 s |
| Đọc DIP / 5-way | 200 ms / 30 ms |
| Lây nhiễm | 1 ô mỗi 3 s ở đợt 1, nhanh dần 0.25 s mỗi đợt, tối thiểu 1 s |
| Nghỉ giữa hai đợt | 2 s |
| Bỏ qua micro sau khi buzzer tắt | 200 ms |
| Cooldown laser | `p` giây (chỉ khi SW6 bật, `p ≥ 2`) |
| Chờ giữa sự kiện sound/IR | 3 s |
| Cập nhật la bàn | 100 ms |

---

## 18. Nhật ký quyết định

| Ngày | Quyết định |
|---|---|
| 12/10 | Mặt chip hướng lên là bình thường (nay chọn được bằng SW4) |
| 12/10 | BATTLE không gửi telemetry định kỳ |
| 12/10 | Chỉ nộp `main.c` |
| 13/10 | Cắm module: buzzer D6, IR A0, light A1, sound A2, LED matrix vào cổng I2C |
| 13/10 | `[TLM]` giữ đúng 6 trường |
| 13/10 | Giữ lệnh UART `MODE`/`CHARGE`/`RESCUE`; công cụ I2C chỉ dùng lúc phát triển |
| 13/10 | Bỏ stealth mode; light sensor dùng để tìm nơi ở cho người |
| 13/10 | Câu chuyện chung theo vòng đời nhiệm vụ; mỗi module gắn vào một giai đoạn |
| 13/10 | SOS chuyển sang ATTACKED, kêu cả khi im lặng; single press STANDBY = báo cáo trinh sát |
| 13/10 | Thêm la bàn về phòng lab (magnetometer) |
| 13/10 | Bỏ ý xóa vị trí phòng lab; giữ hộp đen |
| 13/10 | DIP: SW6 chuyển chế độ cấu hình / chọn công suất; công suất lấy công tắc số cao nhất |
| 13/10 | Bắn một phát mỗi lần; vùng nổ và âm thanh theo công suất; cooldown `p` giây (trừ `p = 1`), chỉ khi SW6 bật |
| 13/10 | LED matrix = bản đồ lây nhiễm; OLED = buồng lái. Lây 1 ô/3 s, 3 ổ ban đầu, quá 75 % chỉ báo, bản đồ xóa khi về STANDBY |
| 13/10 | Chế độ huấn luyện chuyển thành lệnh `TRAINING ON/OFF` |
| 13/10 | DIP: ở STANDBY là cấu hình (SW6 = khóa cấu hình), ở BATTLE là chế độ bắn (SW6 tắt = đúng đề, bật = chọn công suất + cooldown) |
| 13/10 | Hộp đen ghi từng chuyến từ lúc vào BATTLE; `BLACKBOX` chỉ ở STANDBY; thoát TERMINATED bằng RESET |
| 13/10 | Độ cao: mốc 0 m = `P0` lúc khởi động; chỉ đổi khi áp suất lọc lệch ≥ 0.1 hPa; `HOME` đặt lại mốc |
| 17/10 | Theo bản improve: chiến đấu theo đợt, chống micro nghe nhầm buzzer, OLED ưu tiên theo tình huống + chữ lớn, hộp đen có tổng kết và xem trên OLED, trang kiểm tra hệ thống lúc khởi động |
| 17/10 | Giữ nguyên DIP như v2.1 (không dùng NORMAL/HARD); đợt sau tự khó hơn đợt trước |
