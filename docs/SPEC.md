# EE2028 A2 — The Last of EE2028: Spec v3

Tài liệu này là "hợp đồng" của nhóm: code viết đúng theo đây, muốn đổi hành vi thì sửa file này trước.
Toàn bộ firmware nằm trong **một file `Core/Src/main.c`**. Lịch sử quyết định ở mục 20.

**Giới hạn phần cứng:** tối đa **4 module ngoài** cho cả nhóm. Nhóm dùng **OLED, LED matrix, buzzer, 5-way switch**. Module WiFi ISM43362 có sẵn trên board, không tính là module ngoài.

**Câu chuyện chung:** phòng lab có một đội drone ("a batch of drones"). Hai board của nhóm là hai drone cùng đội, liên lạc qua WiFi với **Command Centre** trên laptop:
- **Drone săn (HUNTER):** board có cắm OLED + LED matrix + buzzer + 5-way.
- **Drone trinh sát (SCOUT):** board không cắm module ngoài, chỉ dùng cảm biến trên board.

Mỗi drone đi qua vòng đời mà đề đã cho: **trinh sát** (STANDBY) → **chiến đấu** (BATTLE) → **bị hạ** (The Last of EE2028) → **được cứu** hoặc **tự hủy**. Hai drone giúp nhau: trinh sát báo vị trí zombie cho drone săn, và drone nào bị lật thì đồng đội có thể gửi cứu viện.

Ký hiệu *(đề)* = yêu cầu bắt buộc của đề bài. Phần còn lại là tính năng thêm. **Mỗi board đứng một mình (không WiFi, không đồng đội) vẫn phải chạy đúng toàn bộ phần *(đề)*.**

---

## 1. Phần cứng

| Thiết bị | Kết nối | Vai trò |
|---|---|---|
| Board ×2 | B-L4S5I-IOT01A (STM32L4S5VI), clock 4 MHz MSI mặc định | Hai drone |
| LED2 (xanh lá) | PB14 | Báo mode *(đề)* |
| User button B1 | PC13, EXTI13, cả hai cạnh | Nút chính *(đề)* |
| HTS221 (T, H), LPS22HB (P), LSM6DSL (A, G), LIS3MDL (M) | I2C2 nội bộ qua BSP | 6 cảm biến *(đề)*, la bàn, phát hiện lật, phát hiện va chạm |
| LSM6DSL INT1 | PD11, EXTI11 | Ngắt 6D khi đổi hướng |
| UART tới PC | USART1 PB6/PB7 (ST-LINK VCP), 115200 8N1 | Command Centre có dây *(đề)* |
| WiFi ISM43362 (trên board) | SPI3 PC10/PC11/PC12, CS PE0, RST PE8, WAKEUP PB13, DATARDY PE1 | Command Centre không dây, liên lạc giữa hai drone, quét mạng |
| OLED 0.96" 128x64 | Cổng Grove I2C, addr 0x3C | Buồng lái (chỉ HUNTER) |
| Red LED Matrix 8x8 (HT16K33) | Cổng Grove I2C, addr 0x70 | Bản đồ / la bàn (chỉ HUNTER) |
| Buzzer | Cổng Grove **D6** (PB1, TIM3_CH4 PWM) | Âm thanh (chỉ HUNTER) |
| 5-way switch | Cổng Grove I2C, addr 0x03 | Đổi trang, ngắm bắn, đổi công suất, cứu đồng đội (chỉ HUNTER) |

Bốn cổng I2C trên shield nối chung một bus (PB8/PB9).

---

## 2. Trạng thái hệ thống *(đề)*

```
  power on ─► BOOT ─► STANDBY ◄──double──► BATTLE ◄────────────┐
                     (trinh sát)          (chiến đấu)          │ double / RESCUE / cứu viện của đồng đội
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

- Ngắt EXTI13 bắt cả cạnh nhấn và cạnh nhả, chỉ ghi thời điểm. Không UART, không I2C, không WiFi trong ISR.
- **Debounce:** một lần nhấn chỉ được tính khi nút đã nhả ổn định ≥ 30 ms và cách lần nhấn trước ≥ 50 ms. Giữ nút bao lâu cũng chỉ là một lần.
- **Double press:** lần thứ hai < 500 ms sau lần đầu, xử lý ngay. **Single press:** đủ 500 ms không có lần thứ hai.

| Trạng thái | Single | Double |
|---|---|---|
| STANDBY | Gửi **báo cáo trinh sát** (đề cho phép tự chọn). Nếu đồng đội đang phát SOS: **gửi cứu viện** thay vì báo cáo | Sang BATTLE *(đề)* |
| BATTLE | Nạp laser +3/10 *(đề)* | Về STANDBY *(đề)* |
| ATTACKED | Bỏ qua *(đề)* | Được cứu, về BATTLE *(đề)* |
| TERMINATED | Bỏ qua | Bỏ qua |

---

## 4. Cấu hình drone (thay cho DIP switch)

| Mục | Mặc định | Đổi bằng |
|---|---|---|
| **Số hiệu drone** | Tính từ **mã định danh duy nhất 96-bit (UID)** của chip: `10 + (UID0 ^ UID1 ^ UID2) % 90`, ra số 10–99. Hai board tự có hai số khác nhau | `ID <10..99>` |
| **Vai trò** | HUNTER nếu thấy OLED và LED matrix trên bus, ngược lại SCOUT | `ROLE HUNTER\|SCOUT\|AUTO` |
| **Kiểu lắp** | Mặt chip hướng lên là bình thường | `MOUNT CHIPUP\|ZDOWN` (`ZDOWN` = Z+ hướng xuống đất, đúng chữ đề) |
| **Im lặng** | Tắt | `MUTE ON\|OFF` (SOS vẫn kêu) |
| **WiFi** | Chưa cấu hình | `WIFI <ssid> <password>`, `SERVER <ip> <port>` |

- Cấu hình được lưu vào **trang flash áp chót** (có mã kiểm tra), nên reset hay mất điện không mất. Mỗi lệnh đổi cấu hình tự lưu và trả `[CMD] OK … (saved)`.
- Cấu hình chỉ đổi được ở **STANDBY**.

---

## 5. Khởi động

| Bước | Làm gì | Hiển thị (HUNTER) | UART |
|---|---|---|---|
| 1 | Đọc cấu hình từ flash, tính số hiệu từ UID | | `=== Drone #37 online === role: HUNTER, mount: chip-up, silent: OFF` |
| 2 | Kiểm tra module | OLED **trang kiểm tra hệ thống**: SENSORS / LED MATRIX / 5-WAY / WIFI / BLACK BOX | `[EVT] System check: sensors OK, OLED OK, LED matrix OK, 5-way OK, WiFi module OK` |
| 3 | Lưu `P0` (trung bình 10 lần đọc) làm mốc độ cao 0 m | | `[EVT] Altitude zero at 1008.52 hPa` |
| 4 | Lưu hướng hiện tại làm **hướng về phòng lab** | | `[EVT] Lab bearing registered: 237 deg` |
| 5 | Nếu có cấu hình WiFi: kết nối hotspot rồi Command Centre (tối đa khoảng 10 s) | `WIFI CONNECTING…` → `WIFI OK 192.168.43.12` | `[EVT] WiFi connected, IP 192.168.43.12, Command Centre 192.168.43.1:5028` hoặc `[EVT] WiFi unavailable, UART only` |
| 6 | Nếu flash còn hộp đen của lần bay trước | | `[EVT] Black box of Drone #37 recovered (18 records). Type BLACKBOX to read` |
| 7 | Vào STANDBY | LED matrix hiện số hiệu 1 s, buzzer kêu khởi động | `Entering STANDBY MODE` *(đề)* |

---

## 6. STANDBY: trinh sát

| Thành phần | Làm gì | Hiển thị (HUNTER) | UART |
|---|---|---|---|
| 6 cảm biến *(đề)* | Đọc mỗi 1 s, so ngưỡng (mục 12) | OLED trang 1 | `[TLM] T: … M: …` **đúng 6 trường**, hoặc `[WARN] …` khi vượt ngưỡng *(đề)* |
| LED2 *(đề)* | Sáng liên tục | | |
| La bàn (mục 13) | Hướng có bù nghiêng, mũi tên về phòng lab | LED matrix: mũi tên. OLED trang 2: `HDG 237 SW`, `TURN +135` | |
| Tìm nơi ở (mục 14) | T + H dễ chịu và quét thấy nhiều mạng WiFi | OLED `HABITAT: FOUND`, matrix hình ngôi nhà 2 s, chuông | `[EVT] Possible human habitat: 12 WiFi networks, T 26.1 C, H 58 %RH` |
| **Va chạm zombie** (SCOUT) | Độ lớn gia tốc vượt `A_HIGH` (lắc mạnh): coi là va chạm với zombie, báo cho drone săn | | `[EVT] Zombie contact! Reporting to hunter (column 3)` + gửi `$CONTACT` |
| Nhận `$CONTACT` (HUNTER) | Trinh sát báo có zombie | OLED `SCOUT: ZOMBIES AHEAD`, matrix mặt zombie 1 s, bíp | `[EVT] Scout #52 reports zombie contact. Recommend BATTLE MODE` |
| 5-way (HUNTER) | Lên/xuống đổi trang OLED. Ở trang 2, nhấn giữa để **quét WiFi**. Ở trang 4, trái/phải lướt hộp đen | | |
| PB single | **Báo cáo trinh sát**, hoặc gửi cứu viện nếu đồng đội đang SOS | OLED `REPORT SENT` / `ASSIST SENT` | `[REPORT] Drone #37: alt +2.1 m, hdg 237, habitat found 2x, contacts 3x, zombies down 12` |

- Lật board trong STANDBY: không có tác dụng.
- Sự kiện va chạm có thời gian chờ 3 s để không lặp.

---

## 7. BATTLE: chiến đấu

| Thành phần | Làm gì | Hiển thị (HUNTER) | UART |
|---|---|---|---|
| Khi vào | Bắt đầu **đợt 1** trên bản đồ mới (mục 8) | | `Entering BATTLE MODE` *(đề)* + `[STAT]` + `[EVT] Wave 1 incoming: 3 outbreaks` |
| LED2 *(đề)* | Nháy 1 Hz | | |
| LED matrix | Bản đồ lây nhiễm + tâm ngắm | | |
| OLED | Màn hình hành động (mục 16) | | |
| PB single *(đề)* | Nạp +3/10, đủ năng lượng thì tự bắn (mục 9) | | `[EVT] LASER charged: 3/10 -> 6/10` |
| 5-way 4 hướng | Dời tâm ngắm 1 ô, giữ nghiêng thì chạy liên tục | Tâm ngắm nhấp nháy | |
| 5-way giữa, bấm ngắn | Nhảy tới ô nhiễm gần nhất. Nếu đồng đội đang SOS: **gửi cứu viện** | `TARGET LOCKED` / `ASSIST SENT` | |
| 5-way giữa, **giữ 1 s** | Đổi công suất: RULE 5/10 → 1 → 2 → 3 → 4 → 5 → RULE (mục 9) | `POWER 3/10 PEW SEL` | `[EVT] Weapon: power 3/10, cooldown 3 s` |
| Nhận `$CONTACT` | Ổ nhiễm mới ở hàng 0, đúng cột trinh sát báo | | `[EVT] Scout #52 reports zombies at column 3: new outbreak (3,0)` |
| Accel *(đề)* | Phát hiện lật theo kiểu lắp (mục 10) | | → ATTACKED |
| PB double *(đề)* | Về STANDBY, **bản đồ bị xóa** | | `Entering STANDBY MODE` |

- BATTLE không gửi telemetry định kỳ, chỉ gửi sự kiện.
- **SCOUT** cũng vào BATTLE được, với laser, phát hiện lật và cứu hộ đúng đề. Bản đồ của SCOUT chỉ tồn tại trong bộ nhớ, trạng thái in qua UART.

---

## 8. Bản đồ lây nhiễm và các đợt

- 8x8 là khu vực phía trước drone nhìn từ trên xuống. Drone ở giữa cạnh dưới. Hàng 0 xa nhất, hàng 7 gần nhất. Ô sáng = khu bị zombie chiếm.
- Đợt *n* rải **2 + n** ổ nhiễm (tối đa 12) ở hàng 0–4. Trong đợt, mỗi khoảng thời gian một ô nhiễm lây sang một ô kề chưa nhiễm. Khoảng này bắt đầu 3 s ở đợt 1, mỗi đợt nhanh hơn 0.25 s, nhanh nhất 1 s.
- **Dọn sạch bản đồ** → `[EVT] Wave 2 cleared! AREA CLEARED`, tiếng chiến thắng, matrix hiện dấu tích, OLED `WAVE 2  CLEARED!`. 2 s sau đợt kế tiếp bắt đầu. Trong lúc nghỉ, `$CONTACT` không tạo ổ mới.
- Thua một đợt không dẫn tới ATTACKED. Theo đề, drone chỉ bị hạ khi bị lật.
- **Quá 75 % (≥ 48/64 ô):** chỉ báo `[EVT] AREA OVERRUN (52/64)` và OLED `OVERRUN!`.
- Về STANDBY thì bản đồ bị xóa, vào lại BATTLE thì bắt đầu từ đợt 1. Bị tấn công thì bản đồ dừng lây. Được cứu thì tiếp tục đúng đợt đó.

---

## 9. Laser gun

- Năng lượng theo đơn vị **1/10**, số nguyên 0–10. Mỗi single press **+3/10** *(đề)*.
- **Chế độ bắn:**
  - **RULE** (mặc định, đúng đề): mỗi phát `p = 5`, đủ năng lượng là tự bắn ngay, **không cooldown** *(đề)*.
  - **SEL** (chọn bằng giữ nút giữa 5-way 1 s, hoặc lệnh `POWER 1..5`; `POWER RULE` để về đúng đề): `p` = 1–5. Sau mỗi phát có **cooldown `p` giây**, riêng `p = 1` không có. Trong lúc cooldown vẫn nạp được. Hết cooldown mà đủ năng lượng thì tự bắn ngay.
- Tự bắn khi năng lượng ≥ `p`, mỗi lần **một phát**, trừ `p`. Năng lượng thừa để dành, tối đa 10/10.
- **Vùng nổ** quanh tâm ngắm:

| `p` | Vùng nổ | Âm thanh |
|---|---|---|
| 1 | 1 ô | "piuuu": cao, ngắn |
| 2–3 | Chữ thập 5 ô | "pew" |
| 4–5 | Vuông 3x3 | "BOOM": ù trầm 400 → 80 Hz, có rung |

- Thông báo: `[EVT] LASER FIRED (5/10, BOOM) at (3,2): 7 cells cleared. Energy 6/10 -> 1/10. Zombies down: 19`; trượt: `… missed, no zombies there …`; hết cooldown: `[EVT] Laser ready`.
- Matrix: tia quét từ drone tới tâm ngắm (4 khung × 50 ms), rồi vùng nổ nháy 150 ms.

---

## 10. Phát hiện lật *(đề)*

- **Kiểu lắp** theo cấu hình `MOUNT` (mục 4).
- Chức năng **6D** trong LSM6DSL (ngưỡng 60°) báo ngắt INT1 khi hướng thay đổi. Main loop xác nhận bằng `Az` và phải **giữ lật ≥ 300 ms**. Dự phòng: đọc `Az` mỗi 100 ms.
- "Lật" khi trọng lực trên trục Z theo chiều ngược lại lớn hơn 0.5 g.
- LSM6DSL là IMU chứa cả gyroscope và accelerometer. Đó là cách hiểu câu "gyroscope detects upside-down" của đề: gyro chỉ đo tốc độ quay nên không tự biết được úp hay ngửa.
- Sau khi được cứu, drone phải được lật ngửa lại một lần mới có thể bị tấn công tiếp.

---

## 11. ATTACKED và TERMINATED *(đề)*

**ATTACKED:**
- LED2 nháy 2 Hz *(đề)*. Khi vào: `[EVT] Drone #37 flipped! The Last of EE2028`. Mỗi 1 s: `[ATTACK] Drone #37 Was Attacked! Rescue in 7 s` *(đề)*.
- Buzzer phát **Morse SOS** liên tục, kêu cả khi im lặng. Matrix hiện chữ S-O-S theo nhịp. OLED: số giây cứu hộ **chữ lớn 2x**.
- Qua WiFi, mỗi 1 s gửi `$SOS,<id>,<secs>` cho đồng đội.
- **Được cứu** khi: double press *(đề)*, lệnh `RESCUE`, hoặc nhận `$RESCUE` của đồng đội. Báo `[EVT] Drone #37 rescued!` / `… rescued by scout #52!`, về BATTLE.
- Single press, 5-way, `$CONTACT` đều bị bỏ qua.

**TERMINATED** (hết 10 s không ai cứu):
1. Ghi hộp đen vào flash (mục 17).
2. `[EVT] Drone #37: no rescue. Black box saved. Self-destructing. Cutting all communication.`
3. Qua WiFi: gửi `$BBOX` (bản tổng kết hộp đen) rồi đóng kết nối, ngắt WiFi, **giữ module WiFi ở trạng thái reset** (radio tắt hẳn).
4. OLED `DRONE #37 LOST / COMMS OFFLINE`, matrix dấu X, giai điệu kết thúc. Tắt LED2, tắt UART và các ngắt, MCU vào Standby mode.
5. Chỉ nút **RESET** đen (hoặc rút nguồn) mới thoát được. Board khởi động lại như một drone mới xuất kích.
6. **Chế độ huấn luyện** (`TRAINING ON`): thay vì cắt liên lạc, báo `[EVT] TRAINING: drone #37 would self-destruct now. Restarting in 3 s` rồi tự khởi động lại. Hộp đen vẫn được ghi.

---

## 12. Cảm biến, đơn vị và ngưỡng *(đề)*

| Ký hiệu | Đại lượng | Đơn vị | Thấp | Cao | Lý do vật lý |
|---|---|---|---|---|---|
| T | Nhiệt độ | `C` | 10.0 | 35.0 | Khoảng hoạt động an toàn của pin Li-ion và điện tử |
| P | Áp suất | `hPa` | 950.0 | 1040.0 | 950 hPa ≈ 540 m (trần bay); trên 1040 hPa gần kỷ lục ở mực nước biển, coi là lỗi |
| H | Độ ẩm tương đối | `%RH` | 20.0 | 80.0 | Trên 80 % dễ đọng sương trên mạch; dưới 20 % dễ tĩnh điện |
| A | Độ lớn gia tốc √(x²+y²+z²) | `m/s^2` | 3.0 | 15.0 | < 0.3 g là rơi tự do; > 1.5 g là va chạm (cảm biến đặt dải ±2 g) |
| G | Độ lớn vận tốc góc | `dps` | — | 300.0 | Quay > 300 °/s là mất kiểm soát |
| M | Độ lớn từ trường | `uT` | 10.0 | 100.0 | Từ trường Trái Đất 25–65 µT; > 100 µT là nhiễu từ, la bàn sai |

- Đổi đơn vị: A `mg × 9.80665 / 1000`; G `mdps / 1000`; M `mGauss / 10 = µT`.
- **Độ cao** (OLED, báo cáo): mốc 0 m là `P0` lúc khởi động, `h = 44330 × (1 − (P/P0)^0.1903)` m. Lọc trung bình trượt rồi làm tròn theo bậc 0.1 hPa (≈ 0.85 m) tính từ P0, có trễ 0.06 hPa để không nhảy. Lệnh `HOME` đặt lại mốc. Dòng `[TLM]` vẫn gửi áp suất thật.

---

## 13. La bàn về phòng lab

- Heading = `atan2` của từ trường ngang, **bù nghiêng** bằng accelerometer, 0–359°, theo kiểu lắp.
- **Hướng về phòng lab** = hướng lúc khởi động. `HOME` đăng ký lại hướng và mốc độ cao.
- Matrix ở STANDBY (HUNTER): mũi tên 8 hướng chỉ `lab_bearing − heading`.
- `CALIB`: xoay board nằm ngang 1–2 vòng trong 10 s, lấy min/max trục X, Y làm offset nhiễu từ cứng.

---

## 14. WiFi: tìm nơi ở và khoảng cách tới phòng lab

- **Quét** (`SCAN`, hoặc nhấn giữa 5-way ở OLED trang 2): liệt kê mạng WiFi xung quanh. Lệnh này chặn chương trình khoảng 2–4 s nên báo trước `[EVT] WiFi scan (telemetry paused ~3 s)`.
- **Dấu hiệu người sống sót:** nơi có nhiều router là nơi từng có người ở. **Nơi ở** = nhiệt độ 20–32 °C và độ ẩm 40–70 %RH liên tục 3 s, **và** lần quét gần nhất (trong 60 s) thấy ≥ 3 mạng. Chuẩn tiện nghi ASHRAE hẹp hơn (khoảng 30–60 %RH); ta nới rộng để demo được trong phòng lab có máy lạnh ở Singapore.
- **Khoảng cách tới phòng lab:** hotspot đóng vai phòng lab. RSSI của hotspot trong lần quét: ≥ −60 dBm `NEAR`, −60…−75 `MID`, < −75 `FAR` (báo `OUT OF RANGE`). OLED trang 2: `LAB SIGNAL -63 dBm MID`.

---

## 15. WiFi: Command Centre và liên lạc giữa hai drone

- Laptop và hai board cùng vào **hotspot điện thoại** (WPA2). WiFi của NUS dùng WPA2-Enterprise nên module không vào được.
- Laptop chạy `tools/command_centre.py`: **TCP server** cổng 5028, có bảng điều khiển cho cả hai drone, nhật ký sự kiện, ô gửi lệnh tới từng drone. Server **chuyển tiếp** các tin `$…` giữa hai drone.
- Mỗi drone là **TCP client**. UART vẫn chạy song song, nên WiFi là add-on đúng như đề yêu cầu.
- **Gửi lên:** mọi dòng drone gửi qua UART cũng được gom lại và gửi qua WiFi **mỗi 500 ms một gói**. **Nhận:** kiểm tra mỗi 200 ms. Nhận được lệnh thì xử lý như lệnh gõ trên UART, nhận được `$…` thì xử lý theo bảng dưới.
- Mất kết nối thì thử lại mỗi 10 s. UART không bị ảnh hưởng.

| Tin | Khi nào | Bên nhận làm gì |
|---|---|---|
| `$HB,<id>,<role>,<mode>,<wave>` | Mỗi 1 s | Ghi nhận đồng đội. HUNTER: OLED `WINGMAN #52 STANDBY`. Mất quá 3 s: `[EVT] Wingman #52 signal lost` |
| `$CONTACT,<id>,<col>` | SCOUT phát hiện va chạm (mục 6) | HUNTER ở BATTLE: ổ nhiễm mới tại `(col, 0)`. Ở STANDBY: cảnh báo |
| `$SOS,<id>,<secs>` | Mỗi 1 s khi ATTACKED | Báo động: `[EVT] Wingman #37 under attack! Rescue in 7 s`. HUNTER: OLED ưu tiên cao nhất, matrix nháy S, buzzer. Cứu: **5-way giữa** (HUNTER), **single press ở STANDBY** (SCOUT), hoặc lệnh `ASSIST` |
| `$RESCUE,<to>,<from>` | Đồng đội gửi cứu viện | Drone `<to>` đang ATTACKED: được cứu như double press |
| `$BBOX,<id>,<waves>,<shots>,<cells>,<end>` | Ngay trước khi tự hủy | Chuyển tiếp về Command Centre: `[EVT] Wingman #37 lost. Last mission: 3 waves, 12 shots, 28 cells, END: RESCUE TIMEOUT` |

- Hai drone trùng số hiệu: `[EVT] Wingman has the same drone ID, use ID <n>`.
- Không có WiFi hoặc không có đồng đội: mọi thứ ở mục này đều bị bỏ qua, không ảnh hưởng phần *(đề)*.

---

## 16. OLED 128x64 (HUNTER)

**Ưu tiên hiển thị:** ATTACKED → SOS của đồng đội → cảnh báo cảm biến → hiệu ứng bắn → thông báo thường. Thông báo đang hiện không bị thông báo kém quan trọng hơn ghi đè. Khi bị lật, mọi hiệu ứng nhường chỗ ngay.

**STANDBY**, 5-way lên/xuống đổi trang:
```
Trang 1: cảm biến         Trang 2: điều hướng        Trang 3: nhiệm vụ          Trang 4: hộp đen
|DRONE#37 STANDBY 1/4 |   |DRONE#37 STANDBY 2/4 |   |DRONE#37 STANDBY 3/4 |   |DRONE#37 STANDBY 4/4 |
|T 25.3C  H 61.2%     |   |HDG 237 SW           |   |HABITAT FOUND: 2x    |   |BLACK BOX: LOST DRONE|
|P 1008.5hPa          |   |LAB BEARING 012 N    |   |CONTACTS:      3x    |   |WAVES 3  SHOTS 12    |
|ALT   +3.2 m         |   |TURN  +135 deg       |   |ZOMBIES DOWN: 12     |   |CELLS CLEARED 28     |
|A  9.79 m/s2         |   |LAB SIGNAL -63 MID   |   |SHOTS FIRED:   6     |   |END: RESCUE TIMEOUT  |
|G  1.25 dps          |   |WIFI NETS 12 (8 s)   |   |FLIGHT 03:12         |   |EVT 05/18 t=41.2s    |
|M  45.3 uT           |   |WINGMAN #52 STANDBY  |   |                     |   |FIRE P5 (3,2) x7     |
|STATUS: ALL NORMAL   |   |CENTRE = SCAN WIFI   |   |1x REPORT 2x BATTLE  |   |<> EVENTS  ^v PAGES  |
```

**BATTLE:**
```
|WAVE 2   TARGETS  5  |
|[■][■][■][■][■][■][ ]|   ← thanh năng lượng lớn 2 dòng
|ENERGY  6/10 P5 BOOM |   ← SEL: P3 PEW SEL; cooldown: COOL3.2
|AIM (3,2)    TARGET! |
|DOWN 12   WAVES CLR 1|
|WINGMAN #52 STANDBY  |   ← SOS đồng đội > HIT!/MISSED!/AREA CLEARED! > OVERRUN!
|1x CHARGE 2x STANDBY |
```

**ATTACKED:** `!! UNDER ATTACK !!`, `DRONE #37  RESCUE IN`, số giây chữ lớn 2x, `DOUBLE-PRESS TO RESCUE`, `SOS BEACON ON`.
**TERMINATED:** `DRONE #37 LOST`, `COMMS OFFLINE`, `BLACK BOX SAVED`, `END: RESCUE TIMEOUT`.

---

## 17. Hộp đen (black box)

- Ghi **từng chuyến xuất kích**: bắt đầu khi vào BATTLE (xóa bản ghi cũ trong RAM và flash), tới khi về STANDBY hoặc tự hủy. Được cứu vẫn là cùng chuyến.
- **32 sự kiện gần nhất** (bộ đệm vòng): đợt bắt đầu/dọn xong, nạp, bắn, đổi công suất, `$CONTACT`, SOS nhận/gửi, cứu viện, bị lật, về STANDBY, tự hủy.
- **Bản tổng kết:** số đợt đã dọn, số phát bắn, số ô đã dọn, thời gian bay, lý do kết thúc (`RETURNED` / `RESCUE TIMEOUT`).
- Khi tự hủy: ghi **một lần** vào **trang flash cuối cùng** (có mã kiểm tra), và gửi bản tổng kết cho đồng đội qua `$BBOX`.
- `BLACKBOX` (chỉ ở STANDBY): in tổng kết rồi từng sự kiện. OLED trang 4 xem được trên màn hình.

---

## 18. UART và lệnh

**Board → PC:** tiền tố `[TLM]`, `[WARN]`, `[ATTACK]`, `[EVT]`, `[STAT]`, `[REPORT]`, `[BBOX]`, `[CMD]`. Thông báo đổi mode giữ đúng chữ của đề. Kết thúc `\r\n`.

**PC → Board:** gõ qua UART hoặc gửi từ Command Centre qua WiFi. Board tự echo trên UART (TeraTerm **tắt** Local echo).

| Lệnh | Tác dụng | Hợp lệ ở |
|---|---|---|
| `HELP`, `STATUS`, `THRESH` | Hướng dẫn, trạng thái, ngưỡng | Mọi lúc |
| `SET <T\|P\|H\|A\|G\|M>_<LOW\|HIGH> <v>` | Đổi ngưỡng | Mọi lúc |
| `MODE STANDBY\|BATTLE`, `CHARGE`, `RESCUE` | Như các thao tác nút | Theo mode |
| `POWER RULE\|1..5` | Chế độ bắn | BATTLE |
| `ASSIST` | Gửi cứu viện cho đồng đội đang SOS | STANDBY, BATTLE |
| `MUTE ON\|OFF`, `ID <n>`, `ROLE …`, `MOUNT …`, `WIFI <ssid> <pwd>`, `SERVER <ip> <port>` | Cấu hình, tự lưu flash | STANDBY |
| `HOME`, `CALIB`, `SCAN`, `BLACKBOX` | La bàn, độ cao, quét WiFi, hộp đen | STANDBY |
| `TRAINING ON\|OFF` | Chế độ huấn luyện | Mọi lúc |
| `I2CSCAN`, `I2CDUMP`, `INPUTS`, `WIFIDBG` | **Chỉ dùng lúc phát triển** (`ENABLE_DEV_TOOLS 0` trước khi nộp) | Mọi lúc |

---

## 19. Kiến trúc, thời gian, rủi ro

- Siêu vòng lặp không chặn, định thời bằng `HAL_GetTick()`. ISR chỉ ghi cờ hoặc dữ liệu (EXTI13 nút, EXTI11 IMU, USART1 RX/TX).
- **Lớp giao tiếp SPI cho module WiFi** (`es_wifi_io`) phải tự viết trong `main.c` vì chỉ nộp một file. Driver `es_wifi.c` đã có sẵn và đã được compile.
- **Lệnh WiFi chặn chương trình** (gửi khoảng 20–50 ms, quét 2–4 s, kết nối vài giây). Vì vậy: chỉ gửi gom mỗi 500 ms; kết nối lúc khởi động hoặc lúc thử lại; quét theo yêu cầu; không gọi WiFi trong lúc đang xác nhận lật hay đếm cứu hộ, ngoại trừ gửi `$SOS` mỗi 1 s. Nhấn nút vẫn không bị mất vì nút đọc bằng ngắt.
- Module ngoài hoặc WiFi không có thì bỏ qua, phần *(đề)* vẫn chạy đúng.

| Hạng mục | Giá trị |
|---|---|
| Cảm biến + TLM/WARN | 1000 ms |
| Debounce nút / cửa sổ double | 30–50 ms / < 500 ms |
| LED BATTLE / ATTACKED | đảo mỗi 500 / 250 ms |
| Giữ lật để xác nhận / cửa sổ cứu hộ | 300 ms / 10 s |
| 5-way: đọc / giữ để đổi công suất | 30 ms / 1 s |
| Lây nhiễm / nghỉ giữa hai đợt | 3 s → 1 s / 2 s |
| WiFi: gửi gom / kiểm tra nhận / heartbeat / thử kết nối lại | 500 ms / 200 ms / 1 s / 10 s |

**Thứ tự làm:**

| Giai đoạn | Nội dung |
|---|---|
| 1 | Phần *(đề)*: UART, cảm biến, ngưỡng, nút, state machine, laser, lật, cứu hộ, tự hủy |
| 2 | OLED, LED matrix, buzzer, 5-way; cấu hình lưu flash; console UART |
| 3 | Bản đồ + đợt + ngắm bắn + công suất; la bàn; hộp đen |
| 4 | WiFi: lớp SPI, kết nối, gửi/nhận với Command Centre; `command_centre.py` |
| 5 | Hai drone phối hợp (`$HB`, `$CONTACT`, `$SOS`, `$RESCUE`, `$BBOX`); quét WiFi, nơi ở, RSSI |

---

## 20. Nhật ký quyết định

| Ngày | Quyết định |
|---|---|
| 12/10 | Mặt chip hướng lên là bình thường (đổi được bằng `MOUNT`) |
| 12/10 | BATTLE không gửi telemetry định kỳ; chỉ nộp `main.c` |
| 13/10 | `[TLM]` đúng 6 trường; giữ lệnh UART `MODE`/`CHARGE`/`RESCUE` |
| 13/10 | Câu chuyện chung theo vòng đời nhiệm vụ; mỗi module gắn vào một giai đoạn |
| 13/10 | SOS khi ATTACKED, kêu cả khi im lặng; single press STANDBY = báo cáo trinh sát |
| 13/10 | La bàn về phòng lab; hộp đen theo từng chuyến; thoát TERMINATED bằng RESET |
| 13/10 | Bắn một phát mỗi lần; vùng nổ và âm thanh theo công suất; cooldown `p` giây (trừ `p = 1`) chỉ ở chế độ chọn công suất |
| 13/10 | Độ cao: mốc 0 m = `P0`, bậc 0.1 hPa; `HOME` đặt lại mốc |
| 17/10 | Chiến đấu theo đợt; OLED ưu tiên theo tình huống + chữ lớn; hộp đen có tổng kết và xem trên OLED; trang kiểm tra hệ thống |
| 20/10 | **Chỉ được dùng 4 module ngoài**: giữ OLED, LED matrix, buzzer, 5-way. Bỏ DIP, IR, light, sound |
| 20/10 | Số hiệu drone lấy từ UID của chip; cấu hình (`ID`, `ROLE`, `MOUNT`, `MUTE`, WiFi) bằng lệnh và lưu flash |
| 20/10 | Chế độ bắn: RULE mặc định; giữ giữa 5-way 1 s hoặc `POWER` để chọn công suất 1–5 có cooldown |
| 20/10 | Tìm nơi ở = T/H dễ chịu + quét thấy ≥ 3 mạng WiFi (thay light sensor); RSSI hotspot = khoảng cách tới phòng lab |
| 20/10 | Hai board = HUNTER + SCOUT, liên lạc qua WiFi qua Command Centre trên laptop (thay bản wingman nối dây). SCOUT báo va chạm zombie bằng gia tốc vượt ngưỡng |
| 20/10 | Tự hủy: gửi `$BBOX`, ngắt WiFi, giữ module WiFi ở reset, rồi tắt UART |
