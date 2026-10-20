# EE2028 A2 — The Last of EE2028: Spec v3.2

Tài liệu này là "hợp đồng" của nhóm: code viết đúng theo đây, muốn đổi hành vi thì sửa file này trước.
Firmware nằm trong **một file `Core/Src/main.c`** (cả drone lẫn tay cầm, chọn bằng cấu hình `ROLE`). Chương trình laptop nằm trong `tools/dashboard.py`. Lịch sử quyết định ở mục 18.

## Tổng quan

| Thành phần | Là gì | Phần cứng |
|---|---|---|
| **Drone** (board 1) | Toàn bộ yêu cầu của đề + tính năng thêm | **LED matrix, buzzer, 5-way switch, IR sensor** (đúng 4 module ngoài) |
| **Tay cầm phi công** (board 2) | Nghiêng để ngắm, nút để khóa mục tiêu / đổi công suất / gọi cứu hộ | Không module ngoài, chỉ dùng accelerometer, nút và LED trên board |
| **Command Centre** (laptop) | Dashboard Python **thay cho OLED**: hiển thị mọi thứ của drone, chuyển lệnh từ tay cầm sang drone | Đọc drone qua USB (UART), sau này qua WiFi |

**Câu chuyện chung:** drone đi qua đúng vòng đời mà đề cho, mỗi module gắn vào một giai đoạn:
**trinh sát** ở STANDBY (6 cảm biến, la bàn về phòng lab, tìm nơi ở cho người, IR tránh vật cản) → **chiến đấu** ở BATTLE (bản đồ lây nhiễm trên LED matrix, ngắm bằng 5-way hoặc tay cầm, IR phát hiện zombie áp sát) → **bị hạ** ở The Last of EE2028 (SOS, chờ cứu) → **được cứu** hoặc **tự hủy** (cắt liên lạc, hộp đen được giữ lại để điều tra).

Ký hiệu *(đề)* = yêu cầu bắt buộc của đề bài. **Drone đứng một mình (không laptop, không tay cầm, không module ngoài) vẫn phải chạy đúng toàn bộ phần *(đề)*.**

---

## 1. Phần cứng

| Thiết bị | Kết nối | Vai trò |
|---|---|---|
| Board ×2 | B-L4S5I-IOT01A (STM32L4S5VI), clock 4 MHz MSI mặc định | Drone + tay cầm |
| LED2 (xanh lá) | PB14 | Báo mode *(đề)* |
| User button B1 | PC13, EXTI13, cả hai cạnh | Nút chính *(đề)* |
| HTS221 (T, H), LPS22HB (P), LSM6DSL (A, G), LIS3MDL (M) | I2C2 nội bộ qua BSP | 6 cảm biến *(đề)*, la bàn, phát hiện lật; trên tay cầm: đo nghiêng |
| LSM6DSL INT1 | PD11, EXTI11 | Ngắt 6D khi đổi hướng |
| UART tới PC | USART1 PB6/PB7 (ST-LINK VCP), 115200 8N1 | Command Centre *(đề)* |
| Red LED Matrix 8x8 (HT16K33) | Cổng Grove I2C, addr 0x70 | Bản đồ / la bàn / biểu tượng |
| Buzzer | Cổng Grove **D6** (PB1, TIM3_CH4 PWM) | Âm thanh |
| 5-way switch | Cổng Grove I2C, addr 0x03 | Ngắm bắn, đổi công suất, đăng ký lại phòng lab |
| IR reflective sensor | Cổng Grove **A0** (PC5, input số) | Vật cản / zombie áp sát |
| WiFi ISM43362 (trên board) | SPI3 | **Giai đoạn cuối, tùy chọn:** gửi dữ liệu lên dashboard không dây |

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
| **Số hiệu drone** | Một chữ số 1–9, tính từ **UID 96-bit** của chip | `ID <1..9>` |
| **Kiểu lắp** | Mặt chip hướng lên là bình thường | `MOUNT CHIPUP\|ZDOWN` (`ZDOWN` = Z+ hướng xuống đất, đúng chữ đề) |
| **Im lặng** | Tắt | `MUTE ON\|OFF` (SOS vẫn kêu) |
| **WiFi** (giai đoạn cuối) | Chưa cấu hình | `WIFI <ssid> <password>`, `SERVER <ip> <port>` |

- Lưu vào **trang flash áp chót** (có mã kiểm tra), reset hay mất điện không mất. Mỗi lệnh tự lưu, trả `[CMD] OK … (saved)`.
- Chỉ đổi được ở **STANDBY**.

---

## 5. Khởi động (drone)

| Bước | Làm gì | LED matrix / buzzer | UART |
|---|---|---|---|
| 1 | Đọc cấu hình, tính số hiệu | | `=== Drone #3 online === mount: chip-up, silent: OFF` |
| 2 | Kiểm tra module | | `[EVT] System check: sensors OK, LED matrix OK, 5-way OK, IR OK` |
| 3 | Lưu `P0` (trung bình 10 lần đọc) làm mốc độ cao 0 m | | `[EVT] Altitude zero at 1008.52 hPa` |
| 4 | Lưu hướng hiện tại làm **hướng về phòng lab** | | `[EVT] Lab bearing registered: 237 deg` |
| 5 | Nếu flash còn hộp đen của lần bay trước | | `[EVT] Black box of Drone #3 recovered (18 records). Type BLACKBOX to read` |
| 6 | Vào STANDBY | Số hiệu 1 s, tiếng khởi động | `Entering STANDBY MODE` *(đề)* |

---

## 6. STANDBY: trinh sát

| Thành phần | Làm gì | LED matrix / buzzer | UART |
|---|---|---|---|
| 6 cảm biến *(đề)* | Đọc mỗi 1 s, so ngưỡng (mục 11) | | `[TLM] T: … M: …` **đúng 6 trường**, hoặc `[WARN] …` *(đề)* |
| LED2 *(đề)* | Sáng liên tục | | |
| La bàn (mục 12) | Hướng có bù nghiêng | Mũi tên chỉ về phòng lab | |
| **Tìm nơi ở** | Nhiệt độ 20–32 °C **và** độ ẩm 40–70 %RH liên tục 3 s (đề: humidity "used to find new human habitats") | Hình ngôi nhà 2 s, chuông | `[EVT] Possible human habitat found: T 26.1 C, H 58 %RH` / `[EVT] Habitat conditions lost` |
| **IR** | Vật ở gần: **tránh va chạm** | Dấu `!` 1 s, bíp | `[EVT] Obstacle at close range, holding position` |
| 5-way giữa, **giữ 1 s** | **Đăng ký lại phòng lab**: hướng hiện tại + mốc độ cao 0 m (như lệnh `HOME`) | Hai tiếng bíp | `[EVT] Lab re-registered: bearing 012 deg, altitude zero` |
| PB single | **Báo cáo trinh sát** | Bíp | `[REPORT] Drone #3: alt +2.1 m, hdg 237, habitat found 2x, zombies down 12, shots 6` |

- Các hướng của 5-way không dùng ở STANDBY. Lật board ở STANDBY không có tác dụng.
- Khoảng tìm nơi ở nới rộng hơn chuẩn tiện nghi ASHRAE (khoảng 30–60 %RH) để demo được trong phòng lab có máy lạnh ở Singapore.
- Sự kiện IR có thời gian chờ 3 s để không lặp.

---

## 7. BATTLE: chiến đấu

| Thành phần | Làm gì | LED matrix / buzzer | UART |
|---|---|---|---|
| Khi vào | Bắt đầu **đợt 1** trên bản đồ mới (mục 8) | | `Entering BATTLE MODE` *(đề)* + `[STAT]` + `[EVT] Wave 1 incoming: 3 outbreaks` |
| LED2 *(đề)* | Nháy 1 Hz | | |
| PB single *(đề)* | Nạp +3/10, đủ năng lượng thì tự bắn (mục 9) | Bíp nạp; tia + vùng nổ, piuuu / pew / BOOM | `[EVT] LASER charged: 3/10 -> 6/10` |
| 5-way 4 hướng / tay cầm nghiêng | Dời tâm ngắm 1 ô, giữ thì chạy liên tục | Tâm ngắm nháy 4 Hz | |
| 5-way giữa ngắn / tay cầm nhấn 1 lần | Nhảy tới ô nhiễm gần nhất | | `[EVT] Target locked at (3,2)` |
| 5-way giữa **giữ 1 s** / tay cầm nhấn đúp | Đổi công suất: RULE 5/10 → 1 → 2 → 3 → 4 → 5 → RULE | | `[EVT] Weapon: power 3/10, cooldown 3 s` |
| **IR** | **Zombie áp sát**: ô ngay trước drone (hàng 7, cột 3–4) bị nhiễm và nhấp nháy | Báo động | `[EVT] ZOMBIE AT CLOSE RANGE!` |
| Accel *(đề)* | Phát hiện lật theo kiểu lắp (mục 10) | | → ATTACKED |
| PB double *(đề)* | Về STANDBY, **bản đồ bị xóa** | | `Entering STANDBY MODE` |

BATTLE không gửi telemetry định kỳ, chỉ gửi sự kiện.

---

## 8. Bản đồ lây nhiễm và các đợt

- 8x8 là khu vực phía trước drone nhìn từ trên xuống. Drone ở giữa cạnh dưới. Hàng 0 xa nhất, hàng 7 gần nhất. Ô sáng = khu bị zombie chiếm.
- Đợt *n* rải **2 + n** ổ nhiễm (tối đa 12) ở hàng 0–4. Trong đợt, mỗi khoảng thời gian một ô nhiễm lây sang một ô kề chưa nhiễm. Khoảng này bắt đầu 3 s ở đợt 1, mỗi đợt nhanh hơn 0.25 s, nhanh nhất 1 s.
- **Dọn sạch bản đồ** → `[EVT] Wave 2 cleared! AREA CLEARED`, tiếng chiến thắng, matrix hiện dấu tích. 2 s sau đợt kế tiếp bắt đầu. Trong lúc nghỉ, IR không tạo ổ mới.
- Thua một đợt không dẫn tới ATTACKED. Theo đề, drone chỉ bị hạ khi bị lật.
- **Quá 75 % (≥ 48/64 ô):** chỉ báo `[EVT] AREA OVERRUN (52/64)`.
- Về STANDBY thì bản đồ bị xóa, vào lại BATTLE thì bắt đầu từ đợt 1. Bị tấn công thì bản đồ dừng lây. Được cứu thì tiếp tục đúng đợt đó.
- Không có 5-way và tay cầm: tâm ngắm tự nhảy tới ô nhiễm gần nhất trước mỗi phát bắn.

---

## 9. Laser gun

- Năng lượng theo đơn vị **1/10**, số nguyên 0–10. Mỗi single press **+3/10** *(đề)*.
- **Chế độ bắn:**
  - **RULE** (mặc định, đúng đề): mỗi phát `p = 5`, đủ năng lượng là tự bắn ngay, **không cooldown** *(đề)*.
  - **SEL** (5-way giữa giữ 1 s, tay cầm nhấn đúp, hoặc lệnh `POWER 1..5`; `POWER RULE` để về đúng đề): `p` = 1–5. Sau mỗi phát có **cooldown `p` giây**, riêng `p = 1` không có. Trong lúc cooldown vẫn nạp được. Hết cooldown mà đủ năng lượng thì tự bắn ngay.
- Tự bắn khi năng lượng ≥ `p`, mỗi lần **một phát**, trừ `p`. Năng lượng thừa để dành, tối đa 10/10.

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
- **Độ cao** (dashboard, báo cáo): mốc 0 m là `P0` lúc khởi động, `h = 44330 × (1 − (P/P0)^0.1903)` m. Lọc trung bình trượt rồi làm tròn theo bậc 0.1 hPa (≈ 0.85 m) tính từ P0, có trễ 0.06 hPa để không nhảy. Dòng `[TLM]` vẫn gửi áp suất thật.

---

## 12. La bàn về phòng lab

- Heading = `atan2` của từ trường ngang, **bù nghiêng** bằng accelerometer, 0–359°, theo kiểu lắp.
- **Hướng về phòng lab** = hướng lúc khởi động. Đăng ký lại bằng 5-way giữa giữ 1 s ở STANDBY hoặc lệnh `HOME` (cùng lúc đặt lại mốc độ cao).
- Matrix ở STANDBY: mũi tên 8 hướng chỉ `lab_bearing − heading`. Dashboard: kim la bàn.
- `CALIB`: xoay board nằm ngang 1–2 vòng trong 10 s, lấy min/max trục X, Y làm offset nhiễu từ cứng.

---

## 13. ATTACKED và TERMINATED *(đề)*

**ATTACKED:**
- LED2 nháy 2 Hz *(đề)*. Khi vào: `[EVT] Drone #3 flipped! The Last of EE2028`. Mỗi 1 s: `[ATTACK] Drone #3 Was Attacked! Rescue in 7 s` *(đề)*.
- Buzzer phát **Morse SOS** liên tục, kêu cả khi im lặng. Matrix hiện chữ S-O-S theo nhịp.
- **Được cứu** khi: double press trên drone *(đề)*, lệnh `RESCUE`, hoặc **nhấn đúp trên tay cầm** ("phòng lab cử đội cứu hộ"). Báo `[EVT] Drone #3 rescued!`, về BATTLE. Matrix mặt cười 1 s, ba nốt đi lên.
- Single press, 5-way, IR bị bỏ qua.

**TERMINATED** (hết 10 s không ai cứu):
1. Ghi hộp đen vào flash (mục 14).
2. `[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.`
3. Matrix dấu X, giai điệu kết thúc. Tắt LED2, tắt UART (và WiFi nếu có, giữ module ở trạng thái reset), tắt các ngắt, MCU vào Standby mode.
4. Chỉ nút **RESET** đen (hoặc rút nguồn) mới thoát được.
5. **Chế độ huấn luyện** (`TRAINING ON`): thay vì cắt liên lạc, báo `[EVT] TRAINING: drone #3 would self-destruct now. Restarting in 3 s` rồi tự khởi động lại. Hộp đen vẫn được ghi.

---

## 14. Hộp đen (black box)

- Ghi **từng chuyến xuất kích**: bắt đầu khi vào BATTLE (xóa bản ghi cũ trong RAM và flash), tới khi về STANDBY hoặc tự hủy. Được cứu vẫn là cùng chuyến.
- **32 sự kiện gần nhất** (bộ đệm vòng): đợt bắt đầu/dọn xong, nạp, bắn, đổi công suất, zombie áp sát, quá 75 %, bị lật, được cứu, về STANDBY, tự hủy.
- **Bản tổng kết:** số đợt đã dọn, số phát bắn, số ô đã dọn, thời gian bay, lý do kết thúc (`RETURNED` / `RESCUE TIMEOUT`).
- Khi tự hủy: ghi **một lần** vào **trang flash cuối cùng** (có mã kiểm tra), sống sót qua RESET và mất điện.
- `BLACKBOX` (chỉ ở STANDBY): in tổng kết rồi từng sự kiện. Dashboard có tab xem dạng dòng thời gian.

---

## 15. Tay cầm phi công (board 2, `ROLE CONTROLLER`)

- Không cắm module ngoài. Cắm USB vào laptop. Dashboard đọc tay cầm và chuyển thành **lệnh UART bình thường** gửi xuống drone, nên drone không cần biết tay cầm tồn tại.
- Khởi động: `=== Pilot controller online ===`.

| Thao tác trên tay cầm | Tay cầm gửi | Dashboard gửi xuống drone |
|---|---|---|
| Nghiêng > 20° theo một hướng (giữ thì lặp mỗi 150 ms) | `[CTL] TILT <dx> <dy>` | `AIM <dx> <dy>` (chỉ khi drone ở BATTLE) |
| Nhấn 1 lần | `[CTL] BTN SINGLE` | BATTLE: `LOCK` |
| Nhấn đúp | `[CTL] BTN DOUBLE` | BATTLE: `POWER NEXT`. ATTACKED: `RESCUE` (đội cứu hộ của phòng lab) |

- **LED2 của tay cầm** báo mode của drone (dashboard gửi `LINK <mode>` mỗi 250 ms): sáng liên tục = STANDBY, nháy 1 Hz = BATTLE, nháy 2 Hz = ATTACKED, tắt = mất kết nối với drone quá 2 s.
- Nút của tay cầm dùng chung bộ debounce và phân biệt single/double như drone.
- Không có tay cầm thì drone vẫn đủ chức năng (5-way hoặc tự ngắm).

---

## 16. UART, lệnh và dữ liệu cho dashboard

**Drone → PC:** tiền tố `[TLM]`, `[WARN]`, `[ATTACK]`, `[EVT]`, `[STAT]`, `[REPORT]`, `[BBOX]`, `[CMD]`. Thông báo đổi mode giữ đúng chữ của đề. Kết thúc `\r\n`.

**Dòng dữ liệu cho dashboard** (chỉ gửi sau khi dashboard bật bằng `DASH ON`, để TeraTerm vẫn gọn):

| Dòng | Khi nào | Nội dung |
|---|---|---|
| `[HUD] MODE BATTLE E 6 P 5 SEL 0 CD 0.0 WAVE 2 TGT 5 DOWN 12 HDG 237 LAB 12 ALT 3.2 RESC 0 IR 0` | Mỗi 250 ms | Trạng thái đầy đủ để vẽ dashboard |
| `[MAP] 00183C0000000000 AIM 3 2` | Khi bản đồ hoặc tâm ngắm đổi, tối đa 5 lần/s | 8 hàng dạng hex + vị trí tâm ngắm |

**PC → Drone:** gõ trên TeraTerm hoặc dashboard gửi. Drone tự echo (TeraTerm **tắt** Local echo). Backspace xóa, ESC hủy dòng.

| Lệnh | Tác dụng | Hợp lệ ở |
|---|---|---|
| `HELP`, `STATUS`, `THRESH` | Hướng dẫn, trạng thái, ngưỡng | Mọi lúc |
| `SET <T\|P\|H\|A\|G\|M>_<LOW\|HIGH> <v>` | Đổi ngưỡng | Mọi lúc |
| `MODE STANDBY\|BATTLE`, `CHARGE`, `RESCUE` | Như các thao tác nút | Theo mode |
| `AIM <dx> <dy>`, `LOCK`, `POWER RULE\|1..5\|NEXT` | Ngắm, khóa mục tiêu, chế độ bắn | BATTLE |
| `MUTE ON\|OFF`, `ID <1..9>`, `MOUNT …`, `ROLE …`, `WIFI …`, `SERVER …` | Cấu hình, tự lưu flash | STANDBY |
| `HOME`, `CALIB`, `BLACKBOX` | Đăng ký lab + mốc độ cao, hiệu chuẩn la bàn, hộp đen | STANDBY |
| `TRAINING ON\|OFF`, `DASH ON\|OFF` | Chế độ huấn luyện, dòng dữ liệu dashboard | Mọi lúc |
| `I2CSCAN`, `I2CDUMP`, `INPUTS` | **Chỉ dùng lúc phát triển** (`ENABLE_DEV_TOOLS 0` trước khi nộp) | Mọi lúc |

---

## 17. Dashboard Command Centre (`tools/dashboard.py`)

Python + Tkinter + pyserial, chạy bằng Python trên Windows (cổng COM nằm phía Windows).

| Khu vực | Nội dung |
|---|---|
| Thanh trên | Số hiệu drone, **MODE** chữ lớn đổi màu (xanh STANDBY, cam BATTLE, đỏ ATTACKED, xám TERMINATED), đèn mô phỏng LED2 nháy đúng nhịp, trạng thái kết nối drone / tay cầm |
| STANDBY | 6 ô cảm biến kèm đơn vị, ô nào vượt ngưỡng tô đỏ; đồ thị 60 s gần nhất; kim la bàn và mũi tên về lab; độ cao; trạng thái tìm nơi ở; thống kê nhiệm vụ |
| BATTLE | Bản đồ 8x8 cỡ lớn kèm tâm ngắm; thanh năng lượng; công suất / cooldown; đợt, số mục tiêu, số zombie đã hạ |
| ATTACKED | Cả màn hình đỏ, **đếm ngược cứu hộ cỡ rất lớn**, `DOUBLE-PRESS TO RESCUE` |
| TERMINATED | `DRONE #3 LOST — COMMS OFFLINE` khi nhận tin cuối rồi mất tín hiệu |
| Tab hộp đen | Gửi `BLACKBOX`, hiện tổng kết và dòng thời gian sự kiện |
| Dưới cùng | Nhật ký sự kiện, ô gõ lệnh gửi xuống drone |

- Khi kết nối: gửi `DASH ON`. Khi đóng: `DASH OFF`.
- Chọn cổng COM của drone và của tay cầm trong giao diện. Tay cầm là tùy chọn.

---

## 18. Kiến trúc, thời gian, thứ tự làm, nhật ký

- Siêu vòng lặp không chặn, định thời bằng `HAL_GetTick()`. ISR chỉ ghi cờ hoặc dữ liệu (EXTI13 nút, EXTI11 IMU, USART1 RX/TX).
- Module ngoài, tay cầm hay laptop không có thì bỏ qua, phần *(đề)* vẫn chạy đúng.
- **WiFi (giai đoạn 5, tùy chọn):** drone là TCP client tới dashboard trên laptop qua hotspot (điện thoại hoặc hotspot của laptop, 2.4 GHz, WPA2). Gửi đúng các dòng như UART, gom mỗi 500 ms. Kết nối lúc khởi động, mất thì chỉ thử lại ở STANDBY mỗi 30 s, vì lệnh WiFi chặn chương trình. UART luôn chạy song song.

| Hạng mục | Giá trị |
|---|---|
| Cảm biến + TLM/WARN | 1000 ms |
| Debounce nút / cửa sổ double | 30–50 ms / < 500 ms |
| LED BATTLE / ATTACKED | đảo mỗi 500 / 250 ms |
| Giữ lật để xác nhận / cửa sổ cứu hộ | 300 ms / 10 s |
| 5-way: đọc / giữ | 30 ms / 1 s |
| IR: đọc / số mẫu ổn định / chờ giữa sự kiện | 20 ms / 3 / 3 s |
| Lây nhiễm / nghỉ giữa hai đợt | 3 s → 1 s / 2 s |
| `[HUD]` / `[MAP]` | 250 ms / tối đa 200 ms |
| Tay cầm: lặp nghiêng / `LINK` | 150 ms / 250 ms |

| Giai đoạn | Nội dung |
|---|---|
| 1 | Phần *(đề)*: UART, cảm biến, ngưỡng, nút, state machine, laser, lật, cứu hộ, tự hủy |
| 2 | LED matrix, buzzer, 5-way, IR; console UART; cấu hình lưu flash |
| 3 | Bản đồ + đợt + ngắm + công suất; la bàn; tìm nơi ở; hộp đen; `[HUD]`/`[MAP]` |
| 4 | Dashboard qua USB; tay cầm (firmware `ROLE CONTROLLER` + phần chuyển lệnh trong dashboard) |
| 5 | WiFi (tùy chọn) |

**Nhật ký quyết định:**

| Ngày | Quyết định |
|---|---|
| 12/10 | Mặt chip hướng lên là bình thường (đổi được bằng `MOUNT`); BATTLE không gửi telemetry định kỳ; chỉ nộp `main.c` |
| 13/10 | `[TLM]` đúng 6 trường; câu chuyện theo vòng đời nhiệm vụ; SOS khi ATTACKED, kêu cả khi im lặng; single press STANDBY = báo cáo trinh sát |
| 13/10 | La bàn về phòng lab; hộp đen theo từng chuyến; thoát TERMINATED bằng RESET; độ cao mốc `P0`, bậc 0.1 hPa |
| 13/10 | Bắn một phát mỗi lần; vùng nổ và âm thanh theo công suất; cooldown `p` giây (trừ `p = 1`) chỉ ở chế độ chọn công suất |
| 17/10 | Chiến đấu theo đợt; hộp đen có tổng kết |
| 20/10 | Chỉ 4 module ngoài. Bỏ WiFi hai chiều và phần hai drone phối hợp vì quá rối |
| 20/10 | **Dashboard Python trên laptop thay OLED**; UART trước, WiFi là giai đoạn cuối tùy chọn |
| 20/10 | Bộ module: **LED matrix, buzzer, 5-way, IR**. Giữ buzzer trên board để drone tự kêu SOS |
| 20/10 | **Board 2 = tay cầm phi công** qua USB, dashboard chuyển thành lệnh UART; giữ 5-way để drone tự đủ chức năng |
| 20/10 | Số hiệu drone một chữ số 1–9 từ UID; cấu hình bằng lệnh, lưu flash; 5-way giữa giữ 1 s ở STANDBY = đăng ký lại phòng lab |
