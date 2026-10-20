/* Host simulation of samplecode.c (spec v3.2): fake clock, sensors, button,
 * UART, Grove 5-way over I2C, IR pin and two flash pages. Logic tests only. */
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <stdlib.h>
#define main fw_main
#include "proj/Core/Src/main.c"   /* = Core/Src/samplecode.c, copied by run_tests.sh */
#undef main

/* ---------------- fake hardware ---------------- */
GPIO_TypeDef sim_gpiob, sim_gpioc, sim_gpiod;
PERIPH sim_i2c1, sim_usart1, sim_tim3;
ADC_TypeDef sim_adc1; ADC_Common_TypeDef sim_adc_common; RCC_TypeDef sim_rcc;
FLASH_TypeDef sim_flash_regs = { FLASH_OPTR_DBANK };
uint8_t sim_flash_page[8192];

static uint32_t tick = 0;
static int btn_level = 1, ir_pin = 1;
static float s_temp = 25, s_hum = 60, s_press = 1008;
static int16_t s_acc[3] = {0, 0, 1000};
static float s_gyro[3] = {1000, 0, 0};
static int16_t s_mag[3] = {300, 200, -200};
static int led = 0, led_toggles = 0;
static char out[400000]; static size_t outn = 0;
static uint8_t *tx_ptr = NULL; static uint16_t tx_n = 0;
static uint8_t *rx_ptr = NULL;
static jmp_buf jmp_end; static int standby_entered = 0, reset_called = 0;
static uint8_t sim_fw  = 0x00;      /* 5-way pressed bits */
static int sim_fiveway_present = 1;
static int sim_stuck_next_read = 0;   /* next 5-way read times out and leaves SDA low */
static uint8_t last_cmd[128];
static int erase_count[2];          /* [0] page 254, [1] page 255 */

static int sim_ir_present = 1, sim_pc5_pull_up = 1;
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { if (p == GPIOC && (g->Pin & GPIO_PIN_5)) sim_pc5_pull_up = g->Pull == GPIO_PULLUP; }
int sim_i2c_busy = 0; static int sim_recover_pulses = 0;
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState st) { if (p == GPIOB && pin == GPIO_PIN_8 && st == GPIO_PIN_SET) { sim_recover_pulses++; sim_i2c_busy = 0; } }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { if (p == GPIOB) return sim_i2c_busy && pin == GPIO_PIN_9 ? GPIO_PIN_RESET : GPIO_PIN_SET; if (pin == GPIO_PIN_5) { if (!sim_ir_present) return sim_pc5_pull_up ? GPIO_PIN_SET : GPIO_PIN_RESET; return ir_pin ? GPIO_PIN_SET : GPIO_PIN_RESET; } return btn_level ? GPIO_PIN_SET : GPIO_PIN_RESET; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t a, uint32_t t, uint32_t to) { (void)h;(void)t;(void)to; a >>= 1; if (sim_i2c_busy) return HAL_ERROR; return ((a == 0x03 && sim_fiveway_present) || a == 0x70) ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to) { (void)h;(void)to; a >>= 1; if (sim_i2c_busy) return HAL_ERROR; if (a == 0x03 && !sim_fiveway_present) return HAL_ERROR; if (n >= 1) last_cmd[a & 127] = d[0]; return (a == 0x03 || a == 0x70) ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to)
{
    (void)h;(void)to; a >>= 1;
    if (sim_i2c_busy) return HAL_ERROR;
    if (sim_stuck_next_read && a == 0x03) { sim_stuck_next_read = 0; sim_i2c_busy = 1; return HAL_ERROR; }
    if (a == 0x03 && sim_fiveway_present) {
        if (last_cmd[a] == 0x00) { d[0] = 0x02; d[1] = 0x00; d[2] = 0x86; d[3] = 0x28;   /* real module: (VID<<16)|PID, little-endian */ return HAL_OK; }
        memset(d, 0, n);
        for (int i = 0; i < 5 && 4 + i < n; i++) d[4 + i] = (sim_fw & (1u << i)) ? 0x00 : 0x01;   /* active = raw 0 */
        return HAL_OK;
    }
    return HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t a, uint16_t mem, uint16_t ms, uint8_t *d, uint16_t n, uint32_t to)
{ (void)ms; uint8_t c = (uint8_t)mem; if (HAL_I2C_Master_Transmit(h, a, &c, 1, to) != HAL_OK) return HAL_ERROR; return HAL_I2C_Master_Receive(h, a, d, n, to); }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) { (void)h; tx_ptr = d; tx_n = n; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) { (void)h;(void)n; rx_ptr = d; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *h) { (void)h; }
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *o, uint32_t c) { (void)h;(void)o;(void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h;(void)c; return HAL_OK; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 4000000; }
void HAL_NVIC_SetPriority(IRQn_Type i, uint32_t a, uint32_t b) { (void)i;(void)a;(void)b; }
void HAL_NVIC_EnableIRQ(IRQn_Type i) { (void)i; }
void HAL_NVIC_DisableIRQ(IRQn_Type i) { (void)i; }
void HAL_PWR_DisableWakeUpPin(uint32_t p) { (void)p; }
void HAL_PWR_EnterSTANDBYMode(void) { standby_entered = 1; longjmp(jmp_end, 1); }
void NVIC_SystemReset(void) { reset_called = 1; longjmp(jmp_end, 2); }
void HAL_SuspendTick(void) {}
HAL_StatusTypeDef HAL_Init(void) { return HAL_OK; }
void HAL_Delay(uint32_t ms) { (void)ms; }
void BSP_LED_Init(Led_TypeDef l) { (void)l; }
void BSP_LED_On(Led_TypeDef l)  { (void)l; if (!led) led_toggles++; led = 1; }
void BSP_LED_Off(Led_TypeDef l) { (void)l; if (led) led_toggles++; led = 0; }
uint32_t BSP_TSENSOR_Init(void){return 0;} uint32_t BSP_HSENSOR_Init(void){return 0;} uint32_t BSP_PSENSOR_Init(void){return 0;}
int BSP_ACCELERO_Init(void){return 0;} uint8_t BSP_GYRO_Init(void){return 0;} int BSP_MAGNETO_Init(void){return 0;}
float BSP_TSENSOR_ReadTemp(void) { return s_temp; }
float BSP_HSENSOR_ReadHumidity(void) { return s_hum; }
float BSP_PSENSOR_ReadPressure(void) { return s_press; }
void BSP_ACCELERO_AccGetXYZ(int16_t *p) { memcpy(p, s_acc, sizeof s_acc); }
void BSP_GYRO_GetXYZ(float *p) { memcpy(p, s_gyro, sizeof s_gyro); }
void BSP_MAGNETO_GetXYZ(int16_t *p) { memcpy(p, s_mag, sizeof s_mag); }
void SENSOR_IO_Write(uint8_t a, uint8_t r, uint8_t v) { (void)a;(void)r;(void)v; }
uint8_t SENSOR_IO_Read(uint8_t a, uint8_t r) { (void)a;(void)r; return 0; }
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
#ifdef SIM_RT
static void rt_flash_save(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void) { rt_flash_save(); return HAL_OK; }
#else
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }
#endif
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err)
{
    (void)err;
    if (e->Banks != FLASH_BANK_2 || (e->Page != 254 && e->Page != 255)) return HAL_ERROR;
    int k = (int)e->Page - 254;
    erase_count[k]++;
    memset(sim_flash_page + 4096 * k, 0xFF, 4096);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data)
{
    (void)type;
    uint32_t off = addr - (uint32_t)(uintptr_t)sim_flash_page;
    if (off + 8 > sizeof sim_flash_page) return HAL_ERROR;
    memcpy(sim_flash_page + off, &data, 8);
    return HAL_OK;
}

static int auto_tick = 0, free_tick = 0;
static void service_isrs(void);
#ifdef SIM_RT
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
static int rt_fd = -1;
static const char *rt_flash_file = NULL;
static uint32_t rt_ms(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); static long long t0 = -1; long long ms = ts.tv_sec * 1000LL + ts.tv_nsec / 1000000; if (t0 < 0) t0 = ms; return (uint32_t)(ms - t0); }
uint32_t HAL_GetTick(void) { tick = rt_ms(); service_isrs(); return tick; }
#else
uint32_t HAL_GetTick(void) { if (free_tick || (auto_tick && state == ST_TERMINATED)) { tick++; service_isrs(); } return tick; }
#endif

/* ---------------- sim helpers ---------------- */
static int ctl_mode = 0;
static void service_isrs(void)
{
    while (tx_n) {
#ifdef SIM_RT
        if (rt_fd >= 0) { ssize_t w = write(rt_fd, tx_ptr, tx_n); (void)w; }
#else
        memcpy(out + outn, tx_ptr, tx_n); outn += tx_n; out[outn] = 0;
#endif
        tx_n = 0;
        HAL_UART_TxCpltCallback(&huart1);
    }
}
static void run_ms(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { tick++; if (ctl_mode) ctl_loop(); else app_loop(); service_isrs(); } }
static void btn_edge(int level) { btn_level = level; HAL_GPIO_EXTI_Callback(BUTTON_EXTI13_Pin); }
static void press(uint32_t hold_ms) { btn_edge(0); run_ms(hold_ms); btn_edge(1); }
static void single(void) { run_ms(80); press(60); run_ms(700); }
static void dbl(void) { run_ms(80); press(60); run_ms(150); press(60); run_ms(20); }
static void type(const char *s) { for (; *s; s++) { *rx_ptr = (uint8_t)*s; HAL_UART_RxCpltCallback(&huart1); } run_ms(5); }
static void fw_tap(int idx) { sim_fw = (uint8_t)(1u << idx); run_ms(60); sim_fw = 0; run_ms(60); }
static void fw_hold(int idx, uint32_t ms) { sim_fw = (uint8_t)(1u << idx); run_ms(ms); sim_fw = 0; run_ms(60); }
static void clear_out(void) { outn = 0; out[0] = 0; }
static int count(const char *needle) { int c = 0; const char *p = out; while ((p = strstr(p, needle))) { c++; p++; } return c; }
static const char *last(const char *needle) { const char *p = out, *r = ""; while ((p = strstr(p, needle))) { r = p; p++; } return r; }

static int fails = 0;
#define CHECK(cond, msg) do { if (cond) printf("  PASS  %s\n", msg); else { printf("  FAIL  %s\n", msg); fails++; } } while (0)

#ifndef SIM_RT
int main(void)
{
    memset(sim_flash_page, 0xFF, sizeof sim_flash_page);

    printf("== boot without IR ==\n");
    sim_ir_present = 0;
    cfg_load();
    app_init(); service_isrs();
    CHECK(cfg.drone_id == 3 && cfg.role == ROLE_DRONE, "blank flash -> default Drone #3, role DRONE");
    CHECK(strstr(out, "=== Drone #3 online === mount: chip-up, silent: OFF") != NULL, "boot banner");
    CHECK(strstr(out, "[EVT] System check: sensors OK, LED matrix OK, 5-way OK, IR NOT FOUND") != NULL, "system check: empty A0 -> IR NOT FOUND");
    CHECK(fiveway_addr == 0x03, "5-way detected by device ID");
    CHECK(strstr(out, "[EVT] Altitude zero at 1008.00 hPa") && strstr(out, "[EVT] Lab bearing registered:"), "altitude zero + lab bearing at boot");
    CHECK(strstr(out, "Entering STANDBY MODE") != NULL, "Entering STANDBY MODE");
    run_ms(5);
    CHECK(memcmp(mx_shown, GL_DIGIT[3], 8) == 0, "matrix shows drone number 3 at boot");
    CHECK(strstr(out, "[TLM] T: 25.00 C, P: 1008.00 hPa, H: 60.00 %RH, A: 9.81 m/s^2, G: 1.00 dps, M: 41.23 uT\r\n") != NULL, "TLM has exactly 6 fields");
    CHECK(strstr(out, "[HUD]") == NULL && strstr(out, "[MAP]") == NULL, "no dashboard lines before DASH ON");

    printf("== STANDBY scouting ==\n");
    ir_pin = 0; run_ms(200); ir_pin = 1;
    CHECK(strstr(out, "Obstacle") == NULL, "no IR events without the module");
    sim_ir_present = 1;
    clear_out(); run_ms(3100);
    CHECK(strstr(out, "[EVT] Possible human habitat found: T 25.0 C, H 60 %RH") != NULL && habitat_count == 1, "habitat after 3 s of T 20-32 C and H 40-70 %RH");
    CHECK(ir_ok && strstr(out, "[EVT] IR sensor connected"), "IR plugged in later is found in STANDBY (re-probe every 2 s)");
    run_ms(200);
    CHECK(memcmp(mx_shown, GL_HOUSE, 8) == 0, "matrix shows the house icon");
    clear_out(); s_hum = 85; run_ms(1000);
    CHECK(strstr(out, "Habitat conditions lost") && strstr(out, "[WARN] H: 85.00 %RH, exceeds threshold of 80.00 %RH") && count("[TLM]") == 0, "humid: habitat lost + WARN instead of TLM");
    s_hum = 60;
    run_ms(2500);
    clear_out(); ir_pin = 0; run_ms(100); ir_pin = 1; run_ms(100);
    CHECK(strstr(out, "[EVT] Obstacle at close range, holding position") != NULL, "IR in STANDBY -> obstacle");
    clear_out(); ir_pin = 0; run_ms(100); ir_pin = 1; run_ms(100);
    CHECK(strstr(out, "Obstacle") == NULL, "IR event cooldown 3 s");
    clear_out(); single();
    CHECK(strstr(out, "[REPORT] Drone #3: flight") && strstr(out, "habitat found 2x, zombies down 0, shots 0"), "single press -> scout report (habitat found again after the humid spell)");
    clear_out(); fw_tap(FIVEWAY_IDX_CENTER); fw_tap(FIVEWAY_IDX_UP);
    CHECK(strstr(out, "[EVT]") == NULL, "5-way short presses do nothing in STANDBY");
    clear_out(); s_press = 1007.5f; fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(strstr(out, "[EVT] Lab re-registered: bearing") && strstr(out, "altitude zero at 1007.50 hPa"), "5-way centre held 1 s -> lab re-registered");
    CHECK(count("Lab re-registered") == 1, "hold action fires once");
    s_press = 1008.0f; altitude_zero(1008.0f);

    printf("== I2C robustness ==\n");
    clear_out(); sim_stuck_next_read = 1; run_ms(100);
    CHECK(sim_recover_pulses > 0 && i2c_recoveries == 1 && !sim_i2c_busy, "5-way read times out with SDA held low -> bus recovered");
    clear_out(); type("I2CSCAN\r");
    CHECK(strstr(out, "[EVT] I2C bus: 0x03 0x70") && strstr(out, "[recoveries: 1]"), "after recovery both modules answer again");
    sim_fiveway_present = 0; clear_out(); run_ms(600);
    CHECK(fiveway_addr == 0 && strstr(out, "[EVT] 5-way switch lost"), "unplugged 5-way detected");
    sim_fiveway_present = 1; clear_out(); run_ms(2100);
    CHECK(fiveway_addr == 0x03 && strstr(out, "[EVT] 5-way switch connected at 0x03"), "re-plugged 5-way found again in STANDBY");

    printf("== configuration (flash) ==\n");
    clear_out(); type("ID 5\r");
    CHECK(strstr(out, "[CMD] OK Drone #5 (saved)") && cfg.drone_id == 5, "ID 5 saved");
    run_ms(50);
    CHECK(memcmp(mx_shown, GL_DIGIT[5], 8) == 0, "matrix shows the new number");
    clear_out(); type("MUTE ON\r"); type("MOUNT ZDOWN\r");
    CHECK(strstr(out, "[CMD] OK MUTE ON (saved)") && strstr(out, "[CMD] OK MOUNT Z+ down (saved)"), "MUTE + MOUNT saved");
    cfg.drone_id = 1; cfg.muted = false; cfg.mount_down = false;
    cfg_load();
    CHECK(cfg.drone_id == 5 && cfg.muted && cfg.mount_down, "config survives a reset (re-read from flash)");
    bz_fx(SND(SND_REPORT));
    CHECK(bz_quiet(), "MUTE silences the buzzer");
    type("ID 3\r"); type("MUTE OFF\r"); type("MOUNT CHIPUP\r");
    clear_out(); type("ID 0\r"); type("ID 12\r");
    CHECK(count("ERR usage: ID <1..9>") == 2 && cfg.drone_id == 3, "ID outside 1..9 rejected");
    sim_flash_page[0] ^= 0x55;
    cfg.drone_id = 7; cfg_load();
    CHECK(cfg.drone_id == 7, "corrupt config page ignored (checksum)");
    sim_flash_page[0] ^= 0x55; cfg_load();
    CHECK(cfg.drone_id == 3 && !cfg.muted && !cfg.mount_down, "valid config page read back");
    run_ms(1100);

    printf("== dashboard lines ==\n");
    clear_out(); type("DASH ON\r"); run_ms(600);
    CHECK(strstr(out, "[CMD] OK DASH ON") && count("[HUD]") >= 2, "DASH ON -> [HUD] every 250 ms");
    CHECK(strstr(out, "[HUD] ID 3 MODE STANDBY E 0 P 5 SEL 0 CD 0.0 WAVE 0 TGT 0 DOWN 0 SHOTS 0 HDG ") != NULL, "HUD format in STANDBY");
    if (getenv("DBG")) printf("%s\n", last("[HUD]"));
    CHECK(strstr(out, "[MAP]") == NULL, "no [MAP] in STANDBY");

    printf("== BATTLE: map, rule-mode laser ==\n");
    clear_out(); dbl();
    CHECK(state == ST_BATTLE && strstr(out, "Entering BATTLE MODE"), "double -> BATTLE");
    CHECK(map_count() == 3 && wave_no == 1 && strstr(out, "[EVT] Wave 1 incoming: 3 outbreaks, spreading every 3.0 s"), "wave 1: 3 outbreaks");
    CHECK(strstr(out, "[STAT] DRONE: #3, MODE: BATTLE, WAVE: 1") && strstr(out, "WEAPON: RULE 5/10"), "STAT on entering BATTLE");
    CHECK(bb.count == 2 && bb.rec[0].type == BB_SORTIE && bb.rec[1].type == BB_WAVE, "black box restarted at sortie start");
    clear_out(); type("ID 4\r");
    CHECK(strstr(out, "ERR not allowed in BATTLE") && cfg.drone_id == 3, "config commands only in STANDBY");
    int t0 = led_toggles; clear_out(); map_sent_valid = false; run_ms(2000);
    CHECK(led_toggles - t0 == 4, "LED 1 Hz in BATTLE");
    CHECK(count("[TLM]") == 0, "no periodic telemetry in BATTLE");
    {
        char want[64];
        char hex[17];
        for (int r = 0; r < 8; r++) sprintf(&hex[2 * r], "%02X", map_rows[r]);
        snprintf(want, sizeof want, "[MAP] %s AIM 3 3", hex);
        CHECK(strstr(out, want) != NULL, "[MAP] = 8 rows hex + crosshair");
        CHECK(strstr(out, "MODE BATTLE E 0 P 5 SEL 0 CD 0.0 WAVE 1 TGT ") != NULL, "HUD in BATTLE");
    }
    memset(map_rows, 0, 8);
    for (int y = 2; y <= 4; y++) for (int x = 2; x <= 4; x++) cell_set(x, y);   /* 3x3 block */
    cell_set(7, 0);                                   /* keep the wave alive */
    aim_x = 3; aim_y = 3; t_spread = HAL_GetTick();
    clear_out(); single();
    CHECK(strstr(out, "[EVT] LASER charged: 0/10 -> 3/10") && laser_energy == 3, "rule mode: 0 -> 3");
    single();
    CHECK(strstr(out, "LASER FIRED (5/10, BOOM) at (3,3): 9 cells cleared. Energy 6/10 -> 1/10") != NULL, "rule mode: auto-fire at 6, BOOM clears 3x3");
    CHECK(!cooldown_active && zombies_down == 9, "rule mode: no cooldown, 9 zombies down");
    run_ms(250);
    CHECK(strstr(last("[MAP]"), "[MAP] 0100000000000000 AIM 3 3") != NULL, "[MAP] follows the map change");

    printf("== weapon power: 5-way hold and POWER ==\n");
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(strstr(out, "[EVT] Weapon: power 1/10 (PIU), cooldown 0 s") && weapon_sel && weapon_power() == 1, "centre hold: RULE -> 1/10");
    CHECK(strstr(out, "Target locked") == NULL, "hold does not also lock");
    CHECK(strstr(out, "FIRED (1/10, PIU)") && laser_energy == 0, "power change auto-fires the stored 1/10");
    memset(map_rows, 0, 8); cell_set(3, 3); cell_set(4, 3); cell_set(0, 7);
    aim_x = 3; aim_y = 3;
    clear_out(); single();
    CHECK(strstr(out, "LASER FIRED (1/10, PIU) at (3,3): 1 cell cleared. Energy 3/10 -> 2/10") && !cooldown_active, "p=1: single cell, no cooldown");
    fw_tap(FIVEWAY_IDX_RIGHT);
    CHECK(aim_x == 4 && aim_y == 3, "5-way right moves the crosshair");
    clear_out(); single();
    CHECK(strstr(out, "(1/10, PIU) at (4,3): 1 cell cleared") && strstr(out, "Energy 5/10 -> 4/10"), "p=1 fires every press, leftover kept");
    clear_out(); type("POWER 3\r");
    CHECK(weapon_power() == 3 && strstr(out, "power 3/10 (PEW), cooldown 3 s"), "POWER 3");
    CHECK(strstr(out, "FIRED (3/10, PEW)") && cooldown_active && laser_energy == 1, "POWER change auto-fires when energy is enough");
    memset(map_rows, 0, 8);
    for (int x = 2; x <= 6; x++) cell_set(x, 3);
    cell_set(4, 2); cell_set(4, 4); cell_set(3, 2); cell_set(0, 7);
    aim_x = 4; aim_y = 3;
    clear_out(); single();
    CHECK(strstr(out, "FIRED") == NULL && strstr(out, "charged: 1/10 -> 4/10"), "during cooldown: charge only");
    clear_out(); run_ms(3100);
    CHECK(strstr(out, "[EVT] Laser ready") && strstr(out, "FIRED (3/10, PEW) at (4,3): 5 cells cleared"), "cooldown ends -> ready + auto-fire, plus-shaped blast");
    clear_out(); type("POWER NEXT\r"); type("POWER NEXT\r");
    CHECK(strstr(out, "power 4/10 (BOOM)") && strstr(out, "power 5/10 (BOOM)"), "POWER NEXT: 3 -> 4 -> 5");
    clear_out(); type("POWER NEXT\r");
    CHECK(strstr(out, "[EVT] Weapon: rule mode 5/10, no cooldown") && !weapon_sel && !cooldown_active, "POWER NEXT after 5 -> RULE");
    clear_out(); type("POWER 7\r");
    CHECK(strstr(out, "ERR usage: POWER") != NULL, "POWER 7 rejected");

    printf("== controller commands (relayed by the dashboard) ==\n");
    memset(map_rows, 0, 8); cell_set(6, 1); cell_set(0, 7);
    aim_x = 3; aim_y = 3;
    clear_out(); type("AIM 1 0\r"); type("AIM 1 -1\r"); type("AIM 5 0\r");
    CHECK(aim_x == 6 && aim_y == 2 && strstr(out, "[EVT]") == NULL, "AIM moves 1 cell per command, silently");
    for (int i = 0; i < 10; i++) type("AIM 1 0\r");
    CHECK(aim_x == 7, "AIM stops at the edge");
    clear_out(); type("LOCK\r");
    CHECK(strstr(out, "[EVT] Target locked at (6,1)") && aim_x == 6 && aim_y == 1, "LOCK -> nearest target");
    clear_out(); type("RESCUE\r");
    CHECK(strstr(out, "ERR not allowed in BATTLE") != NULL, "RESCUE only when attacked");

    printf("== auto-aim without 5-way or controller ==\n");
    {
        uint8_t keep = fiveway_addr;
        fiveway_addr = 0; aim_manual = false;
        memset(map_rows, 0, 8); cell_set(1, 6); cell_set(7, 0);
        aim_x = 0; aim_y = 7; laser_energy = 4;
        clear_out(); single();
        CHECK(strstr(out, "FIRED (5/10, BOOM) at (1,6)") != NULL, "no aiming device: crosshair jumps to nearest target before firing");
        fiveway_addr = keep;
    }

    printf("== waves ==\n");
    memset(map_rows, 0, 8); cell_set(2, 2); aim_x = 2; aim_y = 2; laser_energy = 4; aim_manual = true;
    clear_out(); single();
    CHECK(strstr(out, "FIRED (5/10, BOOM) at (2,2): 1 cell cleared") != NULL, "last cell destroyed");
    run_ms(20);
    CHECK(!wave_active && strstr(out, "[EVT] Wave 1 cleared! AREA CLEARED") && bb.waves == 1, "AREA CLEARED");
    run_ms(400);
    CHECK(memcmp(mx_shown, GL_CHECK, 8) == 0, "matrix shows the check mark during the pause");
    clear_out(); ir_pin = 0; run_ms(100); ir_pin = 1; run_ms(100);
    CHECK(map_count() == 0 && strstr(out, "ZOMBIE") == NULL, "IR ignored during the pause");
    clear_out(); run_ms(2000);
    CHECK(wave_active && wave_no == 2 && map_count() == 4 && spread_ms == 2750 &&
          strstr(out, "[EVT] Wave 2 incoming: 4 outbreaks, spreading every 2.7 s"), "wave 2: 4 outbreaks, faster spread");

    printf("== BATTLE: spread, IR, overrun ==\n");
    memset(map_rows, 0, 8); cell_set(1, 1); t_spread = HAL_GetTick();
    run_ms(2760);
    CHECK(map_count() == 2, "infection spreads 1 cell per interval");
    run_ms(3000);
    clear_out(); ir_pin = 0; run_ms(100); ir_pin = 1; run_ms(50);
    CHECK(strstr(out, "[EVT] ZOMBIE AT CLOSE RANGE!") && cell(3, 7) && cell(4, 7), "IR in BATTLE -> cells in front infected");
    for (int y = 0; y < 6; y++) map_rows[y] = 0xFF;
    t_spread = HAL_GetTick();
    clear_out(); run_ms(3010);
    CHECK(strstr(out, "[EVT] AREA OVERRUN") != NULL, "overrun reported at >= 75 %");

    printf("== ATTACKED / rescue ==\n");
    int cells_before = map_count();
    clear_out(); s_acc[2] = -1000; run_ms(500);
    CHECK(state == ST_ATTACKED && strstr(out, "[EVT] Drone #3 flipped! The Last of EE2028"), "flip -> ATTACKED");
    CHECK(bz_playing(SND_SOS), "SOS beacon playing");
    CHECK(strstr(out, "[ATTACK] Drone #3 Was Attacked! Rescue in 10 s") != NULL, "attack message with drone id");
    CHECK(strstr(out, "MODE ATTACKED") && strstr(out, "RESC 10"), "HUD carries the rescue countdown");
    t0 = led_toggles; run_ms(2000);
    CHECK(led_toggles - t0 == 8, "LED 2 Hz");
    cfg.muted = true;
    CHECK(!bz_quiet(), "SOS ignores MUTE");
    cfg.muted = false;
    int e = laser_energy; single();
    CHECK(laser_energy == e, "single ignored while attacked");
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100); type("AIM 1 0\r");
    CHECK(strstr(out, "Weapon") == NULL && strstr(out, "ERR not allowed in ATTACKED"), "5-way and AIM ignored while attacked");
    clear_out(); type("RESCUE\r");
    CHECK(state == ST_BATTLE && strstr(out, "Drone #3 rescued!") && map_count() == cells_before, "RESCUE (controller double press) rescues, map kept");
    run_ms(1000);
    CHECK(state == ST_BATTLE, "still upside down after rescue: not re-attacked");
    s_acc[2] = 1000; run_ms(300);
    s_acc[2] = -1000; run_ms(500);
    CHECK(state == ST_ATTACKED, "after flipping back, can be attacked again");
    clear_out(); dbl();
    CHECK(state == ST_BATTLE && strstr(out, "rescued"), "double press rescues too");
    s_acc[2] = 1000; run_ms(300);

    printf("== back to STANDBY + BLACKBOX ==\n");
    clear_out(); dbl();
    CHECK(state == ST_STANDBY && !map_live, "double -> STANDBY, map wiped");
    clear_out(); type("BLACKBOX\r");
    if (getenv("DBG")) printf("----\n%s\n----\n", out);
    CHECK(strstr(out, "LAST MISSION: waves cleared 1, shots fired") && strstr(out, "END: RETURNED"), "black box summary (RETURNED)");
    CHECK(strstr(out, "[BBOX] Last sortie (RAM), drone #3") && strstr(out, "WAVE CLEARED") && strstr(out, "WEAPON") &&
          strstr(out, "FLIPPED Az=-9.80") && strstr(out, "RESCUED") && strstr(out, "BACK TO STANDBY"), "BLACKBOX prints the sortie");
    weapon_sel = true; weapon_p = 2;
    dbl(); clear_out(); type("BLACKBOX\r");
    CHECK(strstr(out, "ERR not allowed in BATTLE") != NULL, "BLACKBOX not allowed in BATTLE");
    CHECK(!weapon_sel && weapon_power() == 5, "new sortie starts in RULE mode");

    printf("== altitude deadband ==\n");
    altitude_zero(1008.0f);
    for (int i = 0; i < 30; i++) altitude_feed(1008.0f + ((i & 1) ? 0.04f : -0.04f));
    CHECK(fabsf(altitude_m()) < 0.01f, "noise of +-0.04 hPa does not move altitude");
    for (int i = 0; i < 30; i++) altitude_feed(1007.8f);
    CHECK(altitude_m() > 1.4f && altitude_m() < 1.9f, "0.2 hPa drop -> about +1.7 m");

    printf("== no rescue -> TERMINATED (black box to flash) ==\n");
    single();
    clear_out(); s_acc[2] = -1000; run_ms(500);
    CHECK(state == ST_ATTACKED, "attacked");
    int cfg_erases = erase_count[0];
    if (setjmp(jmp_end) == 0) { auto_tick = 1; run_ms(11000); }
    auto_tick = 0;
    CHECK(count("Was Attacked!") == 10, "10 attack messages");
    CHECK(strstr(out, "[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.") != NULL, "last message");
    CHECK(standby_entered && !uart_enabled, "comms cut, MCU Standby");
    CHECK(erase_count[0] == cfg_erases, "black box save does not touch the config page");
    bb_flash_check();
    CHECK(bb_flash_valid && bb_flash_image()->drone_id == 3, "black box valid in flash after self-destruct");
    cfg.drone_id = 9; cfg_load();
    CHECK(cfg.drone_id == 3, "config still valid after self-destruct");
    uart_enabled = true; clear_out(); bb_print(bb_flash_image(), "Recovered from flash"); service_isrs();
    CHECK(strstr(out, "FLIPPED") && strstr(out, "SELF-DESTRUCT (no rescue)") && strstr(out, "END: RESCUE TIMEOUT"), "flash black box readable after reset");

    printf("== ROLE CONTROLLER (board 2) ==\n");
    state = ST_STANDBY; standby_entered = 0; s_acc[2] = 1000;
    clear_out();
    if (setjmp(jmp_end) == 0) { free_tick = 1; type("ROLE CONTROLLER\r"); }
    free_tick = 0;
    CHECK(reset_called && strstr(out, "[CMD] OK ROLE CONTROLLER (saved)"), "ROLE CONTROLLER saved, board restarts");
    cfg.role = ROLE_DRONE; cfg_load();
    CHECK(cfg.role == ROLE_CONTROLLER && cfg.drone_id == 3, "after reset: role CONTROLLER read from flash");
    cmd_len = 0; clear_out(); ctl_mode = 1; s_acc[0] = 0; s_acc[1] = 0; s_acc[2] = 1000;
    ctl_init(); run_ms(100);
    CHECK(strstr(out, "=== Pilot controller online ===") != NULL, "controller banner");
    CHECK(!led, "LED off while no drone link");
    clear_out(); s_acc[0] = -500; s_acc[2] = 866; run_ms(400);           /* 30 deg, right edge down */
    CHECK(strstr(out, "[CTL] TILT 1 0") != NULL && count("[CTL] TILT") >= 2 && count("[CTL] TILT") <= 4, "tilt right -> TILT 1 0, repeated every 150 ms");
    clear_out(); s_acc[0] = -260; s_acc[2] = 966; run_ms(300);           /* 15 deg: inside hysteresis */
    CHECK(strstr(out, "[CTL] TILT 1 0") != NULL, "hysteresis: 15 deg keeps the tilt");
    clear_out(); s_acc[0] = 0; s_acc[2] = 1000; run_ms(300);
    CHECK(strstr(out, "[CTL] TILT") == NULL, "level -> no TILT");
    clear_out(); s_acc[1] = -500; s_acc[2] = 866; run_ms(100);           /* top edge down */
    CHECK(strstr(out, "[CTL] TILT 0 -1") != NULL, "tilt forward -> TILT 0 -1 (aim far)");
    s_acc[1] = 0; s_acc[2] = 1000; run_ms(100);
    clear_out(); single();
    CHECK(strstr(out, "[CTL] BTN SINGLE") != NULL, "single press -> BTN SINGLE");
    clear_out(); dbl(); run_ms(600);
    CHECK(strstr(out, "[CTL] BTN DOUBLE") != NULL && strstr(out, "BTN SINGLE") == NULL, "double press -> BTN DOUBLE");
    CHECK(strstr(out, "[CTL] ALIVE") != NULL, "periodic ALIVE");
    type("LINK STANDBY\r"); run_ms(100);
    CHECK(led, "LINK STANDBY -> LED solid");
    t0 = led_toggles;
    for (int i = 0; i < 8; i++) { type("LINK BATTLE\r"); run_ms(245); }
    CHECK(led_toggles - t0 >= 3 && led_toggles - t0 <= 5, "LINK BATTLE -> LED 1 Hz");
    t0 = led_toggles;
    for (int i = 0; i < 8; i++) { type("LINK ATTACKED\r"); run_ms(245); }
    CHECK(led_toggles - t0 >= 7 && led_toggles - t0 <= 9, "LINK ATTACKED -> LED 2 Hz");
    run_ms(2100);
    CHECK(!led, "no LINK for 2 s -> LED off");
    clear_out(); type("STATUS\r");
    CHECK(strstr(out, "[STAT] CONTROLLER: tilt x 0 deg, y 0 deg, link NONE") != NULL, "controller STATUS");
    clear_out(); type("CHARGE\r");
    CHECK(strstr(out, "ERR unknown command 'CHARGE'") != NULL, "drone commands rejected on the controller");
    reset_called = 0;
    if (setjmp(jmp_end) == 0) { free_tick = 1; type("ROLE DRONE\r"); }
    free_tick = 0;
    cfg_load();
    CHECK(reset_called && cfg.role == ROLE_DRONE, "ROLE DRONE switches back");

    printf("\n%s: %d failure(s)\n", fails ? "FAILED" : "ALL PASSED", fails);
    return fails ? 1 : 0;
}
#endif
