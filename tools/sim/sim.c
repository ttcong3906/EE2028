/* Host simulation of samplecode.c (spec v4): fake clock, sensors, button,
 * USART1, Grove 5-way / LED matrix / OLED on I2C, buzzer sequencer,
 * two flash pages. Logic tests only. */
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <stdlib.h>
#define main fw_main
#include "proj/Core/Src/main.c"   /* = Core/Src/samplecode.c, copied by run_tests.sh */
#undef main

/* ---------------- fake hardware ---------------- */
GPIO_TypeDef sim_gpioa, sim_gpiob, sim_gpioc, sim_gpiod;
PERIPH sim_i2c1, sim_usart1, sim_tim3, sim_uart4;
ADC_TypeDef sim_adc1; ADC_Common_TypeDef sim_adc_common; RCC_TypeDef sim_rcc;
FLASH_TypeDef sim_flash_regs = { FLASH_OPTR_DBANK };
uint8_t sim_flash_page[8192];

static uint32_t tick = 0;
static int btn_level = 1;
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

static int sim_oled_present = 1;
static int max_khz_fw = 0, max_khz_disp = 0;      /* fastest speed seen per device */
static int sim_bad_xfer = 0;                      /* transfers with I2C1 disabled or unknown timing */
static int bus_khz(void) { return sim_i2c1.TIMINGR == 0x00420F13u ? 100 : (sim_i2c1.TIMINGR == 0x30420F13u ? 25 : -1); }
static void note_speed(uint16_t a)
{
    int k = bus_khz();
    if (!(sim_i2c1.CR1 & I2C_CR1_PE) || k < 0) sim_bad_xfer++;
    if (a == 0x03 && k > max_khz_fw) max_khz_fw = k;
    if ((a == 0x3C || a == 0x70) && k > max_khz_disp) max_khz_disp = k;
}
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
int sim_i2c_busy = 0; static int sim_recover_pulses = 0;
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h) { h->Instance->CR1 = 0; return HAL_OK; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState st) { if (p == GPIOB && pin == GPIO_PIN_8 && st == GPIO_PIN_SET) { sim_recover_pulses++; sim_i2c_busy = 0; } }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { (void)pin; if (p == GPIOB) return sim_i2c_busy && pin == GPIO_PIN_9 ? GPIO_PIN_RESET : GPIO_PIN_SET; return btn_level ? GPIO_PIN_SET : GPIO_PIN_RESET; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h) { h->Instance->TIMINGR = h->Init.Timing; h->Instance->CR1 |= I2C_CR1_PE; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t a, uint32_t t, uint32_t to) { (void)h;(void)t;(void)to; a >>= 1; if (sim_i2c_busy) return HAL_ERROR; note_speed(a); return ((a == 0x03 && sim_fiveway_present) || a == 0x70 || (a == 0x3C && sim_oled_present)) ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to) { (void)h;(void)to; a >>= 1; if (sim_i2c_busy) return HAL_ERROR; if (a == 0x03 && !sim_fiveway_present) return HAL_ERROR; note_speed(a); if (n >= 1) last_cmd[a & 127] = d[0]; return (a == 0x03 || a == 0x70 || (a == 0x3C && sim_oled_present)) ? HAL_OK : HAL_ERROR; }
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to)
{
    (void)h;(void)to; a >>= 1;
    if (sim_i2c_busy) return HAL_ERROR;
    if (sim_stuck_next_read && a == 0x03) { sim_stuck_next_read = 0; sim_i2c_busy = 1; return HAL_ERROR; }
    note_speed(a);
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
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) { (void)h; (void)n; rx_ptr = d; return HAL_OK; }
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
static void run_ms(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { tick++; app_loop(); service_isrs(); } }
static void btn_edge(int level) { btn_level = level; HAL_GPIO_EXTI_Callback(BUTTON_EXTI13_Pin); }
static void press(uint32_t hold_ms) { btn_edge(0); run_ms(hold_ms); btn_edge(1); }
static void single(void) { run_ms(80); press(60); run_ms(700); }
static void dbl(void) { run_ms(80); press(60); run_ms(150); press(60); run_ms(20); }
static void type(const char *s) { for (; *s; s++) { *rx_ptr = (uint8_t)*s; HAL_UART_RxCpltCallback(&huart1); } run_ms(5); }
static void fw_tap(int idx) { sim_fw = (uint8_t)(1u << idx); run_ms(60); sim_fw = 0; run_ms(60); }
static void fw_hold(int idx, uint32_t ms) { sim_fw = (uint8_t)(1u << idx); run_ms(ms); sim_fw = 0; run_ms(60); }
static void clear_out(void) { outn = 0; out[0] = 0; }
static bool oled_has(int line, const char *txt) { return strstr(oled_text_cache[line], txt) != NULL; }
static bool big_has(int page, const char *txt)  { return oled_text_cache[page][0] == '\x01' && strstr(oled_text_cache[page], txt) != NULL; }
static bool blank(int line) { for (const char *c = oled_text_cache[line]; *c; c++) if (*c != ' ') return false; return true; }
static int count(const char *needle) { int c = 0; const char *p = out; while ((p = strstr(p, needle))) { c++; p++; } return c; }
static const char *last(const char *needle) { const char *p = out, *r = ""; while ((p = strstr(p, needle))) { r = p; p++; } return r; }

/* OLED_DUMP=dir: save the OLED framebuffer as a 128x64 PBM at a few checkpoints. */
static void dump_oled(const char *name)
{
    const char *dir = getenv("OLED_DUMP");
    if (!dir) return;
    char path[256];
    snprintf(path, sizeof path, "%s/%s.pbm", dir, name);
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "P1\n128 64\n");
    for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 128; x++) fputc((oled_fb[y / 8][x] >> (y % 8)) & 1 ? '1' : '0', f);
        fputc('\n', f);
    }
    fclose(f);
}

static int fails = 0;
#define CHECK(cond, msg) do { if (cond) printf("  PASS  %s\n", msg); else { printf("  FAIL  %s\n", msg); fails++; } } while (0)


#ifndef SIM_RT
int main(void)
{
    memset(sim_flash_page, 0xFF, sizeof sim_flash_page);

    printf("== boot ==\n");
    cfg_load();
    app_init(); service_isrs();
    CHECK(!cfg.muted && cfg.hiscore == 0, "blank flash -> sound on, no high score");
    CHECK(strstr(out, "=== Drone #3 online ===\r\n") != NULL, "boot banner");
    CHECK(fiveway_addr == 0x03 && max_khz_fw == 25, "5-way found at 25 kHz with the real ID byte order");
    if (getenv("DBG")) { printf("%s\n", out); for (int i = 0; i < 8; i++) printf("OLED%d [%s]\n", i, oled_text_cache[i]); }
    CHECK(strstr(out, "[EVT] Check: sensors OK, matrix OK, OLED OK, 5-way OK\r\n") != NULL, "system check line");
    CHECK(p0_hpa == 1008.0f && !strstr(out, "Altitude"), "altitude zero at boot, silently");
    CHECK(strstr(out, "Entering STANDBY MODE") != NULL, "Entering STANDBY MODE");
    run_ms(150);
    CHECK(memcmp(mx_shown, GL_DIGIT3, 8) == 0, "matrix shows drone number 3 at boot");
    CHECK(oled_has(0, "== SYSTEM CHECK ==") && oled_has(3, "5-WAY        OK") && oled_has(4, "BEST SCORE   0"), "OLED boot self-test page");
    CHECK(strstr(out, "[TLM] T: 25.00 C, P: 1008.00 hPa, H: 60.00 %RH, A: 9.81 m/s^2, G: 1.00 dps, M: 41.23 uT\r\n") != NULL, "TLM has exactly 6 fields");
    CHECK(strstr(out, "[HUD]") == NULL && strstr(out, "[MAP]") == NULL, "no dashboard lines any more");

    printf("== STANDBY scouting + OLED ==\n");
    clear_out(); run_ms(3100);
    CHECK(big_has(0, "NESTS 0/3") && big_has(3, "ECHO -") && big_has(6, "PINGS 3/3"), "after 2.5 s: STANDBY = NESTS, ECHO, PINGS in big text");
    CHECK(blank(2) && blank(5), "empty page between the big lines");
    CHECK(strstr(out, "[EVT] Habitat found (T 25.0 C, H 60 %RH)") != NULL && habitat_found, "habitat after 3 s of T 20-32 C and H 40-70 %RH");
    run_ms(200);
    CHECK(memcmp(mx_shown, GL_HOUSE, 8) == 0, "matrix shows the house icon");
    clear_out(); s_hum = 85; run_ms(1000);
    CHECK(!habitat_found && strstr(out, "[WARN] H: 85.00 %RH, exceeds threshold of 80.00 %RH") && count("[TLM]") == 0, "humid: habitat lost + WARN instead of TLM");
    run_ms(150);
    CHECK(big_has(0, "NESTS") && !oled_has(6, "WARN"), "warnings stay on the UART, OLED unchanged");
    s_hum = 60;
    run_ms(2500);
    clear_out(); single();
    CHECK(strstr(out, "[EVT] Ping 1/3: echo") && pings_left == 2, "single press -> sonar ping 1/3");
    CHECK(count("\r\n") == count("[TLM]") + 1 && !strstr(out, "[REPORT]"), "ping prints one line only");
    run_ms(1500);
    clear_out(); fw_tap(FIVEWAY_IDX_CENTER);
    CHECK(strstr(out, "[EVT] Ping 2/3") && pings_left == 1, "5-way centre tap = sonar ping too");
    pings_left = 2;
    clear_out(); s_press = 1007.5f; fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(strstr(out, "[EVT] Altitude zero reset") && p0_hpa == 1007.5f && !strstr(out, "Ping"), "5-way centre held 1 s -> altitude zero (no ping)");
    CHECK(count("Altitude zero reset") == 1, "hold action fires once");
    s_press = 1008.0f; altitude_zero(1008.0f);

    printf("== scouting map ==\n");
    scout_new();
    CHECK(popcount8(scout_nest) == SCOUT_NESTS && scout_found_n == 0 && 64 - popcount8(scout_fog) == 6, "new area: 3 hidden nests, only the 3x3 around the start scanned");
    memset(scout_nest, 0, 8); memset(scout_found, 0, 8); memset(scout_fog, 0xFF, 8);
    rset(scout_nest, 3, 4); rset(scout_nest, 6, 1); rset(scout_nest, 0, 0);
    scout_x = 3; scout_y = 7; scout_found_n = 0; scout_scan(false);
    clear_out(); fw_tap(FIVEWAY_IDX_UP);
    CHECK(scout_y == 6 && strstr(out, "Nest found") == NULL, "5-way up: scout moves 1 cell");
    fw_tap(FIVEWAY_IDX_UP);
    CHECK(strstr(out, "[EVT] Nest found (1/3)") && rbit(scout_found, 3, 4) && scout_found_n == 1, "3x3 scan finds the nest next to the drone");
    CHECK(!rbit(scout_fog, 2, 4) && !rbit(scout_fog, 4, 6) && rbit(scout_fog, 3, 3), "fog cleared only around the path");
    CHECK(score == 20, "nest found: +20 points");

    printf("== sonar ==\n");
    run_ms(600);
    clear_out(); type("PING\r");
    CHECK(strstr(out, "[EVT] Ping 1/3: echo NE\r\n") && pings_left == 2, "PING: nearest unfound nest (6,1) is NE (no distance shown)");
    CHECK(snd_echo[1].ms == 490 && snd_echo[2].f0 == 1600 && bz_seq == snd_echo, "echo after 150+100*4 = 550 ms at 2600-250*4 = 1600 Hz");
    CHECK(scout_found_n == 1 && rbit(scout_fog, 6, 1), "sonar does not reveal anything");
    run_ms(190);
    CHECK((mx_shown[2] & (0x80u >> 3)) && (mx_shown[5] & (0x80u >> 6)), "matrix: expanding ring around the scout");
    run_ms(400);
    CHECK((mx_shown[4] & (0x80u >> 4)) && (mx_shown[3] & (0x80u >> 5)) && (mx_shown[2] & (0x80u >> 6)) && (mx_shown[1] & (0x80u >> 7)),
          "matrix: echo ray 4 cells towards NE");
    run_ms(150);
    CHECK(big_has(6, "PINGS 2/3") && big_has(3, "ECHO NE"), "OLED: pings left + big echo direction");
    dump_oled("1_standby");
    run_ms(1000); type("PING\r"); run_ms(1000); type("PING\r"); run_ms(1000);
    clear_out(); type("PING\r");
    CHECK(pings_left == 0 && strstr(out, "[EVT] No pings left"), "only 3 pings per area");
    for (int i = 0; i < 3; i++) fw_tap(FIVEWAY_IDX_RIGHT);
    for (int i = 0; i < 3; i++) fw_tap(FIVEWAY_IDX_UP);
    CHECK(scout_x == 6 && scout_y == 2 && strstr(out, "Nest found (2/3)") && rbit(scout_found, 6, 1), "5-way flies the scout to the second nest");
    CHECK(score == 40, "second nest: score 40");
    run_ms(130);
    {
        uint32_t now = HAL_GetTick();
        bool drone_lit = (mx_shown[2] & (0x80u >> 6)) != 0, fog_lit = (mx_shown[0] & (0x80u >> 7)) != 0;
        CHECK(drone_lit == (((now / 500U) & 1U) != 0) && fog_lit && !(mx_shown[6] & (0x80u >> 3)),
              "matrix: fog lit, scanned dark, drone blinking");
    }
    run_ms(150);
    CHECK(big_has(0, "NESTS 2/3"), "OLED follows the map");
    clear_out(); type("STATUS\r");
    CHECK(strstr(out, "[STAT] STANDBY, alt +0.0 m, nests 2/3, pings 0/3, score 40, best 0\r\n") && count("[STAT]") == 1, "STATUS in STANDBY: one short line");
    pings_left = 2;
    for (int i = 0; i < 6; i++) fw_tap(FIVEWAY_IDX_LEFT);
    for (int i = 0; i < 2; i++) fw_tap(FIVEWAY_IDX_UP);
    CHECK(scout_found_n == 3 && score == 40 + 20 + 30 + 2 * 20 && strstr(out, "[EVT] All nests found, +70 bonus"),
          "all 3 found: +20, +30 bonus, +20 per unused ping");
    wave_start(1); service_isrs();
    CHECK(map_count() == 13 && strstr(out, "[EVT] Wave 1: 3 outbreaks (3 scouted)"), "wave 1: scouted nests = plus of 5 (clipped at the corner)");
    CHECK(cell(3, 3) && cell(3, 5) && cell(2, 4) && cell(4, 4) && cell(0, 0) && cell(1, 0) && cell(0, 1), "cluster shapes in the battle map");
    map_live = false; memset(map_rows, 0, 8);
    scout_new();                                       /* back to a fresh area for the tests below */

    printf("== I2C bus ==\n");
    CHECK(max_khz_fw == 25 && max_khz_disp == 100, "per-device speed: 5-way only ever at 25 kHz, displays at 100 kHz");
    CHECK(sim_bad_xfer == 0, "speed switch rewrites TIMINGR with I2C1 re-enabled before every transfer");
    clear_out(); sim_stuck_next_read = 1; run_ms(100);
    CHECK(sim_recover_pulses > 0 && !sim_i2c_busy, "5-way read times out with SDA held low -> bus recovered");
    CHECK(i2c1_present(0x3C) && i2c1_present(0x70) && fiveway_addr == 0x03, "after recovery all modules answer again");
    sim_fiveway_present = 0; clear_out(); run_ms(600);
    CHECK(fiveway_addr == 0 && strstr(out, "[EVT] 5-way lost"), "unplugged 5-way detected");
    sim_fiveway_present = 1; clear_out(); run_ms(2100);
    CHECK(fiveway_addr == 0x03 && strstr(out, "[EVT] 5-way connected"), "re-plugged 5-way found again in STANDBY");
    sim_oled_present = 0; oled_dirty = 0xFF; clear_out(); run_ms(300);
    CHECK(!oled_ok && strstr(out, "[EVT] OLED lost"), "unplugged OLED detected (no endless retries)");
    oled_ok = true; oled_dirty = 0xFF; oled_flush_all();
    CHECK(oled_dirty != 0, "oled_flush_all gives up on a missing OLED (self-destruct cannot hang)");
    oled_ok = false; sim_oled_present = 1; clear_out(); run_ms(2100);
    CHECK(oled_ok && strstr(out, "[EVT] OLED connected"), "OLED plugged in later is found in STANDBY");

    printf("== configuration (flash) ==\n");
    clear_out(); type("MUTE ON\r");
    CHECK(strstr(out, "[CMD] OK MUTE ON (saved)") != NULL, "MUTE saved");
    cfg.muted = false;
    cfg_load();
    CHECK(cfg.muted, "config survives a reset (re-read from flash)");
    bz_fx(SND(SND_REPORT));
    CHECK(bz_quiet(), "MUTE silences the buzzer");
    type("MUTE OFF\r");
    clear_out(); type("MUTE MAYBE\r"); type("ID 5\r"); type("MOUNT ZDOWN\r");
    CHECK(strstr(out, "ERR usage: MUTE") && count("ERR unknown command") == 2 && !cfg.muted, "bad MUTE rejected, ID/MOUNT are gone");
    sim_flash_page[0] ^= 0x55;
    cfg.muted = true; cfg_load();
    CHECK(cfg.muted, "corrupt config page ignored (checksum)");
    sim_flash_page[0] ^= 0x55; cfg_load();
    CHECK(!cfg.muted, "valid config page read back");
    run_ms(1100);

    printf("== BATTLE: map, rule-mode laser ==\n");
    clear_out(); btn_doubles = 2; run_ms(5);
    CHECK(state == ST_BATTLE && count("Entering BATTLE MODE") == 1 && !strstr(out, "Entering STANDBY"),
          "two queued doubles: only the first acts (stops at the mode change)");
    CHECK(laser_energy == 3 && pings_left == 0,
          "unused pings become laser energy");
    laser_energy = 0;
    CHECK(map_count() == 3 && wave_no == 1 && strstr(out, "[EVT] Wave 1: 3 outbreaks\r\n"), "wave 1 without scouting: 3 single-cell nests");
    CHECK(!strstr(out, "[STAT]") && count("\r\n") == 2, "entering BATTLE: just the mode line and the wave line");
    clear_out(); type("STATUS\r");
    CHECK(strstr(out, "[STAT] BATTLE, wave 1, laser 0/10, rule 5/10, score 0, best 0\r\n") != NULL, "STATUS in BATTLE: one short line");
    CHECK(bb.count == 2 && bb.rec[0].type == BB_SORTIE && bb.rec[1].type == BB_WAVE, "black box restarted at sortie start");
    clear_out(); type("MUTE ON\r");
    CHECK(strstr(out, "ERR not allowed in BATTLE") && !cfg.muted, "config commands only in STANDBY");
    int t0 = led_toggles; clear_out(); run_ms(2000);
    CHECK(led_toggles - t0 == 4, "LED 1 Hz in BATTLE");
    CHECK(count("[TLM]") == 0, "no periodic telemetry in BATTLE");
    CHECK(beat_period() == 1200 - 850 * 3 / 48, "heartbeat period follows the number of infected cells");
    bz_stop(); t_beat = HAL_GetTick() - 2000; run_ms(2);
    CHECK(bz_playing(SND_HEART), "heartbeat plays when the buzzer is free");
    {
        uint8_t keep[8]; memcpy(keep, map_rows, 8);
        memset(map_rows, 0, 8); cell_set(0, 0);
        uint32_t slow = beat_period();
        memset(map_rows, 0xFF, 6);
        uint32_t fast = beat_period();
        memcpy(map_rows, keep, 8);
        CHECK(slow > 1150 && fast == 350, "heartbeat: ~1200 ms nearly empty, 350 ms at 75 %");
    }
    CHECK(big_has(0, "WAVE 1") && oled_text_cache[3][0] == '\x03' && big_has(6, "0") && blank(2) && blank(5),
          "OLED battle screen: big wave, energy bar, big score");
    memset(map_rows, 0, 8);
    for (int y = 2; y <= 4; y++) for (int x = 2; x <= 4; x++) cell_set(x, y);   /* 3x3 block */
    cell_set(7, 0);                                   /* keep the wave alive */
    aim_x = 3; aim_y = 3; t_spread = HAL_GetTick();
    clear_out(); single();
    CHECK(strstr(out, "[EVT] Laser charged: 0/10 -> 3/10") && laser_energy == 3, "rule mode: 0 -> 3");
    single();
    CHECK(strstr(out, "[EVT] Laser fired: 9 down, +90 x1. Energy 6/10 -> 1/10\r\n") != NULL, "rule mode: auto-fire at 6, BOOM clears 3x3, +90");
    CHECK(!bz_playing(SND_HEART), "shot sound interrupts the heartbeat");
    CHECK(!cooldown_active && bb.cells == 9, "rule mode: no cooldown, 9 zombies down");
    run_ms(150);
    CHECK(big_has(6, "90") && strstr(oled_text_cache[3], "3/10") == NULL && strstr(oled_text_cache[3], "1/10"), "OLED: score and energy bar follow the shot");
    dump_oled("2_battle");

    printf("== weapon power: 5-way centre hold ==\n");
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(strstr(out, "[EVT] Weapon: power 1/10\r\n") && weapon_sel && weapon_power() == 1, "centre hold: RULE -> 1/10");
    CHECK(strstr(out, "Target locked") == NULL, "hold does not also lock");
    CHECK(strstr(out, "Laser fired") && laser_energy == 0, "power change auto-fires the stored 1/10");
    memset(map_rows, 0, 8); cell_set(3, 3); cell_set(4, 3); cell_set(0, 7);
    aim_x = 3; aim_y = 3;
    clear_out(); single();
    CHECK(strstr(out, "Laser fired: 1 down, +10 x1. Energy 3/10 -> 2/10") && !cooldown_active, "p=1: single cell, no cooldown (miss before reset the streak)");
    fw_tap(FIVEWAY_IDX_RIGHT);
    CHECK(aim_x == 4 && aim_y == 3, "5-way right moves the crosshair");
    clear_out(); single();
    CHECK(strstr(out, "Laser fired: 1 down, +20 x2. Energy 5/10 -> 4/10"), "p=1 fires every press, leftover kept, streak x2");
    weapon_p = 2;                                     /* skip 2/10 (it would fire the 4/10 stored) */
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(weapon_power() == 3 && strstr(out, "[EVT] Weapon: power 3/10\r\n"), "hold again: 2 -> 3");
    CHECK(strstr(out, "Laser fired") && cooldown_active && laser_energy == 1, "power change auto-fires when energy is enough");
    memset(map_rows, 0, 8);
    for (int x = 2; x <= 6; x++) cell_set(x, 3);
    cell_set(4, 2); cell_set(4, 4); cell_set(3, 2); cell_set(0, 7);
    aim_x = 4; aim_y = 3;
    clear_out(); single();
    CHECK(strstr(out, "fired") == NULL && strstr(out, "charged: 1/10 -> 4/10"), "during cooldown: charge only");
    run_ms(150);
    clear_out(); run_ms(3100);
    CHECK(strstr(out, "[EVT] Laser ready") && strstr(out, "Laser fired: 5 down"), "cooldown ends -> ready + auto-fire, plus-shaped blast");
    laser_energy = 0;
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100); fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(strstr(out, "power 4/10") && strstr(out, "power 5/10"), "hold: 3 -> 4 -> 5");
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100);
    CHECK(strstr(out, "[EVT] Weapon: rule mode 5/10\r\n") && !weapon_sel && !cooldown_active, "hold after 5 -> RULE");

    printf("== crosshair, lock, commands in BATTLE ==\n");
    memset(map_rows, 0, 8); cell_set(6, 1); cell_set(0, 7);
    aim_x = 6; aim_y = 2;
    for (int i = 0; i < 3; i++) fw_tap(FIVEWAY_IDX_RIGHT);
    CHECK(aim_x == 7, "crosshair stops at the edge");
    clear_out(); fw_tap(FIVEWAY_IDX_CENTER);
    CHECK(aim_x == 6 && aim_y == 1 && !strstr(out, "[EVT]"), "centre tap locks the nearest target");
    clear_out(); type("AIM 1 0\r"); type("LOCK\r"); type("POWER 3\r"); type("HOME\r");
    CHECK(count("ERR unknown command") == 4, "AIM/LOCK/POWER/HOME commands are gone");
    clear_out(); type("RESCUE\r");
    CHECK(strstr(out, "ERR not allowed in BATTLE") != NULL, "RESCUE only when attacked");
    clear_out(); type("PING\r");
    CHECK(strstr(out, "ERR not allowed in BATTLE") != NULL, "PING only in STANDBY");

    printf("== auto-aim without 5-way ==\n");
    {
        uint8_t keep = fiveway_addr;
        fiveway_addr = 0;
        memset(map_rows, 0, 8); cell_set(1, 6); cell_set(7, 0);
        aim_x = 0; aim_y = 7; laser_energy = 4;
        clear_out(); single();
        CHECK(strstr(out, "Laser fired: 1 down") && aim_x == 1 && aim_y == 6, "no aiming device: crosshair jumps to nearest target before firing");
        fiveway_addr = keep;
    }

    printf("== waves ==\n");
    memset(map_rows, 0, 8); cell_set(2, 2); aim_x = 2; aim_y = 2; laser_energy = 4; t_spread = HAL_GetTick();
    clear_out(); single();
    CHECK(strstr(out, "Laser fired: 1 down") != NULL, "last cell destroyed");
    run_ms(20);
    CHECK(!wave_active && strstr(out, "[EVT] Wave 1 cleared, +50") && bb.waves == 1, "AREA CLEARED");
    run_ms(400);
    CHECK(memcmp(mx_shown, GL_CHECK, 8) == 0 && big_has(0, "CLEARED!"), "matrix check mark + OLED during the pause");
    laser_energy = 4; clear_out(); single();
    CHECK(strstr(out, "charged: 4/10 -> 7/10") && !strstr(out, "fired") && laser_energy == 7, "no shot into an empty map during the pause");
    clear_out(); run_ms(2000);
    CHECK(wave_active && wave_no == 2 && spread_ms == 2750 &&
          strstr(out, "[EVT] Wave 2: 4 outbreaks"), "wave 2: 4 outbreaks, faster spread");
    CHECK(strstr(out, "Laser fired") && laser_energy == 2, "stored energy fires as soon as the next wave appears");

    printf("== BATTLE: spread, overrun ==\n");
    memset(map_rows, 0, 8); cell_set(1, 1); t_spread = HAL_GetTick();
    run_ms(2760);
    CHECK(map_count() == 2, "infection spreads 1 cell per interval");
    for (int y = 0; y < 6; y++) map_rows[y] = 0xFF;
    t_spread = HAL_GetTick();
    clear_out(); run_ms(3010);
    CHECK(strstr(out, "[EVT] AREA OVERRUN") != NULL, "overrun reported at >= 75 %");

    printf("== ATTACKED / rescue ==\n");
    int cells_before = map_count();
    clear_out(); s_acc[2] = -1000; run_ms(500);
    CHECK(state == ST_ATTACKED && strstr(out, "[EVT] Drone #3 flipped! The Last of EE2028"), "flip -> ATTACKED");
    CHECK(bz_playing(SND_SOS), "SOS beacon playing");
    run_ms(20);
    CHECK((memcmp(mx_shown, GL_S, 8) == 0 || memcmp(mx_shown, GL_O, 8) == 0) && mx_shown[0] && mx_shown[7],
          "matrix SOS letters use all 8 rows");
    CHECK(strstr(out, "[ATTACK] Drone #3 Was Attacked! Rescue in 10 s") != NULL, "attack message with drone id");
    run_ms(150);
    CHECK(oled_has(0, "!! UNDER ATTACK !!") && big_has(1, "10 s") && big_has(4, "2x PRESS") && big_has(6, "TO RESCUE"), "OLED: big countdown and big rescue hint");
    dump_oled("3_attacked");
    t0 = led_toggles; run_ms(2000);
    CHECK(led_toggles - t0 == 8, "LED 2 Hz");
    cfg.muted = true;
    CHECK(!bz_quiet(), "SOS ignores MUTE");
    cfg.muted = false;
    int e = laser_energy; single();
    CHECK(laser_energy == e, "single ignored while attacked");
    clear_out(); fw_hold(FIVEWAY_IDX_CENTER, 1100); type("CHARGE\r");
    CHECK(strstr(out, "Weapon") == NULL && strstr(out, "ERR not allowed in ATTACKED"), "5-way and CHARGE ignored while attacked");
    clear_out(); type("RESCUE\r");
    CHECK(state == ST_BATTLE && strstr(out, "[EVT] Rescued! Back to BATTLE") && !strstr(out, "[STAT]") && map_count() == cells_before, "RESCUE command rescues, map kept");
    run_ms(1000);
    CHECK(state == ST_BATTLE, "still upside down after rescue: not re-attacked");
    s_acc[2] = 1000; run_ms(300);
    s_acc[2] = -1000; run_ms(500);
    CHECK(state == ST_ATTACKED, "after flipping back, can be attacked again");
    clear_out(); dbl();
    CHECK(state == ST_BATTLE && strstr(out, "Rescued"), "double press rescues too");
    s_acc[2] = 1000; run_ms(300);

    printf("== back to STANDBY: score + BLACKBOX ==\n");
    uint32_t sortie_score = score;
    clear_out(); dbl();
    {
        char want[64];
        snprintf(want, sizeof want, "[EVT] NEW HIGH SCORE: %lu\r\n", (unsigned long)sortie_score);
        CHECK(sortie_score > 0 && strstr(out, "[EVT] Mission score:") && strstr(out, want) && cfg.hiscore == sortie_score && score == 0,
              "score settled on return: new high score, then reset for the new area");
        CHECK(bz_seq == SND_HISCORE, "high-score jingle");
        cfg.hiscore = 1; cfg_load();
        CHECK(cfg.hiscore == sortie_score, "high score saved in flash");
    }
    CHECK(state == ST_STANDBY && !map_live && scout_found_n == 0, "double -> STANDBY, map wiped, new area to scout");
    clear_out(); type("BLACKBOX\r");
    CHECK(strstr(out, "LAST MISSION: waves cleared 1, shots fired") && strstr(out, "END: RETURNED"), "black box summary (RETURNED)");
    CHECK(strstr(out, "[BBOX] Last sortie (RAM), drone #3") && strstr(out, "WAVE CLEARED") && strstr(out, "WEAPON") &&
          strstr(out, "FLIPPED Az=-9.80") && strstr(out, "RESCUED") && strstr(out, "BACK TO STANDBY"), "BLACKBOX prints the sortie");
    weapon_sel = true; weapon_p = 2;
    run_ms(1500);
    clear_out(); type("HISCORE RESET\r");
    CHECK(strstr(out, "[CMD] OK HISCORE 0 (saved)") && cfg.hiscore == 0, "HISCORE RESET");
    cfg.hiscore = sortie_score; cfg_save();
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
    cfg.hiscore = 999999;                                  /* no new record: config page untouched */
    int cfg_erases = erase_count[0];
    if (setjmp(jmp_end) == 0) { auto_tick = 1; run_ms(11000); }
    auto_tick = 0;
    CHECK(count("Was Attacked!") == 10, "10 attack messages");
    CHECK(strstr(out, "[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.") != NULL, "last message");
    CHECK(strstr(out, "[EVT] Mission score:") != NULL, "score settled before self-destruct");
    CHECK(standby_entered && !uart_enabled, "comms cut, MCU Standby");
    CHECK(big_has(0, "LOST") && big_has(3, "SCORE ") && (big_has(6, "BEST") || big_has(6, "HI ")) && blank(2) && blank(5) && oled_dirty == 0, "OLED left on the final screen");
    dump_oled("4_terminated");
    CHECK(erase_count[0] == cfg_erases, "black box save does not touch the config page");
    bb_flash_check();
    CHECK(bb_flash_valid && bb_flash_image()->drone_id == 3, "black box valid in flash after self-destruct");
    cfg.hiscore = 0; cfg_load();
    CHECK(cfg.hiscore == sortie_score, "config + high score still valid after self-destruct");
    uart_enabled = true; clear_out(); bb_print(bb_flash_image(), "Recovered from flash"); service_isrs();
    CHECK(strstr(out, "FLIPPED") && strstr(out, "SELF-DESTRUCT (no rescue)") && strstr(out, "END: RESCUE TIMEOUT"), "flash black box readable after reset");

    printf("\n%s: %d failure(s)\n", fails ? "FAILED" : "ALL PASSED", fails);
    return fails ? 1 : 0;
}
#endif
