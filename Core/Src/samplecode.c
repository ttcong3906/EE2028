/* ============================================================================
 * 0. INCLUDES
 * ==========================================================================*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"

/* BSP low-level sensor bus access (defined in stm32l4s5i_iot01.c) */
extern void    SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
extern uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg);

/* ============================================================================
 * 1. CONFIGURATION
 * ==========================================================================*/

/* --- Feature switches --- */
#define ENABLE_MATRIX           1
#define ENABLE_BUZZER           1
#define ENABLE_OLED             1      /* Grove OLED 128x64 (SSD1306/SSD1315) */
#define ENABLE_FIVEWAY          1      /* Grove 5-way switch (I2C)          */
#define ENABLE_FLASH            1      /* black box + config in flash       */
#define TERMINATE_USE_STANDBY   1      /* 1: MCU Standby mode, 0: sleep loop */

#define DRONE_ID                3      /* our drone number               */

/* --- External module quirks (change if the picture looks wrong) --- */
#define MATRIX_I2C_ADDR         0x70
#define MATRIX_FLIP_X           0      /* 1 if letters/arrows are mirrored  */
#define OLED_I2C_ADDR           0x3C
#define OLED_COL_OFFSET         0      /* 2 for SH1106-based 1.3" modules   */

/* --- Seeed Grove multi-switch (5-way tactile) --- */
#define FIVEWAY_I2C_ADDR        0x03
#define MS_CMD_GET_DEV_ID       0x00
#define MS_CMD_GET_DEV_EVENT    0x01
#define MS_VID                  0x2886
#define MS_PID_5WAY             0x0002
#define MS_RAW_ACTIVE_LEVEL     0      /* raw status bit when pressed       */
#define FIVEWAY_IDX_UP          0      /* button index order inside module  */
#define FIVEWAY_IDX_DOWN        1
#define FIVEWAY_IDX_LEFT        2
#define FIVEWAY_IDX_RIGHT       3
#define FIVEWAY_IDX_CENTER      4

/* --- Timing (ms) --- */
#define SENSOR_PERIOD_MS        1000U
#define DEBOUNCE_PRESS_MS       50U
#define DEBOUNCE_RELEASE_MS     30U
#define DOUBLE_PRESS_MS         500U
#define LED_BATTLE_TOGGLE_MS    500U   /* 1 Hz */
#define LED_ATTACK_TOGGLE_MS    250U   /* 2 Hz */
#define FLIP_POLL_MS            100U
#define FLIP_HOLD_MS            300U
#define ATTACK_MSG_MS           1000U
#define RESCUE_WINDOW_MS        10000U
#define FIVEWAY_POLL_MS         30U
#define FIVEWAY_REPEAT_DELAY_MS 400U
#define FIVEWAY_REPEAT_MS       150U
#define FIVEWAY_HOLD_MS         1000U  /* centre held: lab register / weapon power */
#define I2C_TIMEOUT_MS          25U    /* Grove I2C transfers (5-way may stretch the clock) */
#define MODULE_BOOT_MS          300U   /* let the 5-way's own MCU start before probing */
#define MODULE_REPROBE_MS       2000U  /* STANDBY: look again for unplugged modules */
#define HABITAT_HOLD_S          3
#define WAVE_PAUSE_MS           2000U  /* AREA CLEARED -> next wave            */
#define WAVE_SPREAD_START_MS    3000U  /* wave 1 spreads 1 cell per 3 s        */
#define WAVE_SPREAD_STEP_MS     250U   /* each wave spreads 0.25 s faster      */
#define WAVE_SPREAD_MIN_MS      1000U
#define WAVE_MAX_SEEDS          12
#define CROSSHAIR_BLINK_MS      125U   /* 4 Hz */
#define ICON_MS                 1000U
#define HOUSE_ICON_MS           2000U
#define SCOUT_NESTS             3      /* hidden nests per area = wave 1 outbreaks */
#define SONAR_PINGS             3      /* sonar pings per area                 */
#define SONAR_RING_MS           90U    /* ring animation: 4 frames             */
#define SONAR_RAY_MS            700U   /* echo direction ray on the matrix     */
#define BEAT_SLOW_MS            1200U  /* heartbeat period, empty map          */
#define BEAT_FAST_MS            350U   /* heartbeat period, map at 75 %        */
#define BOOT_ID_MS              1000U
#define BEAM_FRAME_MS           50U
#define BLAST_FLASH_MS          150U
#define BOOT_CHECK_MS           2500U  /* OLED system-check page at power-on    */
#define EVENT_TEXT_MS           2000U  /* OLED event line                      */
#define OLED_REFRESH_MS         100U

/* --- Physics / thresholds --- */
#define G_MS2                   9.80665f
#define FLIP_G_FRACTION         0.5f
#define ALT_STEP_HPA            0.1f      /* altitude only moves per 0.1 hPa */
#define HAB_T_MIN               20.0f
#define HAB_T_MAX               32.0f
#define HAB_H_MIN               40.0f
#define HAB_H_MAX               70.0f

/* --- Laser (units of 1/10 capacity) --- */
#define LASER_CHARGE_STEP       3
#define LASER_RULE_COST         5
#define LASER_MAX               10
#define MAP_OVERRUN_CELLS       48         /* 75 % of 64 */

/* ============================================================================
 * 2. TYPES AND GLOBAL STATE
 * ==========================================================================*/
typedef enum { ST_STANDBY = 0, ST_BATTLE, ST_ATTACKED, ST_TERMINATED } sys_state_t;

enum { S_T = 0, S_P, S_H, S_A, S_G, S_M, S_COUNT };

typedef struct {
    const char *tag, *unit;
    float low, high;
    bool  use_low, use_high;
} thresh_t;

/* Safety thresholds: physical reasoning in docs/SPEC.md section 11. */
static thresh_t th[S_COUNT] = {
    { "T", "C",      10.0f,   35.0f,  true,  true  }, /* Li-ion safe range       */
    { "P", "hPa",   950.0f, 1040.0f,  true,  true  }, /* ~540 m flight ceiling   */
    { "H", "%RH",    20.0f,   80.0f,  true,  true  }, /* static / condensation   */
    { "A", "m/s^2",   3.0f,   15.0f,  true,  true  }, /* free fall / impact      */
    { "G", "dps",     0.0f,  300.0f,  false, true  }, /* spin out of control     */
    { "M", "uT",     10.0f,  100.0f,  true,  true  }, /* magnetic interference   */
};

typedef struct {
    float v[S_COUNT];       /* magnitudes in physical units */
} sensors_t;

typedef struct {
    bool    muted;
    uint32_t hiscore;       /* best mission score */
} drone_cfg_t;

static sys_state_t state = ST_STANDBY;
static drone_cfg_t cfg = { false, 0 };
static sensors_t   sens;
static bool        sens_valid = false;
static uint32_t    boot_tick = 0;

static uint32_t t_sensor = 0, t_led = 0, t_flip_poll = 0, t_attack_start = 0, t_attack_msg = 0;
static bool     led_on = false;
static uint32_t warn_mask = 0;

/* Laser and battle statistics */
static int      laser_energy = 0;
static uint32_t score = 0;               /* current mission (scouting + sortie) */
static int      streak = 0;              /* consecutive hitting shots */
static uint32_t shots_fired = 0, zombies_down = 0;
static uint32_t cooldown_until = 0;
static bool     cooldown_active = false;
static bool     weapon_sel = false;      /* false = RULE (5/10, no cooldown) */
static int      weapon_p = 1;            /* power in SEL mode, 1..5 */

/* Scouting statistics */
static uint32_t habitat_count = 0;
static bool     habitat_found = false;
static int      habitat_secs = 0;

/* Flip detection */
static bool     flip_armed = true, flip_candidate = false;
static uint32_t flip_since = 0;

/* Peripherals */
UART_HandleTypeDef huart1;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim3;

static inline bool elapsed(uint32_t since, uint32_t ms)
{
    return (uint32_t)(HAL_GetTick() - since) >= ms;
}

/* Small xorshift PRNG for the infection map (seeded from sensor noise). */
static uint32_t rng_state = 0x2028EE28u;
static uint32_t rng(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static const char *mode_name(sys_state_t s)
{
    switch (s) {
        case ST_STANDBY:  return "STANDBY";
        case ST_BATTLE:   return "BATTLE";
        case ST_ATTACKED: return "ATTACKED";
        default:          return "TERMINATED";
    }
}

/* ============================================================================
 * 3. UART: non-blocking TX ring buffer + interrupt RX, console line editing
 * ==========================================================================*/
#define TX_BUF_SIZE 4096U
#define RX_BUF_SIZE 256U
#define CMD_MAX_LEN 64U

static uint8_t           tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0, tx_inflight = 0;
static volatile bool     tx_busy = false;
static bool              uart_enabled = true;

static uint8_t           rx_byte;
static volatile uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;
static char              cmd_line[CMD_MAX_LEN];
static uint16_t          cmd_len = 0;

/* Start the next interrupt-driven transfer. Call with IRQs off or from ISR. */
static void uart_kick(void)
{
    if (tx_busy || tx_head == tx_tail) return;
    uint16_t len = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail)
                                       : (uint16_t)(TX_BUF_SIZE - tx_tail);
    tx_inflight = len;
    tx_busy = true;
    if (HAL_UART_Transmit_IT(&huart1, &tx_buf[tx_tail], len) != HAL_OK) tx_busy = false;
}

static void uart_write_raw(const char *s)
{
    if (!uart_enabled) return;
    uint32_t pm = __get_PRIMASK();          /* may be called with IRQs already off */
    __disable_irq();
    while (*s) {
        uint16_t next = (uint16_t)((tx_head + 1U) % TX_BUF_SIZE);
        if (next == tx_tail) break;          /* buffer full: drop the rest */
        tx_buf[tx_head] = (uint8_t)*s++;
        tx_head = next;
    }
    uart_kick();
    __set_PRIMASK(pm);
}

/* Console-aware output: if a command is half typed, erase that line
 * (VT100 "\r ESC[K"), print the message, then redraw the prompt. */
static void uart_write(const char *s)
{
    if (cmd_len > 0) {
        uart_write_raw("\r\x1b[K");
        uart_write_raw(s);
        cmd_line[cmd_len] = '\0';
        uart_write_raw("> ");
        uart_write_raw(cmd_line);
    } else {
        uart_write_raw(s);
    }
}

static void uart_printf(const char *fmt, ...)
{
    char buf[240];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_write(buf);
}

static void uart_flush(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((tx_busy || tx_head != tx_tail) && !elapsed(t0, timeout_ms)) { }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    tx_tail = (uint16_t)((tx_tail + tx_inflight) % TX_BUF_SIZE);
    tx_busy = false;
    uart_kick();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_BUF_SIZE);
    if (next != rx_tail) {
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* HAL aborts RX on overrun */
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* ============================================================================
 * 4. FLASH: last page = black box, second-to-last page = configuration.
 *    Dual-bank (default): bank 2 pages 255/254, 4 KB each.
 *    Single-bank: pages 255/254, 8 KB each.
 * ==========================================================================*/
#define FLASH_PAGE_BLACKBOX 1   /* counted from the end of flash */
#define FLASH_PAGE_CONFIG   2

static uint32_t flash_page_size(void)
{
    return (FLASH->OPTR & FLASH_OPTR_DBANK) ? 0x1000UL : 0x2000UL;
}

static uintptr_t flash_tail_addr(int k)
{
    return (uintptr_t)(FLASH_BASE + 0x200000UL - (uint32_t)k * flash_page_size());
}

static bool flash_erase_tail(int k)
{
#if ENABLE_FLASH
    FLASH_EraseInitTypeDef e = {0};
    uint32_t err = 0;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks     = (FLASH->OPTR & FLASH_OPTR_DBANK) ? FLASH_BANK_2 : FLASH_BANK_1;
    e.Page      = (uint32_t)(256 - k);
    e.NbPages   = 1;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    bool ok = HAL_FLASHEx_Erase(&e, &err) == HAL_OK;
    HAL_FLASH_Lock();
    return ok;
#else
    (void)k;
    return false;
#endif
}

/* Erase page k (from the end) and program len bytes (multiple of 8). */
static bool flash_write_tail(int k, const void *src, uint32_t len)
{
#if ENABLE_FLASH
    if (!flash_erase_tail(k)) return false;
    uint32_t addr = (uint32_t)flash_tail_addr(k);
    const uint64_t *w = (const uint64_t *)src;
    bool ok = true;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len / 8U && ok; i++) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8U * i, w[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok;
#else
    (void)k; (void)src; (void)len;
    return false;
#endif
}

/* --- Configuration image (16 bytes) --- */
#define CFG_MAGIC 0x35474643UL   /* "CFG5" */

typedef struct {
    uint32_t magic;
    uint8_t  muted, rsv[3];
    uint32_t sum;
    uint32_t hiscore;
} cfg_image_t;

static uint32_t cfg_checksum(const cfg_image_t *c)
{
    return 0xEE2028u ^ c->magic ^ ((uint32_t)c->muted << 17) ^ (c->hiscore * 2654435761u);
}

static void cfg_load(void)
{
#if ENABLE_FLASH
    const cfg_image_t *f = (const cfg_image_t *)(uintptr_t)flash_tail_addr(FLASH_PAGE_CONFIG);
    if (f->magic == CFG_MAGIC && f->sum == cfg_checksum(f)) {
        cfg.muted      = f->muted != 0;
        cfg.hiscore    = f->hiscore;
    }
#endif
}

static bool cfg_save(void)
{
    cfg_image_t c;
    memset(&c, 0, sizeof(c));
    c.magic = CFG_MAGIC;
    c.muted = cfg.muted;
    c.hiscore = cfg.hiscore;
    c.sum = cfg_checksum(&c);
    return flash_write_tail(FLASH_PAGE_CONFIG, &c, sizeof(c));
}

/* ============================================================================
 * 5. I2C1 BUS FOR GROVE MODULES
 * ==========================================================================*/
static bool i2c1_ready = false;
/* I2C1 kernel clock = PCLK1 = 4 MHz (MSI after reset). TIMINGR fields:
 * PRESC | SCLDEL | SDADEL | SCLH | SCLL. 100 kHz standard mode: tPRESC 0.25 us,
 * SCL low 5.0 us, high 4.0 us, data hold 0.5 us, setup 1.25 us. 25 kHz: the
 * same with PRESC = 3 (everything x4). */
#define I2C_TIMING_100K 0x00420F13UL
#define I2C_TIMING_25K  0x30420F13UL
/* Per-device speed on the shared bus: OLED and matrix at 100 kHz, the 5-way at
 * 25 kHz (its own MCU does not answer at 50/100 kHz, tested on our module).
 * Switching only re-programs the I2C timing register. */
#define I2C_FAST_KHZ    100
#define I2C_FIVEWAY_KHZ 25
static int  i2c_khz = I2C_FIVEWAY_KHZ;
static bool oled_ok = false;

static uint32_t i2c_timing(void)
{
    return (i2c_khz >= 100) ? I2C_TIMING_100K : I2C_TIMING_25K;
}

static void I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;      /* PB8 SCL (D15), PB9 SDA (D14) */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = i2c_timing();
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    i2c1_ready = (HAL_I2C_Init(&hi2c1) == HAL_OK);
}

/* Short busy wait (roughly microseconds at 4 MHz), no SysTick needed. */
static void i2c_gap(uint32_t n)
{
    for (volatile uint32_t i = 0; i < n; i++) { }
}

/* Change the bus speed between transfers. TIMINGR may only be written while
 * the peripheral is disabled (PE = 0), and PE must stay low for 3 APB clocks. */
static void i2c_for(int khz)
{
    if (i2c_khz == khz) return;
    i2c_khz = khz;
    hi2c1.Init.Timing = i2c_timing();
    if (!i2c1_ready) return;
    __HAL_I2C_DISABLE(&hi2c1);
    i2c_gap(2);
    hi2c1.Instance->TIMINGR = hi2c1.Init.Timing;
    __HAL_I2C_ENABLE(&hi2c1);
}

/* A slave left half-way through a transfer (e.g. after a timeout) can hold
 * SDA low forever, and then every device on the bus stops answering. */
static bool i2c1_bus_stuck(void)
{
    return __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET ||
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET;     /* SDA held low */
}

/* Bus recovery: up to 9 clock pulses until the slave releases SDA, then a
 * STOP condition, then a fresh I2C1 init. */
static void i2c1_recover(void)
{
    HAL_I2C_DeInit(&hi2c1);
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &g);
    i2c_gap(20);
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); i2c_gap(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   i2c_gap(10);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); i2c_gap(10);   /* STOP: SDA rises while SCL high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   i2c_gap(10);
    I2C1_Init();
}

/* After a failure: if the bus is stuck, recover it and try once more
 * (otherwise the device simply did not answer). */
static bool i2c1_retry_after(bool ok, int attempt)
{
    if (ok || attempt > 0 || !i2c1_bus_stuck()) return false;
    i2c1_recover();
    return i2c1_ready;
}

static bool i2c1_present(uint8_t addr7)
{
    for (int k = 0; i2c1_ready; k++) {
        bool ok = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2, I2C_TIMEOUT_MS) == HAL_OK;
        if (ok) return true;
        if (!i2c1_retry_after(ok, k)) return false;
    }
    return false;
}

static bool i2c1_write(uint8_t addr7, uint8_t *data, uint16_t n)
{
    for (int k = 0; i2c1_ready; k++) {
        bool ok = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), data, n, I2C_TIMEOUT_MS) == HAL_OK;
        if (ok) return true;
        if (!i2c1_retry_after(ok, k)) return false;
    }
    return false;
}

/* Write one command byte, then read n bytes back (Seeed register protocol).
 * The 5-way's own MCU needs a moment to prepare the answer, so wait a little
 * between the two. */
static bool i2c1_cmd_read(uint8_t addr7, uint8_t cmd, uint8_t *rx, uint16_t n)
{
    for (int k = 0; i2c1_ready; k++) {
        bool ok = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), &cmd, 1, I2C_TIMEOUT_MS) == HAL_OK;
        if (ok) {
            i2c_gap(200);
            ok = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr7 << 1), rx, n, I2C_TIMEOUT_MS) == HAL_OK;
        }
        if (ok) return true;
        if (!i2c1_retry_after(ok, k)) return false;
    }
    return false;
}

/* ============================================================================
 * 6. LED MATRIX 8x8 (HT16K33). Rows top to bottom, bit7 = leftmost column.
 * ==========================================================================*/
static bool    mx_ok = false;
static uint8_t mx_shown[8];
static bool    mx_shown_valid = false;

static const uint8_t GL_SMILE[8]   = {0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C};
static const uint8_t GL_CROSS[8]   = {0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81};
static const uint8_t GL_S[8]       = {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00};
static const uint8_t GL_O[8]       = {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00};
static const uint8_t GL_CHECK[8]   = {0x00,0x01,0x03,0x06,0x8C,0xD8,0x70,0x20};
static const uint8_t GL_HOUSE[8]   = {0x18,0x3C,0x7E,0xFF,0x42,0x5A,0x5A,0x7E};
static const uint8_t GL_DIGIT3[8]  = {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00};   /* drone number */
static uint8_t mx_map(uint8_t b)
{
#if MATRIX_FLIP_X
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) if (b & (1U << i)) r |= (uint8_t)(0x80U >> i);
    return r;
#else
    return b;
#endif
}

static void mx_init(void)
{
#if ENABLE_MATRIX
    i2c_for(I2C_FAST_KHZ);
    mx_ok = i2c1_present(MATRIX_I2C_ADDR);
    if (!mx_ok) return;
    uint8_t c;
    c = 0x21; i2c1_write(MATRIX_I2C_ADDR, &c, 1);  /* oscillator on */
    c = 0xE8; i2c1_write(MATRIX_I2C_ADDR, &c, 1);  /* brightness 8  */
    c = 0x81; i2c1_write(MATRIX_I2C_ADDR, &c, 1);  /* display on    */
    mx_shown_valid = false;
#endif
}

static void mx_show(const uint8_t rows[8])
{
#if ENABLE_MATRIX
    if (!mx_ok) return;
    if (mx_shown_valid && memcmp(rows, mx_shown, 8) == 0) return;
    i2c_for(I2C_FAST_KHZ);
    uint8_t buf[17];
    buf[0] = 0x00;
    for (int r = 0; r < 8; r++) {
        buf[1 + 2 * r] = mx_map(rows[r]);
        buf[2 + 2 * r] = 0x00;
    }
    if (!i2c1_write(MATRIX_I2C_ADDR, buf, sizeof(buf))) return;   /* retry next frame */
    memcpy(mx_shown, rows, 8);
    mx_shown_valid = true;
#else
    (void)rows;
#endif
}

/* ============================================================================
 * 6b. OLED SSD1306/SSD1315 128x64 - framebuffer, one dirty page per loop pass
 * ==========================================================================*/
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, /*   ! " */
    {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, /* # $ % */
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00}, /* & ' ( */
    {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08}, /* ) * + */
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, /* , - . */
    {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, /* / 0 1 */
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, /* 2 3 4 */
    {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, /* 5 6 7 */
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, /* 8 9 : */
    {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, /* ; < = */
    {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E}, /* > ? @ */
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22}, /* A B C */
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, /* D E F */
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, /* G H I */
    {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40}, /* J K L */
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E}, /* M N O */
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, /* P Q R */
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, /* S T U */
    {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63}, /* V W X */
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00}, /* Y Z [ */
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, /* \ ] ^ */
    {0x40,0x40,0x40,0x40,0x40}, {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, /* _ ` a */
    {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x44,0x44,0x48,0x7F}, /* b c d */
    {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E}, /* e f g */
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, /* h i j */
    {0x7F,0x10,0x28,0x44,0x00}, {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, /* k l m */
    {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,0x08}, /* n o p */
    {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20}, /* q r s */
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, /* t u v */
    {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, /* w x y */
    {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00}, {0x00,0x00,0x7F,0x00,0x00}, /* z { | */
    {0x00,0x41,0x36,0x08,0x00}, {0x10,0x08,0x08,0x10,0x08},                             /* } ~   */
};

#define OLED_LINES 8
#define OLED_COLS  21
static uint8_t oled_fb[OLED_LINES][128];
static char    oled_text_cache[OLED_LINES][OLED_COLS + 1];
static uint8_t oled_dirty = 0;

static void oled_cmd(uint8_t c)
{
    uint8_t d[2] = { 0x00, c };
    i2c1_write(OLED_I2C_ADDR, d, 2);
}

static void oled_init(void)
{
#if ENABLE_OLED
    i2c_for(I2C_FAST_KHZ);
    oled_ok = i2c1_present(OLED_I2C_ADDR);
    if (!oled_ok) return;
    static const uint8_t seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14,             /* charge pump on        */
        0x20, 0x02,             /* page addressing mode  */
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40,
        0xA4, 0xA6, 0xAF
    };
    for (unsigned i = 0; i < sizeof(seq); i++) oled_cmd(seq[i]);
    memset(oled_fb, 0, sizeof(oled_fb));
    memset(oled_text_cache, 0, sizeof(oled_text_cache));
    oled_dirty = 0xFF;
#endif
}

/* Write one 21-char text line; only marks the page dirty if text changed. */
static void oled_line(uint8_t line, const char *text)
{
#if ENABLE_OLED
    if (!oled_ok || line >= OLED_LINES) return;
    char padded[OLED_COLS + 1];
    size_t n = strlen(text);
    for (unsigned i = 0; i < OLED_COLS; i++) padded[i] = (i < n) ? text[i] : ' ';
    padded[OLED_COLS] = '\0';
    if (strcmp(padded, oled_text_cache[line]) == 0) return;
    strcpy(oled_text_cache[line], padded);

    uint8_t *row = oled_fb[line];
    memset(row, 0, 128);
    for (unsigned i = 0; i < OLED_COLS; i++) {
        char c = padded[i];
        if (c < 32 || c > 126) c = ' ';
        memcpy(&row[1 + i * 6], font5x7[c - 32], 5);
    }
    oled_dirty |= (uint8_t)(1U << line);
#else
    (void)line; (void)text;
#endif
}

static void oled_linef(uint8_t line, const char *fmt, ...)
{
    char buf[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    oled_line(line, buf);
}

/* 2x scaled text over two pages (12 px per character, 14 px tall). */
static void oled_big(uint8_t page, uint8_t x, const char *text)
{
#if ENABLE_OLED
    if (!oled_ok || page + 1U >= OLED_LINES) return;
    char key[OLED_COLS + 1];
    snprintf(key, sizeof(key), "\x01%c%s", (char)('0' + (x & 63U)), text);
    if (strcmp(key, oled_text_cache[page]) == 0) return;
    strcpy(oled_text_cache[page], key);
    strcpy(oled_text_cache[page + 1U], "\x02");
    memset(oled_fb[page], 0, 128);
    memset(oled_fb[page + 1U], 0, 128);
    for (const char *c = text; *c && x + 10U <= 128U; c++, x = (uint8_t)(x + 12U)) {
        char ch = (*c < 32 || *c > 126) ? ' ' : *c;
        const uint8_t *g = font5x7[ch - 32];
        for (int col = 0; col < 5; col++) {
            uint16_t v = 0;
            for (int b = 0; b < 7; b++) if (g[col] & (1U << b)) v |= (uint16_t)(3U << (2 * b));
            for (int k = 0; k < 2; k++) {
                oled_fb[page][x + col * 2 + k]       = (uint8_t)(v & 0xFFU);
                oled_fb[page + 1U][x + col * 2 + k]  = (uint8_t)(v >> 8);
            }
        }
    }
    oled_dirty |= (uint8_t)(3U << page);
#else
    (void)page; (void)x; (void)text;
#endif
}

/* Two-page tall segmented bar (e.g. laser energy 0..10). */
static void oled_bar(uint8_t page, int value, int total)
{
#if ENABLE_OLED
    if (!oled_ok || page + 1U >= OLED_LINES || total <= 0) return;
    char key[OLED_COLS + 1];
    snprintf(key, sizeof(key), "\x03%d/%d", value, total);
    if (strcmp(key, oled_text_cache[page]) == 0) return;
    strcpy(oled_text_cache[page], key);
    strcpy(oled_text_cache[page + 1U], "\x02");
    memset(oled_fb[page], 0, 128);
    memset(oled_fb[page + 1U], 0, 128);
    int w = 120 / total;                       /* segment pitch */
    for (int i = 0; i < total; i++) {
        int x0 = 4 + i * w, x1 = x0 + w - 3;
        for (int x = x0; x <= x1 && x < 128; x++) {
            bool edge = (x == x0 || x == x1);
            if (i < value || edge) { oled_fb[page][x] = 0xFC; oled_fb[page + 1U][x] = 0x3F; }
            else                   { oled_fb[page][x] = 0x04; oled_fb[page + 1U][x] = 0x20; }
        }
    }
    oled_dirty |= (uint8_t)(3U << page);
#else
    (void)page; (void)value; (void)total;
#endif
}

static void oled_flush_step(void)
{
#if ENABLE_OLED
    if (!oled_ok || oled_dirty == 0) return;
    i2c_for(I2C_FAST_KHZ);
    for (uint8_t p = 0; p < OLED_LINES; p++) {
        if (oled_dirty & (1U << p)) {
            uint8_t col = OLED_COL_OFFSET;
            oled_cmd((uint8_t)(0xB0 + p));
            oled_cmd((uint8_t)(0x00 | (col & 0x0F)));
            oled_cmd((uint8_t)(0x10 | (col >> 4)));
            uint8_t buf[129];
            buf[0] = 0x40;
            memcpy(&buf[1], oled_fb[p], 128);
            static uint8_t fails = 0;
            if (!i2c1_write(OLED_I2C_ADDR, buf, sizeof(buf))) {     /* retry next pass */
                if (++fails >= 10) {                                /* unplugged */
                    fails = 0;
                    oled_ok = false;
                    uart_write("[EVT] OLED lost (no answer on I2C)\r\n");
                }
                return;
            }
            fails = 0;
            oled_dirty &= (uint8_t)~(1U << p);
            return;
        }
    }
#endif
}

/* Bounded: a missing OLED must never stop the self-destruct sequence. */
static void oled_flush_all(void)
{
#if ENABLE_OLED
    for (int i = 0; i < 2 * OLED_LINES && oled_ok && oled_dirty; i++) oled_flush_step();
#endif
}

/* ============================================================================
 * 7. BUZZER (PB1 = TIM3_CH4 PWM) with a non-blocking sequencer
 * ==========================================================================*/
typedef struct { uint16_t f0, f1, ms; uint8_t fx; } note_t;  /* f0->f1 sweep, 0 = rest, fx 1 = rumble */

#define DOT  100
#define DASH 300
/* SOS in Morse: each letter is exactly 6 entries, so letter = idx / 6. */
static const note_t SND_SOS[] = {
    {1000,1000,DOT,0},{0,0,DOT,0},{1000,1000,DOT,0},{0,0,DOT,0},{1000,1000,DOT,0},{0,0,300,0},
    {1000,1000,DASH,0},{0,0,DOT,0},{1000,1000,DASH,0},{0,0,DOT,0},{1000,1000,DASH,0},{0,0,300,0},
    {1000,1000,DOT,0},{0,0,DOT,0},{1000,1000,DOT,0},{0,0,DOT,0},{1000,1000,DOT,0},{0,0,700,0},
};
static const note_t SND_BOOT[]    = { {1500,1500,60,0},{0,0,40,0},{2000,2000,80,0} };
static const note_t SND_CHARGE[]  = { {1000,1000,50,0} };
static const note_t SND_PIU[]     = { {3000,1400,260,0} };                      /* p = 1   */
static const note_t SND_PEW[]     = { {2000,600,180,0} };                       /* p = 2-3 */
static const note_t SND_BOOM[]    = { {420,80,550,1},{0,0,40,0},{120,60,250,1} }; /* p = 4-5 */
static const note_t SND_READY[]   = { {2500,2500,20,0} };
static const note_t SND_MODE[]    = { {2000,2000,60,0},{0,0,60,0},{2000,2000,60,0} };
static const note_t SND_ALARM[]   = { {1500,1500,120,0},{0,0,60,0},{1500,1500,120,0} };
static const note_t SND_HABITAT[] = { {1319,1319,120,0},{0,0,30,0},{1568,1568,220,0} };
static const note_t SND_REPORT[]  = { {1200,1200,40,0},{0,0,30,0},{1600,1600,40,0} };
static const note_t SND_VICTORY[] = { {1047,1047,100,0},{1319,1319,100,0},{1568,1568,100,0},{2093,2093,300,0} };
static const note_t SND_RESCUE[]  = { {1047,1047,150,0},{1319,1319,150,0},{1568,1568,250,0} };
static const note_t SND_NOPING[]  = { {180,140,300,1} };
static const note_t SND_BATTLE[]  = { {659,659,110,0},{0,0,20,0},{659,659,110,0},{0,0,20,0},{784,784,110,0},{0,0,20,0},{988,988,260,0} };
static const note_t SND_HEART[]   = { {90,70,70,0},{0,0,90,0},{80,60,90,0} };   /* lub ... dub */
static const note_t SND_HISCORE[] = { {1047,1047,90,0},{1319,1319,90,0},{1568,1568,90,0},{2093,2093,150,0},{0,0,40,0},
                                      {1568,1568,90,0},{2093,2093,350,0} };
static note_t snd_echo[3];                                 /* ping, gap, echo: built at run time */
static const note_t SND_GAMEOVER[]= { {1568,1568,300,0},{0,0,50,0},{1319,1319,300,0},{0,0,50,0},
                                      {1047,1047,300,0},{0,0,50,0},{784,784,700,0} };
#define SND(x) (x), (uint8_t)(sizeof(x) / sizeof((x)[0]))

static const note_t *bz_seq = NULL;
static uint8_t  bz_len = 0, bz_idx = 0;
static bool     bz_loop = false;
static uint32_t bz_note_t0 = 0, bz_fx_t = 0;
static uint16_t bz_cur_freq = 0;

/* MUTE silences everything except the SOS distress beacon. */
static bool bz_quiet(void)
{
    return cfg.muted && bz_seq != SND_SOS;
}

static void tone_set(uint16_t f)
{
#if ENABLE_BUZZER
    if (bz_quiet()) f = 0;
    if (f == bz_cur_freq) return;
    bz_cur_freq = f;
    if (f == 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        return;
    }
    uint32_t arr = 1000000UL / f - 1UL;              /* timer tick = 1 MHz */
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (arr + 1UL) / 2UL);
    if (__HAL_TIM_GET_COUNTER(&htim3) > arr) __HAL_TIM_SET_COUNTER(&htim3, 0);
#else
    (void)f;
#endif
}

static void buzzer_init(void)
{
#if ENABLE_BUZZER
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_1;                 /* PB1 = Grove D6 */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &g);

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = (uint32_t)(HAL_RCC_GetPCLK1Freq() / 1000000UL) - 1U;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    bz_cur_freq = 1;
    tone_set(0);
#endif
}

static void bz_play(const note_t *seq, uint8_t len, bool loop)
{
    bz_seq = seq; bz_len = len; bz_idx = 0; bz_loop = loop;
    bz_note_t0 = HAL_GetTick();
    tone_set(seq[0].f0);
}

static void bz_stop(void)
{
    bz_seq = NULL;
    tone_set(0);
}

static bool bz_playing(const note_t *seq)
{
    return bz_seq != NULL && (seq == NULL || bz_seq == seq);
}

/* Never interrupt the SOS beacon with a smaller sound. */
static void bz_fx(const note_t *seq, uint8_t len)
{
    if (bz_playing(SND_SOS)) return;
    bz_play(seq, len, false);
}

static void bz_update(void)
{
    if (!bz_seq) return;
    const note_t *n = &bz_seq[bz_idx];
    uint32_t t = HAL_GetTick() - bz_note_t0;
    if (t >= n->ms) {
        bz_idx++;
        if (bz_idx >= bz_len) {
            if (!bz_loop) { bz_stop(); return; }
            bz_idx = 0;
        }
        bz_note_t0 = HAL_GetTick();
        tone_set(bz_seq[bz_idx].f0);
        return;
    }
    if (n->f0 != 0 && (n->f0 != n->f1 || n->fx)) {
        int32_t f = (int32_t)n->f0 + ((int32_t)n->f1 - (int32_t)n->f0) * (int32_t)t / (int32_t)n->ms;
        if (n->fx == 1) {                              /* rumble: random wobble every 15 ms */
            if (!elapsed(bz_fx_t, 15)) return;
            bz_fx_t = HAL_GetTick();
            f += (int32_t)(rng() % 41U) - 20;
            if (f < 40) f = 40;
        }
        tone_set((uint16_t)f);
    }
}

/* ============================================================================
 * 8. ON-BOARD SENSORS, ALTITUDE
 * ==========================================================================*/
#define LSM6DSL_ADDR         0xD4
#define LSM6DSL_REG_D6D_SRC  0x1D
#define LSM6DSL_REG_TAP_CFG  0x58
#define LSM6DSL_REG_TAP_THS  0x59
#define LSM6DSL_REG_MD1_CFG  0x5E

static volatile bool flip_irq = false;

static bool sensors_ok = false;

static void sensors_init(void)
{
    int err = 0;
    err += (BSP_TSENSOR_Init()  != TSENSOR_OK);
    err += (BSP_HSENSOR_Init()  != HSENSOR_OK);
    err += (BSP_PSENSOR_Init()  != PSENSOR_OK);
    err += (BSP_ACCELERO_Init() != ACCELERO_OK);
    err += (BSP_GYRO_Init()     != GYRO_OK);
    err += (BSP_MAGNETO_Init()  != MAGNETO_OK);
    sensors_ok = (err == 0);

    /* LSM6DSL hardware orientation (6D) detection, 60 deg, routed to INT1. */
    uint8_t v;
    v = SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_TAP_THS);
    SENSOR_IO_Write(LSM6DSL_ADDR, LSM6DSL_REG_TAP_THS, (uint8_t)((v & ~0xE0U) | (0x2U << 5)));
    v = SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_TAP_CFG);
    SENSOR_IO_Write(LSM6DSL_ADDR, LSM6DSL_REG_TAP_CFG, (uint8_t)(v | 0x80U));   /* INTERRUPTS_ENABLE */
    v = SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_MD1_CFG);
    SENSOR_IO_Write(LSM6DSL_ADDR, LSM6DSL_REG_MD1_CFG, (uint8_t)(v | 0x04U));   /* INT1_6D */
    (void)SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_D6D_SRC);
}

/* Accelerometer in m/s^2. Board flat, chip up: Az = +9.8 (flipped: -9.8). */
static void accel_read(float a[3])
{
    int16_t r[3] = {0};
    BSP_ACCELERO_AccGetXYZ(r);
    a[0] = (float)r[0] * G_MS2 / 1000.0f;
    a[1] = (float)r[1] * G_MS2 / 1000.0f;
    a[2] = (float)r[2] * G_MS2 / 1000.0f;
}

static void sensors_read(sensors_t *s)
{
    int16_t a[3] = {0}, m[3] = {0};
    float   g[3] = {0};

    s->v[S_T] = BSP_TSENSOR_ReadTemp();        /* deg C */
    s->v[S_P] = BSP_PSENSOR_ReadPressure();    /* hPa   */
    s->v[S_H] = BSP_HSENSOR_ReadHumidity();    /* %RH   */

    BSP_ACCELERO_AccGetXYZ(a);                 /* mg    */
    float ax = (float)a[0] * G_MS2 / 1000.0f, ay = (float)a[1] * G_MS2 / 1000.0f, az = (float)a[2] * G_MS2 / 1000.0f;
    s->v[S_A] = sqrtf(ax * ax + ay * ay + az * az);

    BSP_GYRO_GetXYZ(g);                        /* mdps (sensitivity already applied) */
    float gx = g[0] / 1000.0f, gy = g[1] / 1000.0f, gz = g[2] / 1000.0f;
    s->v[S_G] = sqrtf(gx * gx + gy * gy + gz * gz);

    BSP_MAGNETO_GetXYZ(m);                     /* mGauss; 1 mG = 0.1 uT */
    float mx = (float)m[0] / 10.0f, my = (float)m[1] / 10.0f, mz = (float)m[2] / 10.0f;
    s->v[S_M] = sqrtf(mx * mx + my * my + mz * mz);

    rng_state ^= (uint32_t)(a[0] * 31 + m[1] * 17 + (int)HAL_GetTick());
    if (rng_state == 0) rng_state = 0x2028EE28u;
}

/* --- Altitude: zero at P0, value only moves in 0.1 hPa steps --- */
static float p0_hpa = 1013.25f, p_filt = 1013.25f, p_disp = 1013.25f;

static void altitude_zero(float p)
{
    p0_hpa = p_filt = p_disp = p;
}

static void altitude_feed(float p)
{
    p_filt = 0.7f * p_filt + 0.3f * p;                      /* smooth sensor noise */
    /* Snap to 0.1 hPa steps counted from P0, with hysteresis (0.6 step) so the
     * value does not flicker between two steps. */
    if (fabsf(p_filt - p_disp) > 0.6f * ALT_STEP_HPA)
        p_disp = p0_hpa + roundf((p_filt - p0_hpa) / ALT_STEP_HPA) * ALT_STEP_HPA;
}

static float altitude_m(void)
{
    return 44330.0f * (1.0f - powf(p_disp / p0_hpa, 0.1903f));
}

static uint32_t check_thresholds(const sensors_t *s)
{
    uint32_t mask = 0;
    for (int i = 0; i < S_COUNT; i++) {
        if (th[i].use_low  && s->v[i] < th[i].low)  mask |= 1UL << (2 * i);
        if (th[i].use_high && s->v[i] > th[i].high) mask |= 1UL << (2 * i + 1);
    }
    return mask;
}

/* ============================================================================
 * 9. GROVE 5-WAY SWITCH (I2C)
 * ==========================================================================*/
static uint8_t  fiveway_addr = 0;                    /* 0 = not found */
static uint8_t  fw_bits = 0, fw_prev = 0;            /* 5-way pressed bits */
static uint32_t t_fw = 0, fw_hold_t0 = 0, fw_rep_t = 0;
static uint32_t fw_center_t0 = 0;
static bool     fw_center_held = false;              /* hold action already done */

/* The 5-way answers at 0x03 with its ID: one little-endian uint32
 * (VID << 16) | PID, i.e. the bytes 02 00 86 28 (checked on the module). */
static void fiveway_probe(void)
{
#if ENABLE_FIVEWAY
    static const uint8_t want[4] = { MS_PID_5WAY & 0xFF, MS_PID_5WAY >> 8, MS_VID & 0xFF, MS_VID >> 8 };
    i2c_for(I2C_FIVEWAY_KHZ);
    if (!i2c1_present(FIVEWAY_I2C_ADDR)) return;
    for (int tries = 0; tries < 2 && !fiveway_addr; tries++) {
        uint8_t id[4] = {0};
        if (i2c1_cmd_read(FIVEWAY_I2C_ADDR, MS_CMD_GET_DEV_ID, id, 4) && memcmp(id, want, 4) == 0)
            fiveway_addr = FIVEWAY_I2C_ADDR;
    }
#endif
}

/* Read the raw status of n buttons as a bitmask (bit set = pressed). */
static bool fiveway_read(uint8_t *bits)
{
    uint8_t buf[4 + 5];
    if (!fiveway_addr) return false;
    i2c_for(I2C_FIVEWAY_KHZ);
    if (!i2c1_cmd_read(fiveway_addr, MS_CMD_GET_DEV_EVENT, buf, sizeof(buf))) return false;
    /* Sanity check: event word = 0 except bit 31 (has event), button bytes < 0x20
     * (raw level + click flags). Anything else means the buffer slipped. */
    bool sane = buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && (buf[3] & 0x7FU) == 0;
    for (uint8_t i = 0; i < 5; i++) sane = sane && buf[4 + i] < 0x20U;
    if (!sane) return false;                         /* garbled frame: skip it */
    uint8_t b = 0;
    for (uint8_t i = 0; i < 5; i++) {
        bool raw = (buf[4 + i] & 0x01U) != 0;
        if (raw == (MS_RAW_ACTIVE_LEVEL != 0)) b |= (uint8_t)(1U << i);
    }
    *bits = b;
    return true;
}

/* Weapon: RULE = assignment behaviour (5/10, no cooldown), SEL = power 1..5. */
static int weapon_power(void)
{
    return weapon_sel ? weapon_p : LASER_RULE_COST;
}

/* ============================================================================
 * 10. BUTTON (PC13, EXTI13 both edges) AND IMU INT1 (PD11, EXTI11)
 * ==========================================================================*/
#define BTN_PRESSED_LEVEL GPIO_PIN_RESET

static volatile bool     btn_down = false;
static volatile uint32_t btn_press_t = 0, btn_release_t = 0;
static volatile uint8_t  btn_pending = 0;
static volatile uint32_t btn_first_t = 0;
static volatile uint8_t  btn_singles = 0, btn_doubles = 0;

static void button_on_press(uint32_t now)
{
    if (btn_pending && (now - btn_first_t) < DOUBLE_PRESS_MS) {
        btn_pending = 0;
        btn_doubles++;
    } else {
        if (btn_pending) btn_singles++;
        btn_pending = 1;
        btn_first_t = now;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == BUTTON_EXTI13_Pin) {
        bool pressed = (HAL_GPIO_ReadPin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin) == BTN_PRESSED_LEVEL);
        if (pressed) {
            if (!btn_down &&
                (now - btn_release_t) >= DEBOUNCE_RELEASE_MS &&
                (now - btn_press_t)   >= DEBOUNCE_PRESS_MS) {
                btn_down = true;
                btn_press_t = now;
                button_on_press(now);
            }
        } else {
            btn_down = false;
            btn_release_t = now;
        }
    } else if (GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin) {
        flip_irq = true;
    }
}

static void button_collect(uint8_t *singles, uint8_t *doubles)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    if (btn_pending && elapsed(btn_first_t, DOUBLE_PRESS_MS)) {
        btn_pending = 0;
        btn_singles++;
    }
    *singles = btn_singles;  btn_singles = 0;
    *doubles = btn_doubles;  btn_doubles = 0;
    __set_PRIMASK(pm);
}

/* ============================================================================
 * 11. BLACK BOX: last 32 events of the current sortie, saved to flash on
 *     self-destruct so it survives RESET and power loss.
 * ==========================================================================*/
typedef enum {
    BB_SORTIE = 1, BB_CHARGE, BB_FIRE, BB_RSV4, BB_CLOSE, BB_OVERRUN,
    BB_WEAPON, BB_FLIPPED, BB_RESCUED, BB_STANDBY, BB_DESTRUCT, BB_WAVE, BB_WAVE_CLEAR
} bb_type_t;

enum { BB_END_FLYING = 0, BB_END_RETURNED, BB_END_TIMEOUT };

typedef struct {
    uint32_t t;            /* ms since sortie start */
    uint8_t  type, mode;
    int16_t  a, b, c;
    int32_t  d;
} bb_rec_t;                /* 16 bytes */

#define BB_SIZE  32U
#define BB_MAGIC 0x58424242UL   /* "BBBX" */

typedef struct {
    uint32_t magic;
    uint8_t  drone_id, count, head, end_reason;
    uint32_t sum;
    uint16_t shots, cells;          /* mission summary for this sortie */
    uint16_t waves, flight_s;
    uint32_t rsv;
    bb_rec_t rec[BB_SIZE];
} bb_image_t;              /* 24 + 512 bytes, multiple of 8 for double-word programming */

static bb_image_t bb;                  /* RAM copy of the current sortie */
static uint32_t   bb_t0 = 0;
static bool       bb_flash_valid = false;

static uint32_t bb_checksum(const bb_image_t *img)
{
    const uint32_t *w = (const uint32_t *)(const void *)img->rec;
    uint32_t s = 0x2028u ^ img->drone_id ^ ((uint32_t)img->count << 8) ^ ((uint32_t)img->head << 16) ^
                 ((uint32_t)img->end_reason << 24) ^ img->shots ^ ((uint32_t)img->cells << 16) ^
                 ((uint32_t)img->waves << 3) ^ ((uint32_t)img->flight_s << 11);
    for (unsigned i = 0; i < sizeof(img->rec) / 4U; i++) s = (s << 5 | s >> 27) ^ w[i];
    return s;
}

static void bb_clear(void)
{
    memset(&bb, 0, sizeof(bb));
    bb.drone_id = DRONE_ID;
    bb_t0 = HAL_GetTick();
}

static void bb_log(bb_type_t type, int a, int b, int c, int32_t d)
{
    bb_rec_t *r = &bb.rec[bb.head];
    r->t = HAL_GetTick() - bb_t0;
    r->type = (uint8_t)type; r->mode = (uint8_t)state;
    r->a = (int16_t)a; r->b = (int16_t)b; r->c = (int16_t)c; r->d = d;
    bb.head = (uint8_t)((bb.head + 1U) % BB_SIZE);
    if (bb.count < BB_SIZE) bb.count++;
}

static const bb_image_t *bb_flash_image(void)
{
    return (const bb_image_t *)(uintptr_t)flash_tail_addr(FLASH_PAGE_BLACKBOX);
}

static bool bb_flash_save(void)
{
    bb.magic = BB_MAGIC;
    bb.drone_id = DRONE_ID;
    bb.sum = bb_checksum(&bb);
    return flash_write_tail(FLASH_PAGE_BLACKBOX, &bb, sizeof(bb));
}

static void bb_flash_check(void)
{
#if ENABLE_FLASH
    const bb_image_t *f = bb_flash_image();
    bb_flash_valid = f->magic == BB_MAGIC && f->count <= BB_SIZE && f->head < BB_SIZE &&
                     f->sum == bb_checksum(f);
#endif
}

static const char *end_name(uint8_t e)
{
    return e == BB_END_RETURNED ? "RETURNED" : (e == BB_END_TIMEOUT ? "RESCUE TIMEOUT" : "IN FLIGHT");
}

/* ============================================================================
 * 12. BATTLE: INFECTION MAP, AIMING AND LASER
 * ==========================================================================*/
static uint8_t  map_rows[8];             /* bit7 = column 0 (left) */
static bool     map_live = false;        /* a map exists for the current sortie */
static uint32_t t_spread = 0;
static int      wave_no = 0;
static bool     wave_active = false;     /* false during the AREA CLEARED pause */
static uint32_t wave_pause_t = 0, spread_ms = WAVE_SPREAD_START_MS;
static bool     overrun_reported = false;
static int      aim_x = 3, aim_y = 3;
static uint32_t fire_t0 = 0;
static bool     fire_anim = false;
static int      fire_x = 0, fire_y = 0;
static uint8_t  blast_rows[8];

/* 8x8 bitmaps (battle map, fog, nests): row y, bit7 = column 0. */
static inline bool rbit(const uint8_t r[8], int x, int y) { return (r[y] & (0x80U >> x)) != 0; }
static inline void rset(uint8_t r[8], int x, int y)       { r[y] |= (uint8_t)(0x80U >> x); }
static inline void rclr(uint8_t r[8], int x, int y)       { r[y] &= (uint8_t)~(0x80U >> x); }
static int popcount8(const uint8_t r[8])
{
    int n = 0;
    for (int y = 0; y < 8; y++) n += __builtin_popcount(r[y]);
    return n;
}

static inline bool cell(int x, int y)       { return rbit(map_rows, x, y); }
static inline void cell_set(int x, int y)   { rset(map_rows, x, y); }
static inline void cell_clr(int x, int y)   { rclr(map_rows, x, y); }
static inline bool inside(int x, int y)     { return x >= 0 && x < 8 && y >= 0 && y < 8; }
static inline int  clamp07(int v)           { return v < 0 ? 0 : (v > 7 ? 7 : v); }
static int map_count(void)                  { return popcount8(map_rows); }

static void event_text(const char *t, uint8_t prio);   /* section 13 */

/* --- Scouting (STANDBY): the same 8x8 area under fog, with hidden nests --- */
static uint8_t  scout_fog[8];            /* bit set = not scanned yet */
static uint8_t  scout_nest[8];           /* hidden nests */
static uint8_t  scout_found[8];          /* nests located */
static int      scout_x = 3, scout_y = 7, scout_found_n = 0;
static bool     scout_ready = false;
static int      pings_left = SONAR_PINGS;
static char     last_echo[24] = "";

/* Wave n: 2 + n outbreaks in the far half of the map, spreading faster each wave.
 * Wave 1 uses the scouted area: located nests are bigger (plus of 5 cells),
 * nests nobody found are a single cell. */
static void wave_start(int n)
{
    wave_no = n;
    int seeds = 2 + n;
    if (seeds > WAVE_MAX_SEEDS) seeds = WAVE_MAX_SEEDS;
    uint32_t dec = (uint32_t)(n - 1) * WAVE_SPREAD_STEP_MS;
    spread_ms = (dec + WAVE_SPREAD_MIN_MS < WAVE_SPREAD_START_MS) ? WAVE_SPREAD_START_MS - dec : WAVE_SPREAD_MIN_MS;
    memset(map_rows, 0, sizeof(map_rows));
    int big = 0;
    if (n == 1 && scout_ready) {
        seeds = 0;
        for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
            if (!rbit(scout_nest, x, y)) continue;
            seeds++;
            cell_set(x, y);
            if (rbit(scout_found, x, y)) {
                big++;
                static const int dx[4] = { 0, 0, -1, 1 }, dy[4] = { -1, 1, 0, 0 };
                for (int k = 0; k < 4; k++) if (inside(x + dx[k], y + dy[k])) cell_set(x + dx[k], y + dy[k]);
            }
        }
    } else {
        for (int placed = 0, tries = 0; placed < seeds && tries < 200; tries++) {
            int x = (int)(rng() % 8U), y = (int)(rng() % 5U);
            if (!cell(x, y)) { cell_set(x, y); placed++; }
        }
    }
    overrun_reported = false;
    map_live = true;
    wave_active = true;
    t_spread = HAL_GetTick();
    char extra[48] = "";
    if (big) snprintf(extra, sizeof(extra), " (%d scouted nest%s, bigger)", big, big > 1 ? "s" : "");
    uart_printf("[EVT] Wave %d incoming: %d outbreaks%s, spreading every %lu.%lu s\r\n",
                n, seeds, extra, (unsigned long)(spread_ms / 1000U), (unsigned long)((spread_ms % 1000U) / 100U));
    bb_log(BB_WAVE, n, seeds, big, 0);
}

static void map_new(void)
{
    aim_x = 3; aim_y = 3;
    wave_start(1);
    if (scout_ready && scout_found_n) {               /* crosshair on the nearest located nest */
        int best = 99;
        for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
            int d = abs(x - 3) + abs(y - 3);
            if (rbit(scout_found, x, y) && d < best) { best = d; aim_x = x; aim_y = y; }
        }
        uart_printf("[EVT] Target locked on scouted nest at (%d,%d)\r\n", aim_x, aim_y);
    }
}

static void map_spread(void)
{
    int n = map_count();
    if (n == 0 || n >= 64) return;
    /* Collect every free cell next to an infected one, then infect one at random. */
    static const int dx[4] = { 0, 0, -1, 1 }, dy[4] = { -1, 1, 0, 0 };
    uint8_t cand[64];
    int nc = 0;
    for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
        if (cell(x, y)) continue;
        for (int k = 0; k < 4; k++) {
            int ix = x + dx[k], iy = y + dy[k];
            if (inside(ix, iy) && cell(ix, iy)) { cand[nc++] = (uint8_t)(y * 8 + x); break; }
        }
    }
    if (nc) {
        uint8_t c = cand[rng() % (uint32_t)nc];
        cell_set(c % 8, c / 8);
    }
}

/* Blast footprint around the crosshair by power: 1 cell, plus (5), 3x3 (9). */
static void blast_mask(int p, int cx, int cy, uint8_t out[8])
{
    memset(out, 0, 8);
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            bool in = (dx == 0 && dy == 0) ||
                      (p >= 2 && p <= 3 && (dx == 0 || dy == 0)) ||
                      (p >= 4);
            if (in && inside(cx + dx, cy + dy)) out[cy + dy] |= (uint8_t)(0x80U >> (cx + dx));
        }
    }
}

static const char *power_name(int p)
{
    return p <= 1 ? "PIU" : (p <= 3 ? "PEW" : "BOOM");
}

static void aim_nearest(void)
{
    int best = 99, bx = aim_x, by = aim_y;
    for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
        if (!cell(x, y)) continue;
        int d = abs(x - aim_x) + abs(y - aim_y);
        if (d < best) { best = d; bx = x; by = y; }
    }
    aim_x = bx; aim_y = by;
}

/* Scan the 3x3 cells around the scout drone. */
static void scout_scan(bool announce)
{
    for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
        int x = scout_x + dx, y = scout_y + dy;
        if (!inside(x, y)) continue;
        rclr(scout_fog, x, y);
        if (rbit(scout_nest, x, y) && !rbit(scout_found, x, y)) {
            rset(scout_found, x, y);
            scout_found_n++;
            if (announce) {
                score += 20;
                uart_printf("[EVT] Scouting: nest found at (%d,%d) (%d/%d)\r\n", x, y, scout_found_n, SCOUT_NESTS);
                char t[24];
                snprintf(t, sizeof(t), "NEST FOUND %d/%d", scout_found_n, SCOUT_NESTS);
                event_text(t, 2);
                if (scout_found_n == SCOUT_NESTS) {
                    uint32_t bonus = 30U + 20U * (uint32_t)pings_left;
                    score += bonus;
                    uart_printf("[EVT] Scouting complete: all %d nests located, +%lu bonus (%d pings left). Score %lu\r\n",
                                SCOUT_NESTS, (unsigned long)bonus, pings_left, (unsigned long)score);
                    bz_fx(SND(SND_VICTORY));
                } else {
                    bz_fx(SND(SND_ALARM));
                }
            }
        }
    }
}

/* New area: everything under fog, nests hidden in rows 0..5, drone at the bottom. */
static void scout_new(void)
{
    memset(scout_fog, 0xFF, sizeof(scout_fog));
    memset(scout_nest, 0, sizeof(scout_nest));
    memset(scout_found, 0, sizeof(scout_found));
    for (int placed = 0, tries = 0; placed < SCOUT_NESTS && tries < 200; tries++) {
        int x = (int)(rng() % 8U), y = (int)(rng() % 6U);
        if (!rbit(scout_nest, x, y)) { rset(scout_nest, x, y); placed++; }
    }
    scout_x = 3; scout_y = 7; scout_found_n = 0;
    pings_left = SONAR_PINGS;
    last_echo[0] = '\0';
    score = 0; streak = 0;                            /* a new area is a new mission */
    scout_ready = true;
    scout_scan(false);
}

static int scout_scanned(void)
{
    return 64 - popcount8(scout_fog);
}

static void scout_move(int dx, int dy)
{
    if (state != ST_STANDBY || !scout_ready) return;
    int nx = scout_x + dx, ny = scout_y + dy;
    if (!inside(nx, ny) || (nx == scout_x && ny == scout_y)) return;
    scout_x = nx; scout_y = ny;
    scout_scan(true);
}

/* --- Sonar (single press in STANDBY): a ring goes out, the echo of the
 *     nearest unfound nest comes back sooner and higher the closer it is.
 *     Nothing is revealed: the pilot has to fly there. --- */
static uint32_t sonar_t0 = 0, sonar_echo_ms = 0;
static bool     sonar_anim = false;
static int      echo_d = 0, echo_ux = 0, echo_uy = 0;   /* distance + unit step of the ray */

static const char *dir8(int dx, int dy)          /* N = towards row 0 */
{
    static const char *n[8] = { "E", "NE", "N", "NW", "W", "SW", "S", "SE" };
    float a = atan2f((float)-dy, (float)dx) * 57.29578f;
    if (a < 0) a += 360.0f;
    return n[((int)(a + 22.5f) / 45) % 8];
}

static void sonar_ping(void)
{
    if (state != ST_STANDBY || !scout_ready) return;
    if (pings_left <= 0) {
        uart_write("[EVT] Sonar: no pings left in this area\r\n");
        event_text("NO PINGS LEFT", 1);
        bz_fx(SND(SND_NOPING));
        return;
    }
    pings_left--;
    int best = 99, bx = 0, by = 0;
    for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
        if (!rbit(scout_nest, x, y) || rbit(scout_found, x, y)) continue;
        int dx = x - scout_x, dy = y - scout_y;
        int d = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
        if (d < best) { best = d; bx = dx; by = dy; }
    }
    sonar_t0 = HAL_GetTick();
    sonar_anim = true;
    snd_echo[0] = (note_t){ 2400, 2200, 60, 0 };              /* ping */
    if (best == 99) {                                         /* every nest already found */
        echo_d = 0;
        sonar_echo_ms = 0;
        snprintf(last_echo, sizeof(last_echo), "NO ECHO");
        uart_printf("[EVT] Sonar ping %d/%d: no echo, all nests located\r\n", SONAR_PINGS - pings_left, SONAR_PINGS);
        bz_fx(snd_echo, 1);
    } else {
        echo_d = best;
        echo_ux = bx > 0 ? 1 : (bx < 0 ? -1 : 0);
        echo_uy = by > 0 ? 1 : (by < 0 ? -1 : 0);
        sonar_echo_ms = 150U + 100U * (uint32_t)best;
        uint16_t f = (uint16_t)(2600 - 250 * best);
        snd_echo[1] = (note_t){ 0, 0, (uint16_t)(sonar_echo_ms - 60U), 0 };
        snd_echo[2] = (note_t){ f, f, 90, 0 };                /* the echo */
        snprintf(last_echo, sizeof(last_echo), "ECHO: %d CELL%s %s", best, best > 1 ? "S" : "", dir8(bx, by));
        uart_printf("[EVT] Sonar ping %d/%d: echo from %d cell%s %s\r\n", SONAR_PINGS - pings_left, SONAR_PINGS,
                    best, best > 1 ? "s" : "", dir8(bx, by));
        bz_fx(snd_echo, 3);
    }
    event_text(last_echo, 2);
}

/* ============================================================================
 * 13. DISPLAY CONTENT: OLED PAGES AND LED MATRIX
 * ==========================================================================*/
static const uint8_t *icon = NULL;
static uint32_t icon_until = 0;
static bool     smile = false;
static uint32_t smile_t0 = 0;
static bool     boot_id_shown = false;
static void show_icon(const uint8_t *g, uint32_t ms)
{
    icon = g;
    icon_until = HAL_GetTick() + ms;
}

static int rescue_secs_left(void)
{
    uint32_t t = HAL_GetTick() - t_attack_start;
    if (t >= RESCUE_WINDOW_MS) return 0;
    return (int)((RESCUE_WINDOW_MS - t + 999U) / 1000U);
}

/* --- OLED content ------------------------------------------------------- */
#define OLED_PAGES 4
static uint8_t  oled_page = 0;                   /* STANDBY page 0..3 */
static int      bb_view = 0;                     /* black box page: record shown */
static char     ev_text[24] = "";
static uint32_t ev_until = 0;
static uint8_t  ev_prio = 0;
#define EV_NORMAL 1
#define EV_EFFECT 2
static uint32_t t_oled = 0;

static bool event_active(void) { return ev_text[0] && (int32_t)(ev_until - HAL_GetTick()) > 0; }

/* A message is never replaced by a less important one while still shown. */
static void event_text(const char *t, uint8_t prio)
{
    if (event_active() && prio < ev_prio) return;
    strncpy(ev_text, t, sizeof(ev_text) - 1);
    ev_text[sizeof(ev_text) - 1] = '\0';
    ev_prio = prio;
    ev_until = HAL_GetTick() + EVENT_TEXT_MS;
}

/* Black box shown on OLED: the self-destructed drone's record if any,
 * otherwise the last sortie still in RAM. */
static const bb_image_t *bb_shown(void)
{
    return bb_flash_valid ? bb_flash_image() : &bb;
}

static const bb_rec_t *bb_rec_at(const bb_image_t *img, int i)
{
    uint8_t start = (uint8_t)((img->head + BB_SIZE - img->count) % BB_SIZE);
    return &img->rec[(start + (unsigned)i) % BB_SIZE];
}

static void bb_short(const bb_rec_t *r, char *buf, size_t n)
{
    switch (r->type) {
        case BB_SORTIE:     snprintf(buf, n, "SORTIE START"); break;
        case BB_CHARGE:     snprintf(buf, n, "CHARGE %d->%d", r->a, r->b); break;
        case BB_FIRE:       snprintf(buf, n, "FIRE P%d (%d,%d) x%ld", r->a, r->b, r->c, (long)r->d); break;
        case BB_OVERRUN:    snprintf(buf, n, "OVERRUN %d/64", r->a); break;
        case BB_WEAPON:     snprintf(buf, n, "WEAPON %s %d", r->a ? "SEL" : "RULE", r->b); break;
        case BB_FLIPPED:    snprintf(buf, n, "FLIPPED"); break;
        case BB_RESCUED:    snprintf(buf, n, "RESCUED %ds LEFT", r->a); break;
        case BB_STANDBY:    snprintf(buf, n, "BACK TO STANDBY"); break;
        case BB_DESTRUCT:   snprintf(buf, n, "SELF-DESTRUCT"); break;
        case BB_WAVE:       snprintf(buf, n, "WAVE %d: %d NESTS", r->a, r->b); break;
        case BB_WAVE_CLEAR: snprintf(buf, n, "WAVE %d CLEARED", r->a); break;
        default:            snprintf(buf, n, "?"); break;
    }
}

static void hud_boot_check(void)
{
    oled_line(0, "== SYSTEM CHECK ==");
    oled_linef(1, "SENSORS      %s", sensors_ok ? "OK" : "FAIL");
    oled_linef(2, "LED MATRIX   %s", mx_ok ? "OK" : "NOT FOUND");
    oled_linef(3, "5-WAY        %s", fiveway_addr ? "OK" : "NOT FOUND");
    oled_linef(4, "BEST SCORE   %lu", (unsigned long)cfg.hiscore);
    oled_linef(5, "BLACK BOX    %s", bb_flash_valid ? "FOUND" : "EMPTY");
    oled_line(6, "");
    oled_linef(7, "DRONE #%d  STARTING", DRONE_ID);
}

static void hud_standby(void)
{
    if (!elapsed(boot_tick, BOOT_CHECK_MS)) { hud_boot_check(); return; }
    oled_linef(0, "DRONE#%d STANDBY  %d/%d", DRONE_ID, oled_page + 1, OLED_PAGES);
    if (oled_page == 0) {                                    /* sensors */
        if (sens_valid) {
            oled_linef(1, "T %5.1f C  H %4.1f %%", sens.v[S_T], sens.v[S_H]);
            oled_linef(2, "P %7.2f hPa", sens.v[S_P]);
            oled_linef(3, "A %5.2f m/s2", sens.v[S_A]);
            oled_linef(4, "G %6.2f dps", sens.v[S_G]);
            oled_linef(5, "M %6.1f uT", sens.v[S_M]);
            oled_linef(6, "ALT %+6.1f m", altitude_m());
        }
        if (warn_mask) {
            char buf[40] = "WARN:";
            for (int i = 0; i < S_COUNT; i++) {
                if (warn_mask & (1UL << (2 * i)))     { strcat(buf, " "); strcat(buf, th[i].tag); strcat(buf, "LO"); }
                if (warn_mask & (1UL << (2 * i + 1))) { strcat(buf, " "); strcat(buf, th[i].tag); strcat(buf, "HI"); }
            }
            oled_line(7, buf);
        } else if (event_active()) oled_line(7, ev_text);
        else oled_line(7, "STATUS: ALL NORMAL");
    } else if (oled_page == 1) {                             /* scouting */
        oled_linef(1, "NESTS FOUND   %d/%d%s", scout_found_n, SCOUT_NESTS, scout_found_n >= SCOUT_NESTS ? " ALL" : "");
        oled_linef(2, "PINGS LEFT    %d/%d", pings_left, SONAR_PINGS);
        oled_linef(3, "SCANNED %2d/64 (%d,%d)", scout_scanned(), scout_x, scout_y);
        oled_line(4, last_echo[0] ? last_echo : "ECHO: -");
        oled_line(5, habitat_found ? "HABITAT: FOUND" : "HABITAT: SEARCHING");
        oled_line(6, "");
        oled_line(7, event_active() ? ev_text : "5WAY FLY  MID PAGE");
    } else if (oled_page == 2) {                             /* mission */
        uint32_t sec = (HAL_GetTick() - boot_tick) / 1000U;
        oled_linef(1, "SCORE        %lu", (unsigned long)score);
        oled_linef(2, "BEST         %lu", (unsigned long)cfg.hiscore);
        oled_linef(3, "FLIGHT       %02lu:%02lu", (unsigned long)(sec / 60U), (unsigned long)(sec % 60U));
        oled_linef(4, "ZOMBIES DOWN %lu", (unsigned long)zombies_down);
        oled_linef(5, "SHOTS FIRED  %lu", (unsigned long)shots_fired);
        oled_linef(6, "HABITATS     %lux", (unsigned long)habitat_count);
        oled_line(7, event_active() ? ev_text : "1x PING 2x BATTLE");
    } else {                                                 /* black box */
        const bb_image_t *img = bb_shown();
        oled_line(1, bb_flash_valid ? "BLACK BOX: LOST DRONE" : (img->count ? "BLACK BOX: LAST RUN" : "BLACK BOX: EMPTY"));
        if (img->count == 0) {
            for (uint8_t l = 2; l < 7; l++) oled_line(l, "");
            oled_line(3, "  NO SORTIE YET");
            oled_line(7, "MID PAGE");
            return;
        }
        if (bb_view >= img->count) bb_view = img->count - 1;
        if (bb_view < 0) bb_view = 0;
        const bb_rec_t *rc = bb_rec_at(img, bb_view);
        char ev[32];
        bb_short(rc, ev, sizeof(ev));
        oled_linef(2, "WAVES %u  SHOTS %u", img->waves, img->shots);
        oled_linef(3, "CELLS CLEARED %u", img->cells);
        oled_linef(4, "END: %s", end_name(img->end_reason));
        oled_linef(5, "EVT %02d/%02d t=%lu.%lus", bb_view + 1, img->count,
                   (unsigned long)(rc->t / 1000U), (unsigned long)((rc->t % 1000U) / 100U));
        oled_line(6, ev);
        oled_line(7, "<> EVENTS  MID PAGE");
    }
}

static void hud_battle(void)
{
    int p = weapon_power();
    int n = map_count();
    if (wave_active) oled_linef(0, "WAVE %d   TARGETS %2d", wave_no, n);
    else             oled_linef(0, "WAVE %d   CLEARED!", wave_no);
    oled_bar(1, laser_energy, 10);
    if (cooldown_active) {
        int32_t sleft = (int32_t)(cooldown_until - HAL_GetTick());
        uint32_t left = sleft > 0 ? (uint32_t)sleft : 0U;
        oled_linef(3, "ENERGY %2d/10 COOL%lu.%lu", laser_energy, (unsigned long)(left / 1000U), (unsigned long)((left % 1000U) / 100U));
    } else {
        oled_linef(3, "ENERGY %2d/10 P%d %s", laser_energy, p, weapon_sel ? power_name(p) : "RULE");
    }
    oled_linef(4, "AIM (%d,%d)   %s", aim_x, aim_y, cell(aim_x, aim_y) ? "TARGET!" : "");
    int mult = streak + 1 > 4 ? 4 : streak + 1;
    oled_linef(5, "SCORE %-6lu NEXT x%d", (unsigned long)score, mult);
    if (event_active())              oled_line(6, ev_text);
    else if (n >= MAP_OVERRUN_CELLS) oled_line(6, "AREA OVERRUN!");
    else                             oled_line(6, "");
    oled_line(7, "1x CHARGE 2x STANDBY");
}

static void hud_attacked(void)
{
    char big[16];
    snprintf(big, sizeof(big), "%2d s", rescue_secs_left());
    oled_line(0, "!! UNDER ATTACK !!");
    oled_linef(1, "DRONE #%d  RESCUE IN", DRONE_ID);
    oled_big(2, 40, big);
    oled_line(4, "");
    oled_line(5, "  DOUBLE-PRESS TO");
    oled_line(6, "      RESCUE");
    oled_line(7, "SOS BEACON ON");
}

static void hud_terminated(void)
{
    oled_line(0, "");
    oled_linef(1, "   DRONE #%d LOST", DRONE_ID);
    oled_line(2, "");
    oled_line(3, "  COMMS OFFLINE");
    oled_line(4, "  BLACK BOX SAVED");
    oled_linef(5, "  END: %s", end_name(bb.end_reason));
    oled_line(6, "   press RESET");
    oled_line(7, "");
}

static void hud_render(void)
{
    switch (state) {
        case ST_STANDBY:  hud_standby();    break;
        case ST_BATTLE:   hud_battle();     break;
        case ST_ATTACKED: hud_attacked();   break;
        default:          hud_terminated(); break;
    }
}

/* Text lines are cached, so re-rendering only marks the changed pages dirty. */
static void hud_tick(void)
{
    if (!oled_ok || !elapsed(t_oled, OLED_REFRESH_MS)) return;
    t_oled = HAL_GetTick();
    hud_render();
}

static void matrix_render(void)
{
    uint8_t rows[8];
    uint32_t now = HAL_GetTick();

    if (state == ST_TERMINATED) { mx_show(GL_CROSS); return; }
    if (!boot_id_shown) {
        if (elapsed(boot_tick, BOOT_ID_MS)) boot_id_shown = true;
        else { mx_show(GL_DIGIT3); return; }
    }
    /* ATTACKED has the highest priority: any animation yields immediately. */
    if (state == ST_ATTACKED) {
        mx_show(bz_playing(SND_SOS) && (bz_idx / 6U) == 1U ? GL_O : GL_S);
        return;
    }
    if (smile) {
        if (elapsed(smile_t0, ICON_MS)) smile = false;
        else { mx_show(GL_SMILE); return; }
    }

    if (state == ST_STANDBY) {
        if (icon && (int32_t)(icon_until - now) > 0) { mx_show(icon); return; }
        icon = NULL;
        /* Scouting map: fog lit, scanned cells dark, located nests blink 4 Hz,
         * the drone blinks 1 Hz. */
        bool nest_on = ((now / 125U) & 1U) != 0, drone_on = ((now / 500U) & 1U) != 0;
        for (int r = 0; r < 8; r++)
            rows[r] = nest_on ? (uint8_t)(scout_fog[r] | scout_found[r]) : (uint8_t)(scout_fog[r] & ~scout_found[r]);
        if (drone_on) rset(rows, scout_x, scout_y); else rclr(rows, scout_x, scout_y);
        if (sonar_anim) {
            uint32_t t = now - sonar_t0;
            if (t < 4U * SONAR_RING_MS) {                     /* expanding ring, radius 1..4 */
                int r = 1 + (int)(t / SONAR_RING_MS);
                for (int y = scout_y - r; y <= scout_y + r; y++) for (int x = scout_x - r; x <= scout_x + r; x++)
                    if (inside(x, y) && (abs(x - scout_x) == r || abs(y - scout_y) == r)) rset(rows, x, y);
            }
            if (echo_d && t >= sonar_echo_ms && t < sonar_echo_ms + SONAR_RAY_MS) {   /* echo: ray towards the nest */
                for (int k = 1; k <= echo_d; k++) {
                    int x = scout_x + echo_ux * k, y = scout_y + echo_uy * k;
                    if (inside(x, y)) rset(rows, x, y);
                }
            }
            if (t >= 4U * SONAR_RING_MS && t >= sonar_echo_ms + SONAR_RAY_MS) sonar_anim = false;
        }
        mx_show(rows);
        return;
    }

    /* BATTLE: infection map + crosshair + fire animation */
    memcpy(rows, map_rows, 8);
    if (fire_anim) {
        uint32_t t = now - fire_t0;
        uint32_t beam_ms = 4U * BEAM_FRAME_MS;
        if (t < beam_ms) {
            int frame = (int)(t / BEAM_FRAME_MS) + 1;
            for (int k = 1; k <= frame; k++) {
                float f = (float)k / 4.0f;
                int x = (int)(3.5f + ((float)fire_x - 3.5f) * f + 0.5f);
                int y = (int)(7.0f + ((float)fire_y - 7.0f) * f + 0.5f);
                if (inside(x, y)) rows[y] |= (uint8_t)(0x80U >> x);
            }
            mx_show(rows);
            return;
        }
        if (t < beam_ms + BLAST_FLASH_MS) {
            for (int r = 0; r < 8; r++) rows[r] |= blast_rows[r];
            mx_show(rows);
            return;
        }
        fire_anim = false;
    }
    if (!wave_active) { mx_show(GL_CHECK); return; }      /* AREA CLEARED pause */
    if ((now / CROSSHAIR_BLINK_MS) & 1U) rows[aim_y] ^= (uint8_t)(0x80U >> aim_x);
    mx_show(rows);
}

/* ============================================================================
 * 14. STATE MACHINE
 * ==========================================================================*/
static void led_set(bool on)
{
    led_on = on;
    if (on) BSP_LED_On(LED2); else BSP_LED_Off(LED2);
}

static void send_stat(void)
{
    int p = weapon_power();
    uart_printf("[STAT] DRONE: #%d, MODE: %s, WAVE: %d, LASER: %d/10, WEAPON: %s %d/10, SHOTS: %lu, ZOMBIES DOWN: %lu, ALT: %+.1f m, SCOUTED: %d/64, NESTS: %d/%d\r\n",
                DRONE_ID, mode_name(state), map_live ? wave_no : 0, laser_energy,
                weapon_sel ? "SELECT" : "RULE", p,
                (unsigned long)shots_fired, (unsigned long)zombies_down,
                altitude_m(), scout_scanned(), scout_found_n, SCOUT_NESTS);
    uart_printf("[STAT] SCORE: %lu, BEST: %lu, HABITAT: %s, MUTE: %s, OLED: %s, LED MATRIX: %s, 5-WAY: %s\r\n",
                (unsigned long)score, (unsigned long)cfg.hiscore, habitat_found ? "FOUND" : "SEARCHING",
                cfg.muted ? "ON" : "OFF",
                oled_ok ? "OK" : "missing",
                mx_ok ? "OK" : "missing", fiveway_addr ? "OK" : "missing");
}

static void laser_try_fire(void);

/* --- Mission score: settled when the sortie ends (back to STANDBY or self-destruct) --- */
static void score_finalize(bool play)
{
    uart_printf("[EVT] Mission score: %lu (best %lu)\r\n", (unsigned long)score, (unsigned long)cfg.hiscore);
    if (score <= cfg.hiscore) return;
    uart_printf("[EVT] NEW HIGH SCORE: %lu (was %lu)\r\n", (unsigned long)score, (unsigned long)cfg.hiscore);
    cfg.hiscore = score;
    cfg_save();
    event_text("NEW HIGH SCORE!", EV_EFFECT);
    if (play) bz_fx(SND(SND_HISCORE));
}

static void enter_standby(void)
{
    bool from_battle = (state == ST_BATTLE);
    if (from_battle) {
        bb_log(BB_STANDBY, 0, 0, 0, 0);
        bb.end_reason = BB_END_RETURNED;
        bb.flight_s = (uint16_t)((HAL_GetTick() - bb_t0) / 1000U);
    }
    state = ST_STANDBY;
    uart_write("Entering STANDBY MODE\r\n");
    led_set(true);
    warn_mask = 0;
    map_live = false;                 /* map is wiped when leaving BATTLE */
    if (from_battle) score_finalize(true);
    scout_new();                      /* a new area to scout, score back to 0 */
    oled_page = 0;
    bb_view = 0;
    cooldown_active = false;
    fire_anim = false;
    t_sensor = HAL_GetTick() - SENSOR_PERIOD_MS;
}

static void enter_battle(bool from_rescue)
{
    state = ST_BATTLE;
    if (!from_rescue) {
        uart_write("Entering BATTLE MODE\r\n");
        flip_armed = true;
        weapon_sel = false;                             /* every sortie starts in RULE mode */
        streak = 0;
        bz_fx(SND(SND_BATTLE));
        bb_clear();                                     /* new sortie, new black box */
        if (bb_flash_valid) { flash_erase_tail(FLASH_PAGE_BLACKBOX); bb_flash_valid = false; }
        if (pings_left > 0) {                           /* unused sonar pings -> laser energy */
            int before = laser_energy;
            laser_energy += pings_left;
            if (laser_energy > LASER_MAX) laser_energy = LASER_MAX;
            uart_printf("[EVT] Unused sonar pings: +%d/10 laser energy (%d/10 -> %d/10)\r\n",
                        pings_left, before, laser_energy);
            pings_left = 0;
        }
        bb_log(BB_SORTIE, laser_energy, weapon_power(), 0, 0);
        map_new();
    }
    flip_candidate = false;
    led_set(true);
    t_led = HAL_GetTick();
    t_flip_poll = HAL_GetTick();
    t_spread = HAL_GetTick();
    if (!wave_active) wave_pause_t = HAL_GetTick();
    send_stat();
    if (!from_rescue) laser_try_fire();                 /* bonus energy may already be enough */
}

static void enter_attacked(float az)
{
    bb_log(BB_FLIPPED, 0, 0, 0, (int32_t)(az * 100.0f));
    state = ST_ATTACKED;
    t_attack_start = HAL_GetTick();
    t_attack_msg = t_attack_start - ATTACK_MSG_MS;
    fire_anim = false;
    led_set(true);
    t_led = t_attack_start;
    bz_play(SND(SND_SOS), true);                        /* distress beacon, ignores MUTE */
    uart_printf("[EVT] Drone #%d flipped! The Last of EE2028\r\n", DRONE_ID);
}

static void rescue(void)
{
    uart_printf("[EVT] Drone #%d rescued! Returning to BATTLE MODE\r\n", DRONE_ID);
    event_text("RESCUED!", EV_EFFECT);
    bb_log(BB_RESCUED, rescue_secs_left(), 0, 0, 0);
    flip_armed = false;
    smile = true;
    smile_t0 = HAL_GetTick();
    bz_stop();
    bz_play(SND(SND_RESCUE), false);
    enter_battle(true);
}

static void terminate(void)
{
    bb_log(BB_DESTRUCT, 0, 0, 0, 0);
    score_finalize(false);
    bb.end_reason = BB_END_TIMEOUT;
    bb.flight_s = (uint16_t)((HAL_GetTick() - bb_t0) / 1000U);
    state = ST_TERMINATED;
    bool saved = bb_flash_save();
    bz_stop();
    uart_printf("[EVT] Drone #%d: no rescue. Black box %s. Self-destructing. Cutting all communication.\r\n",
                DRONE_ID, saved ? "saved" : "NOT saved");
    led_set(false);
    matrix_render();
    hud_render();
    oled_flush_all();                                    /* the OLED keeps this image */
    bz_play(SND(SND_GAMEOVER), false);

    uint32_t t0 = HAL_GetTick();
    while (!elapsed(t0, 2500U)) bz_update();        /* let the last message and tune finish */
    uart_flush(500);

    /* Cut all communication (spec) */
    uart_enabled = false;
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_DeInit(&huart1);
    tone_set(0);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    BSP_LED_Off(LED2);

#if TERMINATE_USE_STANDBY
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN5);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTANDBYMode();
#endif
    HAL_SuspendTick();
    __disable_irq();
    while (1) { __WFI(); }
}

/* --- Laser --- */
static void laser_fire(void)
{
    int p = weapon_power();
    int before = laser_energy;
    laser_energy -= p;
    shots_fired++;

    /* No 5-way to aim with: lock the nearest target. */
    if (!fiveway_addr) aim_nearest();

    blast_mask(p, aim_x, aim_y, blast_rows);
    int cleared = 0;
    for (int y = 0; y < 8; y++) for (int x = 0; x < 8; x++) {
        if ((blast_rows[y] & (0x80U >> x)) && cell(x, y)) { cell_clr(x, y); cleared++; }
    }
    zombies_down += (uint32_t)cleared;
    bb.shots++;
    bb.cells = (uint16_t)(bb.cells + cleared);
    fire_anim = true; fire_t0 = HAL_GetTick(); fire_x = aim_x; fire_y = aim_y;
    if (p <= 1)      bz_fx(SND(SND_PIU));
    else if (p <= 3) bz_fx(SND(SND_PEW));
    else             bz_fx(SND(SND_BOOM));

    if (cleared) {
        int mult = streak + 1 > 4 ? 4 : streak + 1;     /* hit streak: x1, x2, x3, x4 */
        uint32_t pts = 10U * (uint32_t)cleared * (uint32_t)mult;
        score += pts;
        streak++;
        uart_printf("[EVT] LASER FIRED (%d/10, %s) at (%d,%d): %d cell%s cleared, +%lu points (x%d). Energy %d/10 -> %d/10. Zombies down: %lu\r\n",
                    p, power_name(p), aim_x, aim_y, cleared, cleared > 1 ? "s" : "", (unsigned long)pts, mult,
                    before, laser_energy, (unsigned long)zombies_down);
        char t[40];
        snprintf(t, sizeof(t), "HIT! %d +%lu x%d", cleared, (unsigned long)pts, mult);
        event_text(t, EV_EFFECT);
    } else {
        streak = 0;
        uart_printf("[EVT] LASER FIRED (%d/10, %s) at (%d,%d): missed, no zombies there. Energy %d/10 -> %d/10\r\n",
                    p, power_name(p), aim_x, aim_y, before, laser_energy);
        event_text("MISSED!", EV_EFFECT);
    }
    bb_log(BB_FIRE, p, aim_x, aim_y, cleared);

    if (weapon_sel && p >= 2) {
        cooldown_active = true;
        cooldown_until = HAL_GetTick() + (uint32_t)p * 1000U;
    }
    if (map_count() < MAP_OVERRUN_CELLS) overrun_reported = false;
}

/* Auto-fire rule: enough energy, gun not cooling, and zombies on the map
 * (energy is kept during the AREA CLEARED pause and fires at the next wave). */
static bool laser_can_fire(void)
{
    return state == ST_BATTLE && wave_active && !cooldown_active && laser_energy >= weapon_power();
}

static void laser_try_fire(void)
{
    if (laser_can_fire()) laser_fire();
}

static void laser_charge(void)
{
    int before = laser_energy;
    laser_energy += LASER_CHARGE_STEP;
    if (laser_energy > LASER_MAX) laser_energy = LASER_MAX;
    uart_printf("[EVT] LASER charged: %d/10 -> %d/10\r\n", before, laser_energy);
    bb_log(BB_CHARGE, before, laser_energy, 0, 0);
    if (laser_can_fire()) laser_fire();
    else bz_fx(SND(SND_CHARGE));
}

/* --- Weapon power (5-way centre hold): RULE -> 1 -> 2 -> 3 -> 4 -> 5 -> RULE --- */
static void weapon_next(void)
{
    if (!weapon_sel)       { weapon_sel = true; weapon_p = 1; }
    else if (weapon_p < 5) weapon_p++;
    else                   weapon_sel = false;
    bool sel = weapon_sel;
    int  p   = weapon_p;
    char t[24];
    if (sel) {
        uart_printf("[EVT] Weapon: power %d/10 (%s), cooldown %d s\r\n", p, power_name(p), p >= 2 ? p : 0);
        snprintf(t, sizeof(t), "POWER %d/10 %s", p, power_name(p));
    } else {
        uart_write("[EVT] Weapon: rule mode 5/10, no cooldown\r\n");
        snprintf(t, sizeof(t), "RULE MODE 5/10");
    }
    event_text(t, EV_NORMAL);
    if (!sel) cooldown_active = false;
    bb_log(BB_WEAPON, sel, weapon_power(), 0, 0);
    bz_fx(SND(SND_REPORT));
    laser_try_fire();
}

static void aim_move(int dx, int dy)
{
    aim_x = clamp07(aim_x + dx);
    aim_y = clamp07(aim_y + dy);
}

static void target_lock(void)
{
    aim_nearest();
    uart_printf("[EVT] Target locked at (%d,%d)\r\n", aim_x, aim_y);
    event_text("TARGET LOCKED", EV_NORMAL);
}

/* --- Altitude zero here (5-way centre hold in STANDBY) --- */
static void altitude_rezero(void)
{
    altitude_zero(BSP_PSENSOR_ReadPressure());
    uart_printf("[EVT] Altitude zero reset at %.2f hPa\r\n", p0_hpa);
    event_text("ALTITUDE ZERO SET", EV_NORMAL);
    bz_fx(SND(SND_MODE));
}

/* --- Scout report (single press in STANDBY) --- */
static void scout_report(void)
{
    uint32_t s = (HAL_GetTick() - boot_tick) / 1000U;
    uart_printf("[REPORT] Drone #%d: flight %02lu:%02lu, alt %+.1f m, area scanned %d/64, nests found %d/%d, "
                "pings left %d/%d, score %lu, habitat found %lux, zombies down %lu, shots %lu\r\n",
                DRONE_ID, (unsigned long)(s / 60U), (unsigned long)(s % 60U), altitude_m(),
                scout_scanned(), scout_found_n, SCOUT_NESTS, pings_left, SONAR_PINGS, (unsigned long)score,
                (unsigned long)habitat_count, (unsigned long)zombies_down, (unsigned long)shots_fired);
}

/* --- Actions shared by the push button and UART commands --- */
static void action_single(void)
{
    switch (state) {
    case ST_STANDBY: sonar_ping(); scout_report(); break;
    case ST_BATTLE:  laser_charge(); break;
    default: break;                         /* ignored while attacked (spec) */
    }
}

static void action_double(void)
{
    switch (state) {
    case ST_STANDBY:  enter_battle(false); break;
    case ST_BATTLE:   bz_fx(SND(SND_MODE)); enter_standby(); break;
    case ST_ATTACKED: rescue(); break;
    default: break;
    }
}

/* --- 5-way switch --- */
static void fiveway_dir(int idx)
{
    int dx = idx == FIVEWAY_IDX_LEFT ? -1 : (idx == FIVEWAY_IDX_RIGHT ? 1 : 0);
    int dy = idx == FIVEWAY_IDX_UP   ? -1 : (idx == FIVEWAY_IDX_DOWN  ? 1 : 0);
    if (state == ST_STANDBY && oled_page == 3 && dx) {  /* black box page: browse */
        int n = bb_shown()->count;
        bb_view += dx;
        if (bb_view < 0) bb_view = 0;
        if (bb_view > n - 1) bb_view = n > 0 ? n - 1 : 0;
    }
    else if (state == ST_STANDBY) scout_move(dx, dy);  /* fly the scout drone */
    else if (state == ST_BATTLE) aim_move(dx, dy);    /* move the crosshair  */
}

static void fiveway_center_short(void)
{
    if (state == ST_BATTLE) target_lock();
    else if (state == ST_STANDBY) { oled_page = (uint8_t)((oled_page + 1U) % OLED_PAGES); bb_view = 0; }
}

static void fiveway_center_hold(void)
{
    if (state == ST_STANDBY) altitude_rezero();
    else if (state == ST_BATTLE) weapon_next();
}

static void fiveway_tick(void)
{
#if ENABLE_FIVEWAY
    uint8_t b;
    if (!fiveway_addr || !elapsed(t_fw, FIVEWAY_POLL_MS)) return;
    t_fw = HAL_GetTick();
    static uint8_t fw_fail = 0;
    if (!fiveway_read(&b)) {
        if (++fw_fail >= 10) {                       /* ~0.3 s of silence: unplugged */
            fw_fail = 0;
            fiveway_addr = 0;
            fw_bits = fw_prev = 0;
            uart_write("[EVT] 5-way switch lost (no answer on I2C)\r\n");
        }
        return;
    }
    fw_fail = 0;
    fw_bits = b;
    const uint8_t C = (uint8_t)(1U << FIVEWAY_IDX_CENTER);
    uint8_t pressed  = (uint8_t)(fw_bits & ~fw_prev);
    uint8_t released = (uint8_t)(~fw_bits & fw_prev);

    /* Directions: act on press, auto-repeat while held. */
    for (int i = 0; i < 4; i++) if (pressed & (1U << i)) { fiveway_dir(i); fw_hold_t0 = HAL_GetTick(); fw_rep_t = fw_hold_t0; }
    uint8_t held = (uint8_t)(fw_bits & fw_prev & (uint8_t)~C);
    if (held && elapsed(fw_hold_t0, FIVEWAY_REPEAT_DELAY_MS) && elapsed(fw_rep_t, FIVEWAY_REPEAT_MS)) {
        fw_rep_t = HAL_GetTick();
        for (int i = 0; i < 4; i++) if (held & (1U << i)) fiveway_dir(i);
    }

    /* Centre: short press on release, hold action once at 1 s. */
    if (pressed & C) { fw_center_t0 = HAL_GetTick(); fw_center_held = false; }
    if ((fw_bits & C) && !fw_center_held && elapsed(fw_center_t0, FIVEWAY_HOLD_MS)) {
        fw_center_held = true;
        fiveway_center_hold();
    }
    if ((released & C) && !fw_center_held) fiveway_center_short();
    fw_prev = fw_bits;
#endif
}

/* --- Per-state periodic work --- */
static void habitat_check(void)
{
    bool ok = sens.v[S_T] >= HAB_T_MIN && sens.v[S_T] <= HAB_T_MAX &&
              sens.v[S_H] >= HAB_H_MIN && sens.v[S_H] <= HAB_H_MAX;
    if (ok) {
        if (habitat_secs < 100) habitat_secs++;
        if (!habitat_found && habitat_secs >= HABITAT_HOLD_S) {
            habitat_found = true;
            habitat_count++;
            uart_printf("[EVT] Possible human habitat found: T %.1f C, H %.0f %%RH\r\n", sens.v[S_T], sens.v[S_H]);
            show_icon(GL_HOUSE, HOUSE_ICON_MS);
            bz_fx(SND(SND_HABITAT));
        }
    } else {
        habitat_secs = 0;
        if (habitat_found) {
            habitat_found = false;
            uart_write("[EVT] Habitat conditions lost\r\n");
        }
    }
}

static void standby_tick(void)
{
    if (elapsed(t_sensor, SENSOR_PERIOD_MS)) {
        t_sensor += SENSOR_PERIOD_MS;
        if (elapsed(t_sensor, SENSOR_PERIOD_MS)) t_sensor = HAL_GetTick();

        sensors_read(&sens);
        sens_valid = true;
        altitude_feed(sens.v[S_P]);
        warn_mask = check_thresholds(&sens);

        if (warn_mask == 0) {
            uart_printf("[TLM] T: %.2f C, P: %.2f hPa, H: %.2f %%RH, A: %.2f m/s^2, G: %.2f dps, M: %.2f uT\r\n",
                        sens.v[S_T], sens.v[S_P], sens.v[S_H], sens.v[S_A], sens.v[S_G], sens.v[S_M]);
        } else {
            for (int i = 0; i < S_COUNT; i++) {
                if (warn_mask & (1UL << (2 * i + 1)))
                    uart_printf("[WARN] %s: %.2f %s, exceeds threshold of %.2f %s\r\n",
                                th[i].tag, sens.v[i], th[i].unit, th[i].high, th[i].unit);
                if (warn_mask & (1UL << (2 * i)))
                    uart_printf("[WARN] %s: %.2f %s, below threshold of %.2f %s\r\n",
                                th[i].tag, sens.v[i], th[i].unit, th[i].low, th[i].unit);
            }
        }
        habitat_check();
    }
}

/* Heartbeat: faster as the map fills up (1200 ms empty -> 350 ms at 75 %). */
static uint32_t t_beat = 0;
static uint32_t beat_period(void)
{
    int n = map_count();
    if (n > MAP_OVERRUN_CELLS) n = MAP_OVERRUN_CELLS;
    return BEAT_SLOW_MS - (BEAT_SLOW_MS - BEAT_FAST_MS) * (uint32_t)n / MAP_OVERRUN_CELLS;
}

static void battle_tick(void)
{
    if (elapsed(t_led, LED_BATTLE_TOGGLE_MS)) {
        t_led += LED_BATTLE_TOGGLE_MS;
        if (elapsed(t_led, LED_BATTLE_TOGGLE_MS)) t_led = HAL_GetTick();
        led_set(!led_on);
    }
    if (map_live && wave_active && !bz_playing(NULL) && elapsed(t_beat, beat_period())) {
        t_beat = HAL_GetTick();
        bz_play(SND(SND_HEART), false);                  /* only when the buzzer is free */
    }

    /* Waves: cleared map -> AREA CLEARED, short pause, next (bigger, faster) wave */
    if (map_live && wave_active && map_count() == 0) {
        wave_active = false;
        wave_pause_t = HAL_GetTick();
        bb.waves++;
        bb_log(BB_WAVE_CLEAR, wave_no, bb.cells, 0, 0);
        score += 50U * (uint32_t)wave_no;
        uart_printf("[EVT] Wave %d cleared! AREA CLEARED (%u cells cleared this sortie), +%d bonus. Score %lu\r\n",
                    wave_no, bb.cells, 50 * wave_no, (unsigned long)score);
        event_text("AREA CLEARED!", EV_EFFECT);
        bz_fx(SND(SND_VICTORY));
    } else if (map_live && !wave_active && elapsed(wave_pause_t, WAVE_PAUSE_MS)) {
        wave_start(wave_no + 1);
        laser_try_fire();                                /* energy kept over the pause */
    } else if (map_live && wave_active && elapsed(t_spread, spread_ms)) {
        t_spread = HAL_GetTick();
        map_spread();
        int n = map_count();
        if (n >= MAP_OVERRUN_CELLS && !overrun_reported) {
            overrun_reported = true;
            uart_printf("[EVT] AREA OVERRUN (%d/64)\r\n", n);
            bb_log(BB_OVERRUN, n, 0, 0, 0);
        } else if (n < MAP_OVERRUN_CELLS) {
            overrun_reported = false;
        }
    }

    /* Cooldown */
    if (cooldown_active && (int32_t)(cooldown_until - HAL_GetTick()) <= 0) {
        cooldown_active = false;
        uart_write("[EVT] Laser ready\r\n");
        bz_fx(SND(SND_READY));
        laser_try_fire();
    }

    /* Flip detection: IMU interrupt for instant response, polling as a safety net. */
    if (flip_irq || elapsed(t_flip_poll, FLIP_POLL_MS)) {
        flip_irq = false;
        t_flip_poll = HAL_GetTick();
        float a[3];
        accel_read(a);                                   
        float lim = FLIP_G_FRACTION * G_MS2;
        if (!flip_armed) {
            if (a[2] > lim) flip_armed = true;
            flip_candidate = false;
        } else if (a[2] < -lim) {
            if (!flip_candidate) {
                flip_candidate = true;
                flip_since = HAL_GetTick();
            } else if (elapsed(flip_since, FLIP_HOLD_MS)) {
                enter_attacked(a[2]);
            }
        } else {
            flip_candidate = false;
        }
    }
}

static void attacked_tick(void)
{
    if (elapsed(t_attack_start, RESCUE_WINDOW_MS)) {
        terminate();
        return;
    }
    if (elapsed(t_led, LED_ATTACK_TOGGLE_MS)) {
        t_led += LED_ATTACK_TOGGLE_MS;
        if (elapsed(t_led, LED_ATTACK_TOGGLE_MS)) t_led = HAL_GetTick();
        led_set(!led_on);
    }
    if (elapsed(t_attack_msg, ATTACK_MSG_MS)) {
        t_attack_msg += ATTACK_MSG_MS;
        uart_printf("[ATTACK] Drone #%d Was Attacked! Rescue in %d s\r\n", DRONE_ID, rescue_secs_left());
    }
}

/* ============================================================================
 * 15. UART COMMAND INTERFACE
 * ==========================================================================*/
static void cmd_help(void)
{
    uart_write("[CMD] Commands:\r\n"
               "[CMD]   HELP | STATUS | THRESH | SET <T|P|H|A|G|M>_<LOW|HIGH> <v>\r\n"
               "[CMD]   MODE STANDBY|BATTLE | CHARGE | RESCUE | PING\r\n"
               "[CMD]   standby: BLACKBOX | MUTE ON|OFF | HISCORE RESET\r\n"
               );
}

static void cmd_thresh(void)
{
    for (int i = 0; i < S_COUNT; i++) {
        char lo[16] = "-", hi[16] = "-";
        if (th[i].use_low)  snprintf(lo, sizeof(lo), "%.2f", th[i].low);
        if (th[i].use_high) snprintf(hi, sizeof(hi), "%.2f", th[i].high);
        uart_printf("[CMD] %s: low %s, high %s %s\r\n", th[i].tag, lo, hi, th[i].unit);
    }
}

static void cmd_not_allowed(void)
{
    uart_printf("[CMD] ERR not allowed in %s\r\n", mode_name(state));
}

static void cmd_cfg_saved(const char *what)
{
    bool ok = cfg_save();
    uart_printf("[CMD] OK %s (%s)\r\n", what, ok ? "saved" : "NOT saved");
}

static const char *bb_type_name(uint8_t t)
{
    switch (t) {
        case BB_SORTIE:   return "SORTIE START";
        case BB_CHARGE:   return "CHARGE";
        case BB_FIRE:     return "FIRE";
        case BB_CLOSE:    return "ZOMBIE CLOSE";
        case BB_OVERRUN:  return "AREA OVERRUN";
        case BB_WEAPON:   return "WEAPON";
        case BB_FLIPPED:  return "FLIPPED";
        case BB_RESCUED:  return "RESCUED";
        case BB_STANDBY:  return "BACK TO STANDBY";
        case BB_DESTRUCT: return "SELF-DESTRUCT (no rescue)";
        case BB_WAVE:     return "WAVE START";
        case BB_WAVE_CLEAR: return "WAVE CLEARED";
        default:          return "?";
    }
}

static void bb_print(const bb_image_t *img, const char *source)
{
    if (img->count == 0) { uart_write("[BBOX] empty\r\n"); return; }
    uart_printf("[BBOX] %s, drone #%d, %d records\r\n", source, img->drone_id, img->count);
    uart_printf("[BBOX] LAST MISSION: waves cleared %u, shots fired %u, cells cleared %u, flight %u s, END: %s\r\n",
                img->waves, img->shots, img->cells, img->flight_s, end_name(img->end_reason));
    uint8_t start = (uint8_t)((img->head + BB_SIZE - img->count) % BB_SIZE);
    for (uint8_t i = 0; i < img->count; i++) {
        const bb_rec_t *r = &img->rec[(start + i) % BB_SIZE];
        char extra[64] = "";
        switch (r->type) {
            case BB_CHARGE:  snprintf(extra, sizeof(extra), " %d/10 -> %d/10", r->a, r->b); break;
            case BB_FIRE:    snprintf(extra, sizeof(extra), " p=%d at (%d,%d), %ld cell%s cleared", r->a, r->b, r->c, (long)r->d, r->d == 1 ? "" : "s"); break;
            case BB_OVERRUN: snprintf(extra, sizeof(extra), " %d/64", r->a); break;
            case BB_WEAPON:  snprintf(extra, sizeof(extra), " %s %d/10", r->a ? "select" : "rule", r->b); break;
            case BB_FLIPPED: snprintf(extra, sizeof(extra), " Az=%ld.%02ld m/s^2", (long)(r->d / 100), (long)labs(r->d % 100)); break;
            case BB_RESCUED: snprintf(extra, sizeof(extra), " with %d s left", r->a); break;
            case BB_SORTIE:  snprintf(extra, sizeof(extra), " energy %d/10, power %d/10", r->a, r->b); break;
            case BB_WAVE:    snprintf(extra, sizeof(extra), " %d, %d outbreaks", r->a, r->b); break;
            case BB_WAVE_CLEAR: snprintf(extra, sizeof(extra), " %d (%d cells so far)", r->a, r->b); break;
            default: break;
        }
        uart_printf("[BBOX] t=%3lu.%lus %-9s %s%s\r\n", (unsigned long)(r->t / 1000U), (unsigned long)((r->t % 1000U) / 100U),
                    mode_name((sys_state_t)r->mode), bb_type_name(r->type), extra);
    }
}

static bool parse_on_off(const char *verb, const char *arg, bool *on)
{
    if (arg && !strcmp(arg, "ON"))  { *on = true;  return true; }
    if (arg && !strcmp(arg, "OFF")) { *on = false; return true; }
    uart_printf("[CMD] ERR usage: %s ON | %s OFF\r\n", verb, verb);
    return false;
}

static void cmd_exec(char *line)
{
    for (char *p = line; *p; p++) *p = (char)toupper((unsigned char)*p);
    char *verb = strtok(line, " \t");
    char *arg1 = strtok(NULL, " \t");
    char *arg2 = strtok(NULL, " \t");
    if (!verb) return;


    if (!strcmp(verb, "HELP"))   { cmd_help();   return; }
    if (!strcmp(verb, "STATUS")) { send_stat();  return; }
    if (!strcmp(verb, "THRESH")) { cmd_thresh(); return; }

    if (!strcmp(verb, "SET")) {
        if (!arg1 || !arg2) { uart_write("[CMD] ERR usage: SET T_HIGH 30\r\n"); return; }
        char *end = NULL;
        float val = strtof(arg2, &end);
        if (end == arg2) { uart_write("[CMD] ERR value must be a number\r\n"); return; }
        char *us = strchr(arg1, '_');
        if (!us) { uart_write("[CMD] ERR usage: SET T_HIGH 30\r\n"); return; }
        *us = '\0';
        for (int i = 0; i < S_COUNT; i++) {
            if (!strcmp(arg1, th[i].tag)) {
                if (!strcmp(us + 1, "LOW"))       { th[i].low  = val; th[i].use_low  = true; }
                else if (!strcmp(us + 1, "HIGH")) { th[i].high = val; th[i].use_high = true; }
                else break;
                uart_printf("[CMD] OK %s_%s = %.2f %s\r\n", th[i].tag, us + 1, val, th[i].unit);
                return;
            }
        }
        uart_write("[CMD] ERR unknown key\r\n");
        return;
    }

    if (!strcmp(verb, "MODE")) {
        if (arg1 && !strcmp(arg1, "BATTLE"))       { if (state == ST_STANDBY) action_double(); else cmd_not_allowed(); }
        else if (arg1 && !strcmp(arg1, "STANDBY")) { if (state == ST_BATTLE)  action_double(); else cmd_not_allowed(); }
        else uart_write("[CMD] ERR usage: MODE STANDBY | MODE BATTLE\r\n");
        return;
    }
    if (!strcmp(verb, "CHARGE")) { if (state == ST_BATTLE)   laser_charge(); else cmd_not_allowed(); return; }
    if (!strcmp(verb, "PING"))   { if (state == ST_STANDBY)  action_single(); else cmd_not_allowed(); return; }
    if (!strcmp(verb, "RESCUE")) { if (state == ST_ATTACKED) rescue();       else cmd_not_allowed(); return; }

    /* --- Standby only: configuration (saved to flash) and tools --- */
    bool is_sb = !strcmp(verb, "MUTE") || !strcmp(verb, "HISCORE") || !strcmp(verb, "BLACKBOX");
    if (is_sb && state != ST_STANDBY) { cmd_not_allowed(); return; }

    if (!strcmp(verb, "MUTE")) {
        bool on;
        if (!parse_on_off(verb, arg1, &on)) return;
        cfg.muted = on;
        if (on && !bz_playing(SND_SOS)) tone_set(0);
        cmd_cfg_saved(on ? "MUTE ON" : "MUTE OFF");
        return;
    }
    if (!strcmp(verb, "HISCORE")) {
        if (!arg1 || strcmp(arg1, "RESET")) { uart_write("[CMD] ERR usage: HISCORE RESET\r\n"); return; }
        cfg.hiscore = 0;
        cmd_cfg_saved("HISCORE 0");
        return;
    }
    if (!strcmp(verb, "BLACKBOX")) {
        if (bb_flash_valid) bb_print(bb_flash_image(), "Recovered from flash (drone self-destructed)");
        else                bb_print(&bb, "Last sortie (RAM)");
        return;
    }

    uart_printf("[CMD] ERR unknown command '%s' (type HELP)\r\n", verb);
}

/* The board echoes what is typed (turn OFF "Local echo" in TeraTerm). */
static void uart_poll_commands(void)
{
    while (rx_tail != rx_head) {
        char c = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_BUF_SIZE);

        if (c == '\r' || c == '\n') {
            if (cmd_len > 0) {
                cmd_line[cmd_len] = '\0';
                cmd_len = 0;
                uart_write_raw("\r\n");
                cmd_exec(cmd_line);
                if (state == ST_TERMINATED) return;
            }
        } else if (c == '\b' || c == 0x7F) {
            if (cmd_len > 0) {
                cmd_len--;
                uart_write_raw(cmd_len > 0 ? "\b \b" : "\r\x1b[K");
            }
        } else if (c == 0x1B) {
            if (cmd_len > 0) { cmd_len = 0; uart_write_raw("\r\x1b[K"); }
        } else if (cmd_len < CMD_MAX_LEN - 1U && c >= 32 && c < 127) {
            char echo[2] = { c, '\0' };
            if (cmd_len == 0) uart_write_raw("> ");
            cmd_line[cmd_len++] = c;
            uart_write_raw(echo);
        }
    }
}

/* ============================================================================
 * 16. LOW-LEVEL INIT
 * ==========================================================================*/
static void GPIO_IT_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    g.Pin  = BUTTON_EXTI13_Pin;                 /* both edges: debounce press and release */
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &g);

    g.Pin  = LSM6DSL_INT1_EXTI11_Pin;           /* IMU 6D orientation interrupt */
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &g);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;      /* PB6 TX, PB7 RX (ST-LINK VCP) */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &g);

    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1) { }
    }
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* ============================================================================
 * 17. MAIN
 * ==========================================================================*/
static void app_init(void)
{
    boot_tick = HAL_GetTick();
    BSP_LED_Init(LED2);
    UART1_Init();
    sensors_init();
    uint32_t up = HAL_GetTick();                      /* Grove modules' own MCUs need to boot */
    if (up < MODULE_BOOT_MS) HAL_Delay(MODULE_BOOT_MS - up);
    I2C1_Init();
    if (i2c1_ready && i2c1_bus_stuck()) i2c1_recover();   /* RESET does not reset the modules */
    mx_init();
    oled_init();
    buzzer_init();
    fiveway_probe();
    GPIO_IT_Init();

    uart_printf("\r\n=== Drone #%d online === silent: %s\r\n", DRONE_ID, cfg.muted ? "ON" : "OFF");
    uart_printf("[EVT] System check: sensors %s, LED matrix %s, OLED %s, 5-way %s. Type HELP for commands.\r\n",
                sensors_ok ? "OK" : "FAIL", mx_ok ? "OK" : "NOT FOUND", oled_ok ? "OK" : "NOT FOUND",
                fiveway_addr ? "OK" : "NOT FOUND");

    /* Altitude zero: average of 10 readings */
    float sum = 0.0f;
    for (int i = 0; i < 10; i++) { sum += BSP_PSENSOR_ReadPressure(); HAL_Delay(40); }
    float p0 = sum / 10.0f;
    if (p0 < 300.0f || p0 > 1200.0f) p0 = 1013.25f;
    altitude_zero(p0);
    uart_printf("[EVT] Altitude zero at %.2f hPa\r\n", p0);

    bb_clear();
    bb_flash_check();
    if (bb_flash_valid) {
        const bb_image_t *f = bb_flash_image();
        uart_printf("[EVT] Black box of Drone #%d recovered (%d records). Type BLACKBOX to read\r\n", f->drone_id, f->count);
    }

    /* Seed the map generator from sensor noise so every power-on is a new area. */
    {
        int16_t a[3] = {0}, m[3] = {0};
        BSP_ACCELERO_AccGetXYZ(a);
        BSP_MAGNETO_GetXYZ(m);
        float pr = BSP_PSENSOR_ReadPressure(), t = BSP_TSENSOR_ReadTemp();
        uint32_t pb, tb;
        memcpy(&pb, &pr, 4); memcpy(&tb, &t, 4);
        rng_state ^= pb * 2654435761u ^ tb ^ ((uint32_t)(uint16_t)a[0] << 16) ^ (uint16_t)m[1] ^ ((uint32_t)(uint16_t)a[2] << 7);
        if (rng_state == 0) rng_state = 0x2028EE28u;
    }

    bz_fx(SND(SND_BOOT));
    enter_standby();
}

/* One pass of the super-loop. Nothing in here blocks for long. */
static void app_loop(void)
{
    uint8_t singles, doubles;

    uart_poll_commands();

    /* Normally at most one event per pass. If several piled up, stop at the
     * first mode change so old presses do not act in the new mode. */
    button_collect(&singles, &doubles);
    sys_state_t s0 = state;
    while (singles-- && state == s0) action_single();
    while (doubles-- && state == s0) action_double();

    fiveway_tick();

    /* Modules plugged in after power-on (or after a glitch) are found again. */
    static uint32_t t_reprobe = 0;
    if (state == ST_STANDBY && elapsed(t_reprobe, MODULE_REPROBE_MS)) {
        t_reprobe = HAL_GetTick();
        if (!mx_ok) {
            mx_init();
            if (mx_ok) uart_write("[EVT] LED matrix connected\r\n");
        }
        if (!oled_ok) {
            oled_init();
            if (oled_ok) uart_write("[EVT] OLED connected\r\n");
        }
        if (!fiveway_addr && ENABLE_FIVEWAY) {                     /* newly plugged in */
            fiveway_probe();
            if (fiveway_addr) uart_printf("[EVT] 5-way switch connected at 0x%02X\r\n", fiveway_addr);
        }
    }

    switch (state) {
        case ST_STANDBY:  standby_tick();  break;
        case ST_BATTLE:   battle_tick();   break;
        case ST_ATTACKED: attacked_tick(); break;
        default: break;
    }

    bz_update();
    matrix_render();
    hud_tick();
    oled_flush_step();
}

int main(void)
{
    HAL_Init();
    cfg_load();
    app_init();
    while (1) {
        app_loop();
    }
}
