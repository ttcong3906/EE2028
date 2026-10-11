/******************************************************************************
 * @file    main.c
 * @brief   EE2028 Assignment 2 (AY26/27 S1) - "The Last of EE2028"
 *          Firmware for an autonomous attacking drone on the B-L4S5I-IOT01A.
 *
 * Behaviour is specified in docs/SPEC.md. Summary:
 *   STANDBY  : 6 sensors read every 1 s, telemetry or threshold warnings over
 *              UART, LED2 solid. Single press toggles SOS beacon.
 *   BATTLE   : LED2 1 Hz. Single press charges laser +3/10, auto-fire at
 *              >= 5/10 (-5/10). Flip (chip side down) -> ATTACKED.
 *   ATTACKED : "The Last of EE2028". LED2 2 Hz, "Drone Was Attacked!" every
 *              1 s, double press within 10 s rescues, else TERMINATED.
 *   TERMINATED: last message, comms cut, MCU enters Standby (low power).
 *
 * Add-ons: 2-way UART command interface, flip detection by LSM6DSL 6D
 * interrupt, SSD1306 OLED HUD, HT16K33 8x8 LED matrix, PWM buzzer, SOS.
 *
 * Design rules:
 *   - Super-loop, fully non-blocking, all timing from HAL_GetTick().
 *   - ISRs only record timestamps / flags / bytes. No I2C, no blocking UART,
 *     no HAL_Delay inside interrupts.
 ******************************************************************************/

/* ============================================================================
 * 0. INCLUDES
 * ==========================================================================*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
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

/* --- Feature switches: set to 0 to test the core without external parts --- */
#define ENABLE_OLED             1
#define ENABLE_MATRIX           1
#define ENABLE_BUZZER           1
#define TERMINATE_USE_STANDBY   1   /* 1: MCU Standby mode, 0: sleep loop     */

/* --- Orientation: +1 means "chip side up" is normal flight.
 *     Set to -1 to follow the literal spec text (Z+ pointing to ground).    */
#define FLIP_SIGN               (+1)

/* --- External display quirks (change if the picture looks wrong) --- */
#define OLED_I2C_ADDR           0x3C
#define OLED_COL_OFFSET         0   /* 2 for SH1106-based 1.3" modules        */
#define MATRIX_I2C_ADDR         0x70
#define MATRIX_FLIP_X           0   /* 1 if letters on the matrix are mirrored */

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
#define SOS_BLINK_MS            500U
#define RESCUE_SMILE_MS         1000U
#define FIRE_ANIM_FRAME_MS      60U

/* --- Physics --- */
#define G_MS2                   9.80665f
#define FLIP_G_FRACTION         0.5f      /* |Az| > 0.5 g in the wrong sign  */

/* --- Laser (units of 1/10 capacity) --- */
#define LASER_CHARGE_STEP       3
#define LASER_FIRE_COST         5
#define LASER_MAX               10

/* ============================================================================
 * 2. TYPES AND GLOBAL STATE
 * ==========================================================================*/
typedef enum { ST_STANDBY = 0, ST_BATTLE, ST_ATTACKED, ST_TERMINATED } sys_state_t;

enum { S_T = 0, S_P, S_H, S_A, S_G, S_M, S_COUNT };

typedef struct {
    const char *tag;      /* "T", "P", ...          */
    const char *unit;     /* unit printed on UART   */
    float low, high;
    bool  use_low, use_high;
} thresh_t;

/* Thresholds: see docs/SPEC.md section 7 for the physical reasoning. */
static thresh_t th[S_COUNT] = {
    /* tag  unit      low     high    low?   high? */
    { "T", "C",      10.0f,   35.0f,  true,  true  }, /* Li-ion safe range   */
    { "P", "hPa",   950.0f, 1040.0f,  true,  true  }, /* ~540 m ceiling     */
    { "H", "%RH",    20.0f,   80.0f,  true,  true  }, /* static / condensation */
    { "A", "m/s^2",   3.0f,   15.0f,  true,  true  }, /* free fall / impact  */
    { "G", "dps",     0.0f,  300.0f,  false, true  }, /* spin out of control */
    { "M", "uT",     10.0f,  100.0f,  true,  true  }, /* magnetic interference */
};

typedef struct {
    float v[S_COUNT];     /* magnitudes in physical units */
    float ax, ay, az;     /* m/s^2 */
} sensors_t;

static sys_state_t state = ST_STANDBY;
static sensors_t   sens;               /* latest reading                    */
static bool        sens_valid = false;
static float       p0_hpa = 1013.25f;  /* ground pressure at boot           */

static uint32_t t_sensor = 0, t_led = 0, t_flip_poll = 0, t_attack_start = 0;
static uint32_t t_attack_msg = 0, t_sos_blink = 0;
static bool     led_on = false;

static int      laser_energy = 0;      /* 0..10 */
static uint32_t shots_fired  = 0;
static bool     sos_active   = false;
static bool     sos_line_on  = false;
static bool     muted        = false;

/* Flip detection */
static bool     flip_armed = true;
static bool     flip_candidate = false;
static uint32_t flip_since = 0;

/* Peripherals */
UART_HandleTypeDef huart1;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim3;

static inline bool elapsed(uint32_t since, uint32_t ms)
{
    return (uint32_t)(HAL_GetTick() - since) >= ms;
}

/* ============================================================================
 * 3. UART: non-blocking TX ring buffer + interrupt RX ring buffer
 * ==========================================================================*/
#define TX_BUF_SIZE 2048U
#define RX_BUF_SIZE 128U
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

/* Start the next DMA-less IT transfer. Call with IRQs disabled or from ISR. */
static void uart_kick(void)
{
    if (tx_busy || tx_head == tx_tail) return;
    uint16_t len = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail)
                                       : (uint16_t)(TX_BUF_SIZE - tx_tail);
    tx_inflight = len;
    tx_busy = true;
    if (HAL_UART_Transmit_IT(&huart1, &tx_buf[tx_tail], len) != HAL_OK) {
        tx_busy = false;
    }
}

static void uart_write(const char *s)
{
    if (!uart_enabled) return;
    __disable_irq();
    while (*s) {
        uint16_t next = (uint16_t)((tx_head + 1U) % TX_BUF_SIZE);
        if (next == tx_tail) break;          /* buffer full: drop the rest */
        tx_buf[tx_head] = (uint8_t)*s++;
        tx_head = next;
    }
    uart_kick();
    __enable_irq();
}

static void uart_printf(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_write(buf);
}

/* Wait until everything queued has left the UART (used before shutdown). */
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
    /* Overrun / framing error aborts reception in HAL: re-arm it. */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* ============================================================================
 * 4. I2C1 BUS FOR EXTERNAL MODULES (OLED + LED MATRIX)
 * ==========================================================================*/
static bool i2c1_ready = false;

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
    hi2c1.Init.Timing           = 0x10320309;   /* same as last year, 4 MHz PCLK */
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    i2c1_ready = (HAL_I2C_Init(&hi2c1) == HAL_OK);
}

static bool i2c1_present(uint8_t addr7)
{
    if (!i2c1_ready) return false;
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 2, 10) == HAL_OK;
}

/* ============================================================================
 * 5. OLED SSD1306 128x64 - framebuffer, one dirty page flushed per loop pass
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
static bool    oled_ok = false;
static uint8_t oled_fb[OLED_LINES][128];
static char    oled_text_cache[OLED_LINES][OLED_COLS + 1];
static uint8_t oled_dirty = 0;           /* bit n = page n needs flushing */

static void oled_cmd(uint8_t c)
{
    uint8_t d[2] = { 0x00, c };
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR << 1, d, 2, 10);
}

static void oled_init(void)
{
#if ENABLE_OLED
    oled_ok = i2c1_present(OLED_I2C_ADDR);
    if (!oled_ok) return;
    static const uint8_t seq[] = {
        0xAE,             /* display off                    */
        0xD5, 0x80,       /* clock divide                   */
        0xA8, 0x3F,       /* multiplex 64                   */
        0xD3, 0x00,       /* display offset 0               */
        0x40,             /* start line 0                   */
        0x8D, 0x14,       /* charge pump on                 */
        0x20, 0x02,       /* page addressing mode           */
        0xA1, 0xC8,       /* segment remap, COM scan dec    */
        0xDA, 0x12,       /* COM pins config                */
        0x81, 0x7F,       /* contrast                       */
        0xD9, 0xF1,       /* pre-charge                     */
        0xDB, 0x40,       /* VCOMH                          */
        0xA4, 0xA6,       /* resume RAM, normal (not inverted) */
        0xAF              /* display on                     */
    };
    for (unsigned i = 0; i < sizeof(seq); i++) oled_cmd(seq[i]);
    memset(oled_fb, 0, sizeof(oled_fb));
    memset(oled_text_cache, 0, sizeof(oled_text_cache));
    oled_dirty = 0xFF;
#endif
}

/* Write one text line (21 chars, padded). Only marks the page dirty if the
 * text actually changed, so calling this every loop is cheap. */
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
        const uint8_t *glyph = font5x7[c - 32];
        memcpy(&row[1 + i * 6], glyph, 5);
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

static void oled_clear_lines(void)
{
    for (uint8_t i = 0; i < OLED_LINES; i++) oled_line(i, "");
}

static void oled_flush_page(uint8_t p)
{
    uint8_t col = OLED_COL_OFFSET;
    oled_cmd((uint8_t)(0xB0 + p));
    oled_cmd((uint8_t)(0x00 | (col & 0x0F)));
    oled_cmd((uint8_t)(0x10 | (col >> 4)));
    uint8_t buf[129];
    buf[0] = 0x40;
    memcpy(&buf[1], oled_fb[p], 128);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR << 1, buf, sizeof(buf), 30);
}

/* Push at most one dirty page per call to keep the main loop responsive. */
static void oled_flush_step(void)
{
#if ENABLE_OLED
    if (!oled_ok || oled_dirty == 0) return;
    for (uint8_t p = 0; p < OLED_LINES; p++) {
        if (oled_dirty & (1U << p)) {
            oled_flush_page(p);
            oled_dirty &= (uint8_t)~(1U << p);
            return;
        }
    }
#endif
}

static void oled_flush_all(void)
{
#if ENABLE_OLED
    while (oled_ok && oled_dirty) oled_flush_step();
#endif
}

/* ============================================================================
 * 6. LED MATRIX 8x8 (HT16K33). Glyph rows: bit7 = leftmost column.
 * ==========================================================================*/
static bool    mx_ok = false;
static uint8_t mx_shown[8];
static bool    mx_shown_valid = false;

static const uint8_t GL_RADAR[8]   = {0x3C,0x42,0x81,0x99,0x99,0x81,0x42,0x3C};
static const uint8_t GL_EXCLAIM[8] = {0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18};
static const uint8_t GL_SMILE[8]   = {0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C};
static const uint8_t GL_CROSS[8]   = {0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81};
static const uint8_t GL_S[8]       = {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00};
static const uint8_t GL_O[8]       = {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00};
static const uint8_t GL_BLANK[8]   = {0};
static const uint8_t GL_FIRE[5][8] = {          /* laser beam sweeping right */
    {0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00},
    {0x00,0x00,0x00,0xF0,0xF0,0x00,0x00,0x00},
    {0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x3F,0x3F,0x18,0x00,0x00},
    {0x00,0x24,0x18,0x0F,0x0F,0x18,0x24,0x00},  /* impact burst */
};

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
    mx_ok = i2c1_present(MATRIX_I2C_ADDR);
    if (!mx_ok) return;
    uint8_t c;
    c = 0x21; HAL_I2C_Master_Transmit(&hi2c1, MATRIX_I2C_ADDR << 1, &c, 1, 10); /* osc on   */
    c = 0xE8; HAL_I2C_Master_Transmit(&hi2c1, MATRIX_I2C_ADDR << 1, &c, 1, 10); /* bright 8 */
    c = 0x81; HAL_I2C_Master_Transmit(&hi2c1, MATRIX_I2C_ADDR << 1, &c, 1, 10); /* disp on  */
    mx_shown_valid = false;
#endif
}

/* Send a frame only if it differs from what is on the matrix already. */
static void mx_show(const uint8_t rows[8])
{
#if ENABLE_MATRIX
    if (!mx_ok) return;
    if (mx_shown_valid && memcmp(rows, mx_shown, 8) == 0) return;
    uint8_t buf[17];
    buf[0] = 0x00;
    for (int r = 0; r < 8; r++) {
        buf[1 + 2 * r] = mx_map(rows[r]);
        buf[2 + 2 * r] = 0x00;
    }
    HAL_I2C_Master_Transmit(&hi2c1, MATRIX_I2C_ADDR << 1, buf, sizeof(buf), 20);
    memcpy(mx_shown, rows, 8);
    mx_shown_valid = true;
#else
    (void)rows;
#endif
}

/* ============================================================================
 * 7. BUZZER (passive, PB1 = TIM3_CH4 PWM) with a non-blocking sequencer
 * ==========================================================================*/
typedef struct { uint16_t f0, f1, ms; } note_t;   /* f0->f1 sweep, 0 = rest */

#define DOT  100
#define DASH 300
/* SOS in Morse. Each letter is exactly 6 entries so letter = idx / 6. */
static const note_t SND_SOS[] = {
    {1000,1000,DOT},{0,0,DOT},{1000,1000,DOT},{0,0,DOT},{1000,1000,DOT},{0,0,300},  /* S */
    {1000,1000,DASH},{0,0,DOT},{1000,1000,DASH},{0,0,DOT},{1000,1000,DASH},{0,0,300},/* O */
    {1000,1000,DOT},{0,0,DOT},{1000,1000,DOT},{0,0,DOT},{1000,1000,DOT},{0,0,700},  /* S */
};
static const note_t SND_CHARGE[]  = { {1000,1000,50} };
static const note_t SND_FIRE[]    = { {1000,1000,50},{0,0,40},{2000,500,200} };
static const note_t SND_MODE[]    = { {2000,2000,60},{0,0,60},{2000,2000,60} };
static const note_t SND_ALARM[]   = { {1500,1500,125} };
static const note_t SND_RESCUE[]  = { {1047,1047,150},{1319,1319,150},{1568,1568,250} };
static const note_t SND_GAMEOVER[]= { {1568,1568,300},{0,0,50},{1319,1319,300},{0,0,50},
                                      {1047,1047,300},{0,0,50},{784,784,700} };
#define SND(x) (x), (uint8_t)(sizeof(x) / sizeof((x)[0]))

static const note_t *bz_seq = NULL;
static uint8_t  bz_len = 0, bz_idx = 0;
static bool     bz_loop = false;
static uint32_t bz_note_t0 = 0;
static uint16_t bz_cur_freq = 0;

static void tone_set(uint16_t f)
{
#if ENABLE_BUZZER
    if (muted) f = 0;
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
    g.Pin       = GPIO_PIN_1;                 /* PB1 = Arduino D6 */
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
    bz_cur_freq = 1;      /* force the first tone_set(0) to write */
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
    if (n->f0 != n->f1 && n->f0 != 0) {              /* linear sweep */
        int32_t f = (int32_t)n->f0 + ((int32_t)n->f1 - (int32_t)n->f0) * (int32_t)t / (int32_t)n->ms;
        tone_set((uint16_t)f);
    }
}

/* ============================================================================
 * 8. SENSORS
 * ==========================================================================*/
#define LSM6DSL_ADDR         0xD4   /* LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW */
#define LSM6DSL_REG_D6D_SRC  0x1D
#define LSM6DSL_REG_TAP_CFG  0x58
#define LSM6DSL_REG_TAP_THS  0x59
#define LSM6DSL_REG_MD1_CFG  0x5E

static volatile bool flip_irq = false;

static void sensors_init(void)
{
    BSP_TSENSOR_Init();
    BSP_HSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_MAGNETO_Init();

    /* Hardware orientation (6D) detection inside the LSM6DSL IMU:
     * 60 deg threshold, routed to INT1 (PD11, EXTI11). */
    uint8_t v;
    v = SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_TAP_THS);
    v = (uint8_t)((v & ~0xE0U) | (0x2U << 5));            /* SIXD_THS = 60 deg, D4D off */
    SENSOR_IO_Write(LSM6DSL_ADDR, LSM6DSL_REG_TAP_THS, v);
    v = SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_TAP_CFG);
    SENSOR_IO_Write(LSM6DSL_ADDR, LSM6DSL_REG_TAP_CFG, (uint8_t)(v | 0x80U)); /* INTERRUPTS_ENABLE */
    v = SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_MD1_CFG);
    SENSOR_IO_Write(LSM6DSL_ADDR, LSM6DSL_REG_MD1_CFG, (uint8_t)(v | 0x04U)); /* INT1_6D */
    (void)SENSOR_IO_Read(LSM6DSL_ADDR, LSM6DSL_REG_D6D_SRC);                 /* clear */
}

static float accel_z_ms2(void)
{
    int16_t a[3] = {0};
    BSP_ACCELERO_AccGetXYZ(a);
    return (float)a[2] * G_MS2 / 1000.0f;
}

static void sensors_read(sensors_t *s)
{
    int16_t a[3] = {0}, m[3] = {0};
    float   g[3] = {0};

    s->v[S_T] = BSP_TSENSOR_ReadTemp();        /* deg C */
    s->v[S_P] = BSP_PSENSOR_ReadPressure();    /* hPa   */
    s->v[S_H] = BSP_HSENSOR_ReadHumidity();    /* %RH   */

    BSP_ACCELERO_AccGetXYZ(a);                 /* mg    */
    s->ax = (float)a[0] * G_MS2 / 1000.0f;
    s->ay = (float)a[1] * G_MS2 / 1000.0f;
    s->az = (float)a[2] * G_MS2 / 1000.0f;
    s->v[S_A] = sqrtf(s->ax * s->ax + s->ay * s->ay + s->az * s->az);

    BSP_GYRO_GetXYZ(g);                        /* mdps (sensitivity already applied) */
    float gx = g[0] / 1000.0f, gy = g[1] / 1000.0f, gz = g[2] / 1000.0f;
    s->v[S_G] = sqrtf(gx * gx + gy * gy + gz * gz);

    BSP_MAGNETO_GetXYZ(m);                     /* mGauss; 1 mG = 0.1 uT */
    float mx = (float)m[0] / 10.0f, my = (float)m[1] / 10.0f, mz = (float)m[2] / 10.0f;
    s->v[S_M] = sqrtf(mx * mx + my * my + mz * mz);
}

static float altitude_m(float p_hpa)
{
    /* International barometric formula, relative to the pressure at boot. */
    return 44330.0f * (1.0f - powf(p_hpa / p0_hpa, 0.1903f));
}

/* Returns a bitmask of violated sensors; bit (2*i) = low, bit (2*i+1) = high */
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
 * 9. BUTTON (PC13, EXTI13 both edges) AND IMU INT1 (PD11, EXTI11)
 * ==========================================================================*/
#define BTN_PRESSED_LEVEL GPIO_PIN_RESET          /* B1 is active-low */

static volatile bool     btn_down = false;
static volatile uint32_t btn_press_t = 0, btn_release_t = 0;
static volatile uint8_t  btn_pending = 0;         /* 0 none, 1 waiting for 2nd */
static volatile uint32_t btn_first_t = 0;
static volatile uint8_t  btn_singles = 0, btn_doubles = 0;

static void button_on_press(uint32_t now)
{
    if (btn_pending && (now - btn_first_t) < DOUBLE_PRESS_MS) {
        btn_pending = 0;
        btn_doubles++;
    } else {
        if (btn_pending) btn_singles++;           /* stale first press was a single */
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
            /* Accept only a clean press: released long enough, and not a bounce
             * of the previous press. */
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

/* Called from the main loop: turn the ISR state into single/double events. */
static void button_collect(uint8_t *singles, uint8_t *doubles)
{
    __disable_irq();
    if (btn_pending && elapsed(btn_first_t, DOUBLE_PRESS_MS)) {
        btn_pending = 0;
        btn_singles++;
    }
    *singles = btn_singles;  btn_singles = 0;
    *doubles = btn_doubles;  btn_doubles = 0;
    __enable_irq();
}

/* ============================================================================
 * 10. DISPLAY CONTENT (OLED HUD + LED MATRIX) PER STATE
 * ==========================================================================*/
static uint32_t warn_mask = 0;
static uint32_t fire_anim_t0 = 0;  static bool fire_anim = false;
static uint32_t smile_t0 = 0;      static bool smile = false;

static const char *mode_name(sys_state_t s)
{
    switch (s) {
        case ST_STANDBY:  return "STANDBY";
        case ST_BATTLE:   return "BATTLE";
        case ST_ATTACKED: return "ATTACKED";
        default:          return "TERMINATED";
    }
}

static void hud_standby(void)
{
    oled_line(0, "== STANDBY MODE ==");
    if (sens_valid) {
        oled_linef(1, "T %4.1fC  H %4.1f%%", sens.v[S_T], sens.v[S_H]);
        oled_linef(2, "P %6.1fhPa", sens.v[S_P]);
        oled_linef(3, "ALT %+6.1f m", altitude_m(sens.v[S_P]));
        oled_linef(4, "A %5.2f m/s2", sens.v[S_A]);
        oled_linef(5, "G %6.2f dps", sens.v[S_G]);
        oled_linef(6, "M %5.1f uT", sens.v[S_M]);
    }
    if (sos_active) {
        oled_line(7, sos_line_on ? "*** SOS ACTIVE ***" : "");
    } else if (warn_mask) {
        char buf[40] = "WARN:";
        for (int i = 0; i < S_COUNT; i++) {
            if (warn_mask & (1UL << (2 * i)))     { strcat(buf, " "); strcat(buf, th[i].tag); strcat(buf, "LO"); }
            if (warn_mask & (1UL << (2 * i + 1))) { strcat(buf, " "); strcat(buf, th[i].tag); strcat(buf, "HI"); }
        }
        oled_line(7, buf);
    } else {
        oled_line(7, "STATUS: ALL NORMAL");
    }
}

static void hud_battle(void)
{
    char bar[11];
    for (int i = 0; i < 10; i++) bar[i] = (i < laser_energy) ? '#' : '-';
    bar[10] = '\0';
    oled_line(0, "=== BATTLE MODE ===");
    oled_line(1, "");
    oled_line(2, "LASER");
    oled_linef(3, "[%s] %2d/10", bar, laser_energy);
    oled_line(4, "");
    oled_linef(5, "SHOTS FIRED: %lu", (unsigned long)shots_fired);
    oled_line(6, "");
    oled_line(7, "1x CHARGE 2x STANDBY");
}

static int rescue_secs_left(void)
{
    uint32_t t = HAL_GetTick() - t_attack_start;
    if (t >= RESCUE_WINDOW_MS) return 0;
    return (int)((RESCUE_WINDOW_MS - t + 999U) / 1000U);
}

static void hud_attacked(void)
{
    oled_line(0, "!! UNDER ATTACK !!");
    oled_line(1, "");
    oled_line(2, "  RESCUE WINDOW");
    oled_linef(3, "       %2d s", rescue_secs_left());
    oled_line(4, "");
    oled_line(5, "DOUBLE-PRESS TO");
    oled_line(6, "      RESCUE");
    oled_line(7, "");
}

static void hud_terminated(void)
{
    oled_line(0, "");
    oled_line(1, "    DRONE  LOST");
    oled_line(2, "");
    oled_line(3, "  COMMS  OFFLINE");
    oled_line(4, "");
    oled_line(5, "  press RESET to");
    oled_line(6, "     reboot");
    oled_line(7, "");
}

static void matrix_render(void)
{
    uint8_t rows[8];

    if (state == ST_TERMINATED) { mx_show(GL_CROSS); return; }

    if (smile) {
        if (elapsed(smile_t0, RESCUE_SMILE_MS)) smile = false;
        else { mx_show(GL_SMILE); return; }
    }

    switch (state) {
    case ST_STANDBY:
        if (sos_active && bz_playing(SND_SOS)) {
            mx_show((bz_idx / 6U) == 1U ? GL_O : GL_S);
        } else if (sos_active) {
            mx_show(GL_S);                    /* muted or between loops */
        } else {
            mx_show(warn_mask ? GL_EXCLAIM : GL_RADAR);
        }
        break;

    case ST_BATTLE:
        if (fire_anim) {
            uint32_t f = (HAL_GetTick() - fire_anim_t0) / FIRE_ANIM_FRAME_MS;
            if (f < 5) { mx_show(GL_FIRE[f]); break; }
            fire_anim = false;
        }
        /* Energy bar: one column per 1/10 (0..7 reachable), marker at col 5. */
        memset(rows, 0, sizeof(rows));
        rows[0] = 0x80U >> (LASER_FIRE_COST - 1);
        for (int r = 2; r < 8; r++) {
            uint8_t b = 0;
            for (int c = 0; c < laser_energy && c < 8; c++) b |= (uint8_t)(0x80U >> c);
            rows[r] = b;
        }
        mx_show(rows);
        break;

    case ST_ATTACKED:
        mx_show(led_on ? GL_EXCLAIM : GL_BLANK);
        break;

    default:
        break;
    }
}

/* ============================================================================
 * 11. STATE MACHINE
 * ==========================================================================*/
static void led_set(bool on)
{
    led_on = on;
    if (on) BSP_LED_On(LED2); else BSP_LED_Off(LED2);
}

static void send_stat(void)
{
    float alt = 0.0f;
    if (state != ST_TERMINATED) alt = altitude_m(BSP_PSENSOR_ReadPressure());
    if (state == ST_ATTACKED) {
        uart_printf("[STAT] MODE: %s, LASER: %d/10, SHOTS: %lu, ALT: %+.1f m, RESCUE: %d s\r\n",
                    mode_name(state), laser_energy, (unsigned long)shots_fired, alt, rescue_secs_left());
    } else {
        uart_printf("[STAT] MODE: %s, LASER: %d/10, SHOTS: %lu, ALT: %+.1f m, SOS: %s, MUTE: %s\r\n",
                    mode_name(state), laser_energy, (unsigned long)shots_fired, alt,
                    sos_active ? "ON" : "OFF", muted ? "ON" : "OFF");
    }
}

static void sos_set(bool on)
{
    if (on == sos_active) return;
    sos_active = on;
    if (on) {
        uart_write("[EVT] SOS ON - distress beacon active\r\n");
        sos_line_on = true;
        t_sos_blink = HAL_GetTick();
        bz_play(SND(SND_SOS), true);
    } else {
        uart_write("[EVT] SOS OFF\r\n");
        if (bz_playing(SND_SOS)) bz_stop();
    }
}

static void enter_standby(void)
{
    state = ST_STANDBY;
    uart_write("Entering STANDBY MODE\r\n");
    led_set(true);
    warn_mask = 0;
    t_sensor = HAL_GetTick() - SENSOR_PERIOD_MS;   /* report immediately */
    oled_clear_lines();
    hud_standby();
}

static void enter_battle(bool from_rescue)
{
    if (sos_active) sos_set(false);
    state = ST_BATTLE;
    if (!from_rescue) {
        uart_write("Entering BATTLE MODE\r\n");
        flip_armed = true;            /* fresh entry: a flipped drone is attacked */
        bz_play(SND(SND_MODE), false);
    }
    flip_candidate = false;
    led_set(true);
    t_led = HAL_GetTick();
    t_flip_poll = HAL_GetTick();
    oled_clear_lines();
    hud_battle();
    send_stat();
}

static void enter_attacked(void)
{
    state = ST_ATTACKED;
    t_attack_start = HAL_GetTick();
    t_attack_msg = t_attack_start - ATTACK_MSG_MS;  /* first message now */
    fire_anim = false;
    led_set(true);
    t_led = t_attack_start;
    bz_play(SND(SND_ALARM), false);
    uart_write("[EVT] Drone flipped! Entering The Last of EE2028\r\n");
    oled_clear_lines();
    hud_attacked();
}

static void rescue(void)
{
    uart_write("[EVT] Rescued! Returning to BATTLE MODE\r\n");
    flip_armed = false;   /* must be turned upright before it can be attacked again */
    smile = true;
    smile_t0 = HAL_GetTick();
    bz_play(SND(SND_RESCUE), false);
    enter_battle(true);
}

static void terminate(void)
{
    state = ST_TERMINATED;
    uart_write("[EVT] No rescue. Drone self-destructing. Cutting all communication. Goodbye.\r\n");
    led_set(false);
    oled_clear_lines();
    hud_terminated();
    oled_flush_all();
    matrix_render();
    bz_play(SND(SND_GAMEOVER), false);

    /* Let the last message and the melody finish (non-blocking parts still run). */
    uint32_t t0 = HAL_GetTick();
    while (bz_playing(NULL) && !elapsed(t0, 3000)) bz_update();
    uart_flush(500);

    /* Cut all communication */
    uart_enabled = false;
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_DeInit(&huart1);
    tone_set(0);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    BSP_LED_Off(LED2);

#if TERMINATE_USE_STANDBY
    /* Standby: lowest power, all clocks off, only RESET brings it back.
     * OLED / LED matrix keep the last picture (they have their own RAM). */
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

/* --- Actions shared by the push button and the UART commands --- */
static void action_single(void)
{
    switch (state) {
    case ST_STANDBY:
        sos_set(!sos_active);
        break;

    case ST_BATTLE: {
        int before = laser_energy;
        laser_energy += LASER_CHARGE_STEP;
        if (laser_energy > LASER_MAX) laser_energy = LASER_MAX;
        uart_printf("[EVT] LASER charged: %d/10 -> %d/10\r\n", before, laser_energy);
        if (laser_energy >= LASER_FIRE_COST) {
            int b2 = laser_energy;
            laser_energy -= LASER_FIRE_COST;
            shots_fired++;
            uart_printf("[EVT] LASER FIRED! Zombie down. Energy %d/10 -> %d/10\r\n", b2, laser_energy);
            fire_anim = true;
            fire_anim_t0 = HAL_GetTick();
            bz_play(SND(SND_FIRE), false);
        } else {
            bz_play(SND(SND_CHARGE), false);
        }
        hud_battle();
        break;
    }

    case ST_ATTACKED:   /* single press is ignored while under attack */
    default:
        break;
    }
}

static void action_double(void)
{
    switch (state) {
    case ST_STANDBY:  enter_battle(false); break;
    case ST_BATTLE:   bz_play(SND(SND_MODE), false); enter_standby(); break;
    case ST_ATTACKED: rescue(); break;
    default: break;
    }
}

/* --- Per-state periodic work --- */
static void standby_tick(void)
{
    if (elapsed(t_sensor, SENSOR_PERIOD_MS)) {
        t_sensor += SENSOR_PERIOD_MS;
        if (elapsed(t_sensor, SENSOR_PERIOD_MS)) t_sensor = HAL_GetTick(); /* catch up */

        sensors_read(&sens);
        sens_valid = true;
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
        if (sos_active) uart_write("[SOS] Distress signal. Requesting extraction.\r\n");
        hud_standby();
    }

    if (sos_active && elapsed(t_sos_blink, SOS_BLINK_MS)) {
        t_sos_blink = HAL_GetTick();
        sos_line_on = !sos_line_on;
        hud_standby();
    }
}

static void battle_tick(void)
{
    if (elapsed(t_led, LED_BATTLE_TOGGLE_MS)) {
        t_led += LED_BATTLE_TOGGLE_MS;
        if (elapsed(t_led, LED_BATTLE_TOGGLE_MS)) t_led = HAL_GetTick();
        led_set(!led_on);
    }

    /* Flip detection: the IMU interrupt gives an instant sample, polling is
     * the safety net. Upside down must persist FLIP_HOLD_MS to count. */
    if (flip_irq || elapsed(t_flip_poll, FLIP_POLL_MS)) {
        flip_irq = false;
        t_flip_poll = HAL_GetTick();
        float g_up = (float)FLIP_SIGN * accel_z_ms2();   /* > 0 when upright */
        float lim  = FLIP_G_FRACTION * G_MS2;

        if (!flip_armed) {
            if (g_up > lim) flip_armed = true;
            flip_candidate = false;
        } else if (g_up < -lim) {
            if (!flip_candidate) {
                flip_candidate = true;
                flip_since = HAL_GetTick();
            } else if (elapsed(flip_since, FLIP_HOLD_MS)) {
                enter_attacked();
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
        if (led_on && !bz_playing(SND_RESCUE)) bz_play(SND(SND_ALARM), false);
    }
    if (elapsed(t_attack_msg, ATTACK_MSG_MS)) {
        t_attack_msg += ATTACK_MSG_MS;
        uart_printf("[ATTACK] Drone Was Attacked! Rescue in %d s\r\n", rescue_secs_left());
        hud_attacked();
    }
}

/* ============================================================================
 * 12. UART COMMAND INTERFACE
 * ==========================================================================*/
static void cmd_help(void)
{
    uart_write("[CMD] Commands:\r\n"
               "[CMD]   HELP | STATUS | THRESH\r\n"
               "[CMD]   SET <T|P|H|A|G|M>_<LOW|HIGH> <value>   e.g. SET T_HIGH 30\r\n"
               "[CMD]   MODE STANDBY | MODE BATTLE\r\n"
               "[CMD]   CHARGE (battle) | RESCUE (under attack)\r\n"
               "[CMD]   SOS ON|OFF (standby) | MUTE ON|OFF\r\n");
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
        char *us = strchr(arg1, '_');
        char *end = NULL;
        float val = strtof(arg2, &end);
        if (!us || end == arg2) { uart_write("[CMD] ERR usage: SET T_HIGH 30\r\n"); return; }
        *us = '\0';
        for (int i = 0; i < S_COUNT; i++) {
            if (!strcmp(arg1, th[i].tag)) {
                if (!strcmp(us + 1, "LOW"))  { th[i].low  = val; th[i].use_low  = true; }
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
        if (arg1 && !strcmp(arg1, "BATTLE")) {
            if (state == ST_STANDBY) action_double(); else cmd_not_allowed();
        } else if (arg1 && !strcmp(arg1, "STANDBY")) {
            if (state == ST_BATTLE) action_double(); else cmd_not_allowed();
        } else {
            uart_write("[CMD] ERR usage: MODE STANDBY | MODE BATTLE\r\n");
        }
        return;
    }
    if (!strcmp(verb, "CHARGE")) { if (state == ST_BATTLE)   action_single(); else cmd_not_allowed(); return; }
    if (!strcmp(verb, "RESCUE")) { if (state == ST_ATTACKED) rescue();        else cmd_not_allowed(); return; }

    if (!strcmp(verb, "SOS")) {
        if (state != ST_STANDBY) { cmd_not_allowed(); return; }
        if (arg1 && !strcmp(arg1, "ON"))       sos_set(true);
        else if (arg1 && !strcmp(arg1, "OFF")) sos_set(false);
        else uart_write("[CMD] ERR usage: SOS ON | SOS OFF\r\n");
        return;
    }
    if (!strcmp(verb, "MUTE")) {
        if (arg1 && !strcmp(arg1, "ON"))       { muted = true;  tone_set(0); uart_write("[CMD] OK muted\r\n"); }
        else if (arg1 && !strcmp(arg1, "OFF")) { muted = false; uart_write("[CMD] OK sound on\r\n"); }
        else uart_write("[CMD] ERR usage: MUTE ON | MUTE OFF\r\n");
        return;
    }

    uart_printf("[CMD] ERR unknown command '%s' (type HELP)\r\n", verb);
}

static void uart_poll_commands(void)
{
    while (rx_tail != rx_head) {
        char c = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_BUF_SIZE);

        if (c == '\r' || c == '\n') {
            if (cmd_len > 0) {
                cmd_line[cmd_len] = '\0';
                cmd_len = 0;
                cmd_exec(cmd_line);
                if (state == ST_TERMINATED) return;
            }
        } else if (c == '\b' || c == 0x7F) {
            if (cmd_len > 0) cmd_len--;
        } else if (cmd_len < CMD_MAX_LEN - 1U && c >= 32 && c < 127) {
            cmd_line[cmd_len++] = c;
        }
    }
}

/* ============================================================================
 * 13. LOW-LEVEL INIT
 * ==========================================================================*/
static void GPIO_IT_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    /* User button PC13: both edges so press and release can be debounced */
    g.Pin  = BUTTON_EXTI13_Pin;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &g);

    /* LSM6DSL INT1 PD11: active-high pulse on orientation change */
    g.Pin  = LSM6DSL_INT1_EXTI11_Pin;
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
 * 14. MAIN
 * ==========================================================================*/
static void app_init(void)
{
    BSP_LED_Init(LED2);
    UART1_Init();
    sensors_init();
    I2C1_Init();
    oled_init();
    mx_init();
    buzzer_init();
    GPIO_IT_Init();

    uart_write("\r\n=== EE2028 Attacking Drone online ===\r\n");
    uart_printf("[EVT] OLED %s, LED matrix %s. Type HELP for commands.\r\n",
                oled_ok ? "found" : "not found", mx_ok ? "found" : "not found");

    p0_hpa = BSP_PSENSOR_ReadPressure();
    if (p0_hpa < 300.0f || p0_hpa > 1200.0f) p0_hpa = 1013.25f;

    enter_standby();
}

/* One pass of the super-loop. Nothing in here blocks. */
static void app_loop(void)
{
    uint8_t singles, doubles;

    uart_poll_commands();

    button_collect(&singles, &doubles);
    while (singles--) action_single();
    while (doubles--) action_double();

    switch (state) {
        case ST_STANDBY:  standby_tick();  break;
        case ST_BATTLE:   battle_tick();   break;
        case ST_ATTACKED: attacked_tick(); break;
        default: break;
    }

    bz_update();
    matrix_render();
    oled_flush_step();
}

int main(void)
{
    HAL_Init();
    app_init();
    while (1) {
        app_loop();
    }
}
