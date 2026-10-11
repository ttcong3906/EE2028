#include "main.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stdio.h"

#include "stdbool.h"
#include "stdlib.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio_ex.h"
#include "stm32l4xx_hal_i2c.h"

/*Include the nescesary sensor */
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"

#define OLED_I2C_ADDR 0x3C     // OLED address
#define LED_MATRIX_I2C_ADDR 0x70  // Default HT16K33 I2C address
static void I2C1_GPIO_Init(void);
static void OLED_Init(void);
static void Buzzer_Init(void);

static inline uint8_t elapsed(uint32_t since, uint32_t ms) {  // check time elapsed
    return (uint32_t)(HAL_GetTick() - since) >= ms;
}

///image for OLED
// 'project photo (1)', 128x64px
const uint8_t custom_image[] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x80,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x00,0x40,0x40,
0x40,0x00,0x00,0x80,0x84,0x04,0x04,0x00,0x08,0x08,0x00,0x10,0x20,0x00,0x40,0x82,
0x82,0x00,0x00,0x00,0x00,0x00,0xF8,0x02,0x02,0x00,0x00,0x80,0x62,0x02,0x10,0x08,
0x00,0x04,0x00,0x80,0x40,0x00,0x20,0x20,0x00,0x10,0x10,0x10,0x80,0x80,0x88,0x80,
0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x08,0x40,
0x50,0x24,0x20,0xA8,0x92,0x00,0x48,0x87,0x87,0xC8,0x20,0x24,0x54,0x10,0x92,0x20,
0x20,0x40,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x10,0x00,0x08,0x08,0x08,0x04,0x04,0x44,0x04,0x04,0x04,0x04,
0x04,0x04,0x00,0x08,0x48,0x00,0x11,0x11,0x01,0x22,0x20,0x24,0x48,0x00,0x10,0x20,
0x00,0x43,0x00,0x0C,0x30,0x00,0x87,0x10,0x10,0x66,0x00,0x11,0x08,0x00,0x24,0x02,
0x12,0x09,0x00,0x44,0x04,0x00,0x12,0x12,0x12,0x09,0x09,0x09,0x48,0x48,0x08,0x48,
0x08,0x08,0x08,0x08,0x10,0x10,0x11,0x21,0x01,0x02,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x01,0x00,0x04,0x02,0x00,0x01,0x0F,0x0F,0x00,0x00,0x01,0x02,0x00,0x00,0x01,
0x01,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x90,0x80,0x48,0x48,0x48,0x28,0x20,0x24,0x10,
0x10,0x10,0x04,0x44,0x24,0x24,0x24,0x94,0x90,0x90,0x04,0x44,0x48,0x08,0x28,0xA0,
0x80,0x94,0x00,0x40,0x80,0x00,0x00,0xC0,0xC0,0x00,0x00,0xC0,0x10,0x04,0x24,0x20,
0x28,0x48,0x08,0x88,0x10,0x12,0x12,0x20,0x24,0x44,0x40,0x48,0x80,0x02,0x02,0x00,
0x04,0x04,0x00,0x08,0x08,0x00,0x12,0x20,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x80,0x20,0x00,0x10,0x00,
0x08,0x44,0x00,0x22,0x12,0x00,0x01,0x40,0x40,0x18,0x00,0x84,0x32,0x00,0x09,0x00,
0x00,0xE0,0x00,0x1E,0x01,0x00,0x38,0x81,0x81,0x0E,0x00,0x00,0x0F,0x00,0xF0,0x01,
0x01,0x0E,0x00,0x20,0x83,0x00,0x0C,0x20,0x20,0x82,0x00,0x04,0x08,0x00,0x31,0x02,
0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x18,0x00,0x04,0x01,
0x01,0x00,0x00,0x00,0x0C,0x00,0x01,0xC0,0xC0,0x3C,0x00,0x01,0x00,0x00,0x00,0xE0,
0xE0,0x1F,0x00,0x00,0x00,0x00,0x00,0x3F,0x3F,0xC0,0x00,0x00,0x00,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x03,0x00,0x78,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x00,
0x00,0x01,0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (ASCII 32)
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x00, 0x00}, // (
    {0x00, 0x00, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x04, 0x1C, 0x04, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x80, 0x70, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x49, 0x4D, 0x33}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x26, 0x49, 0x49, 0x49, 0x3E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x80, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x00, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x00, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x3A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x1C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x26, 0x49, 0x49, 0x49, 0x32}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x38, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
	{0x00, 0x7F, 0x41, 0x41, 0x41}, // [
	{0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
	{0x00, 0x41, 0x41, 0x41, 0x7F}, // ]
	{0x04, 0x02, 0x01, 0x02, 0x04}, // ^
	{0x40, 0x40, 0x40, 0x40, 0x40}, // _
	{0x00, 0x03, 0x07, 0x08, 0x00}, // `
	{0x20, 0x54, 0x54, 0x78, 0x40}, // a
	{0x7F, 0x28, 0x44, 0x44, 0x38}, // b
	{0x38, 0x44, 0x44, 0x44, 0x28}, // c
	{0x38, 0x44, 0x44, 0x28, 0x7F}, // d
	{0x38, 0x54, 0x54, 0x54, 0x18}, // e
	{0x00, 0x08, 0x7E, 0x09, 0x02}, // f
	{0x18, 0xA4, 0xA4, 0x9C, 0x78}, // g
	{0x7F, 0x08, 0x04, 0x04, 0x78}, // h
	{0x00, 0x44, 0x7D, 0x40, 0x00}, // i
	{0x20, 0x40, 0x40, 0x3D, 0x00}, // j
	{0x7F, 0x10, 0x28, 0x44, 0x00}, // k
	{0x00, 0x41, 0x7F, 0x40, 0x00}, // l
	{0x7C, 0x04, 0x78, 0x04, 0x78}, // m
	{0x7C, 0x08, 0x04, 0x04, 0x78}, // n
	{0x38, 0x44, 0x44, 0x44, 0x38}, // o
	{0xFC, 0x18, 0x24, 0x24, 0x18}, // p
	{0x18, 0x24, 0x24, 0x18, 0xFC}, // q
	{0x7C, 0x08, 0x04, 0x04, 0x08}, // r
	{0x48, 0x54, 0x54, 0x54, 0x24}, // s
	{0x04, 0x04, 0x3F, 0x44, 0x24}, // t
	{0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
	{0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
	{0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
	{0x44, 0x28, 0x10, 0x28, 0x44}, // x
	{0x4C, 0x90, 0x90, 0x90, 0x7C}, // y
	{0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x10, 0x3E, 0x10, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x00, 0x10, 0x3E, 0x00}, // }
    {0x00, 0x08, 0x04, 0x08, 0x00}, // ~
};


static void MX_GPIO_Init(void);
static void UART1_Init(void);
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

///can duplicate this if decided to add I2C3
void HAL_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10320309;  // 400khz
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

    HAL_I2C_Init(&hi2c1);
}
/* ================= ENHANCEMENTS CODE=====================*/

///code for LED MATRIX
// Function to initialize LED matrix
static void LED_Matrix_Init(void) {
	// 1. Turn on the internal oscillator
	uint8_t setup_cmd[2] = {0x21, 0x00}; // 0x21 = Turn on oscillator
	HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_I2C_ADDR << 1, setup_cmd, 1, 100);

	// 2. Set display brightness (0xE0 + brightness level)
	uint8_t brightness_cmd = 0xE0 | 0x0F; // Max brightness (0x00–0x0F)
	HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_I2C_ADDR << 1, &brightness_cmd, 1, 100);

	// 3. Turn on the display, no blinking (0x81 = Display on, blink off)
	uint8_t display_on_cmd = 0x81;
	HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_I2C_ADDR << 1, &display_on_cmd, 1, 100);
}

// Function to clear the LED matrix
void LED_Matrix_Clear(void) {
	uint8_t buffer[17];
	buffer[0] = 0x00; // Start address
	for (int i = 1; i < 17; i++) {
		buffer[i] = 0x00;
	}
	HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_I2C_ADDR << 1, buffer, 17, 100);
}

// Function to display a simple pattern on LED
// General helper to display any 8x8 pattern (16 bytes = 8 rows × 2 bytes each)
void LED_Matrix_Display(uint8_t pattern[16]) {
    uint8_t buffer[17];
    buffer[0] = 0x00; // Start at address 0

    // Copy user pattern to transmit buffer
    for (int i = 0; i < 16; i++) {
        buffer[i + 1] = pattern[i];
    }

    // Send over I2C to LED matrix
    HAL_I2C_Master_Transmit(&hi2c1, LED_MATRIX_I2C_ADDR << 1, buffer, 17, 100);
}

// Normal / Idle pattern (smiley face)
void LED_Matrix_DisplayNormal(void) {
    uint8_t pattern[16] = {
        0x3C, 0x00,
        0x42, 0x00,
        0xA5, 0x00,
        0x81, 0x00,
        0xA5, 0x00,
        0x99, 0x00,
        0x42, 0x00,
        0x3C, 0x00
    };
    LED_Matrix_Display(pattern);
}

// SAD FACE (Game Over)
void LED_Matrix_DisplaySadFace(void) {
    uint8_t pattern[16] = {
        0x3C, 0x00,
        0x42, 0x00,
        0xA5, 0x00,
        0x81, 0x00,
        0x99, 0x00,
        0xA5, 0x00,
        0x42, 0x00,
        0x3C, 0x00
    };
    LED_Matrix_Display(pattern);
}

// ⚪️ CIRCLE (Normal Mode)
void LED_Matrix_DisplayCircle(void) {
    uint8_t pattern[16] = {
        0x3C, 0x00,
        0x42, 0x00,
        0x81, 0x00,
        0x81, 0x00,
        0x81, 0x00,
        0x81, 0x00,
        0x42, 0x00,
        0x3C, 0x00
    };
    LED_Matrix_Display(pattern);
}

// ❗️ EXCLAMATION MARK (Warning)
void LED_Matrix_DisplayExclamation(void) {
    uint8_t pattern[16] = {
        0x18, 0x00,
        0x18, 0x00,
        0x18, 0x00,
        0x18, 0x00,
        0x18, 0x00,
        0x00, 0x00,
        0x18, 0x00,
        0x18, 0x00
    };
    LED_Matrix_Display(pattern);
}

// Game over (cross) pattern
void LED_Matrix_DisplayCross(void) {
    uint8_t pattern[16] = {
        0x81, 0x00,
        0x42, 0x00,
        0x24, 0x00,
        0x18, 0x00,
        0x18, 0x00,
        0x24, 0x00,
        0x42, 0x00,
        0x81, 0x00
    };
    LED_Matrix_Display(pattern);
}

// VICTORY ANIMATION (Fireworks effect)
uint8_t led_anim_playing = 0;
uint8_t led_anim_frame = 0;
uint32_t led_anim_next = 0;
uint8_t led_anim_repeat = 0;

void LED_Matrix_DisplayVictory(void) {
    led_anim_playing = 1;
    led_anim_frame = 0;
    led_anim_repeat = 0;
    led_anim_next = HAL_GetTick();
}

void LED_Matrix_AnimUpdate(void) {
    if (!led_anim_playing) return;
    static const uint8_t frames[5][16] = {
        {0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x10,0x00,0x38,0x00,0x7C,0x00,0x38,0x00,0x10,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x92,0x00,0x7C,0x00,0xFE,0x00,0x7C,0x00,0x92,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x44,0x00,0xAA,0x00,0x10,0x00,0xAA,0x00,0x44,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x24,0x00,0x18,0x00,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00}
    };

    uint32_t now = HAL_GetTick();
    if (now >= led_anim_next) {
        LED_Matrix_Display((uint8_t*)frames[led_anim_frame]);
        led_anim_frame++;
        led_anim_next = now + 150;

        if (led_anim_frame >= 5) {
            led_anim_frame = 0;
            led_anim_repeat++;
            if (led_anim_repeat >= 5) {
                led_anim_playing = 0;
                LED_Matrix_Clear();
            }
        }
    }
}

///
///code for buzzer
/* ===== Buzzer non-blocking support ===== */
uint8_t buzzer_active = 0;
uint32_t buzzer_freq = 0;
uint32_t buzzer_end_time = 0;
uint32_t buzzer_toggle_time = 0;
// -----------------------------------------------------------------------------
// Buzzer helper functions for tone generation
// -----------------------------------------------------------------------------
///BUZZER - play a tone at a given frequency (Hz) and duration (ms)
void Buzzer_PlayTone(uint32_t freq, uint32_t duration_ms) {
    buzzer_freq = freq;
    buzzer_toggle_time = HAL_GetTick();
    buzzer_end_time = HAL_GetTick() + duration_ms;
    buzzer_active = 1;
}

void Buzzer_Update(void) {
    if (!buzzer_active) return;

    uint32_t now = HAL_GetTick();
    uint32_t period_ms = 1000 / (buzzer_freq * 2); // half period in ms (approx)

    if (now >= buzzer_end_time) {
        buzzer_active = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        return;
    }

    if (elapsed(buzzer_toggle_time, period_ms)) {
        buzzer_toggle_time = now;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    }
}

///BUZZER - play predefined musical notes (simplified scale)
typedef struct {
    char note;
    uint32_t duration;
} Note;

const Note melody[] = {
    {'E',250}, {'C',200}, {'D',250}, {'B',200}, {'A',300}, {'G',400}, {'F',500}
};
const uint8_t melody_len = sizeof(melody)/sizeof(melody[0]);

uint8_t melody_index = 0;
uint8_t melody_playing = 0;
uint32_t melody_next_time = 0;

void Buzzer_PlayNote(char note, uint32_t duration_ms) {
    uint32_t freq = 1000; // default middle frequency
    // Map ASCII note symbols to approximate buzzer frequencies
    switch (note) {
        case 'C': freq = 1050; break;
        case 'D': freq = 1170; break;
        case 'E': freq = 1320; break;
        case 'F': freq = 1400; break;
        case 'G': freq = 1570; break;
        case 'A': freq = 1760; break;
        case 'B': freq = 1970; break;
        default: freq = 1000; break;
    }
    Buzzer_PlayTone(freq, duration_ms); // call tone generator
}

///BUZZER - play a short melody
void Buzzer_PlayMelody(void) {
    melody_index = 0;
    melody_playing = 1;
    melody_next_time = 0; // start immediately
}

void Melody_Update(void) {
    if (!melody_playing) return;
    uint32_t now = HAL_GetTick();

    if (now >= melody_next_time) {
        if (melody_index >= melody_len) {
            melody_playing = 0;
            return;
        }
        char note = melody[melody_index].note;
        uint32_t dur = melody[melody_index].duration;
        Buzzer_PlayNote(note, dur);
        melody_next_time = now + dur + 80; // gap between notes
        melody_index++;
    }
}

///code to define OLED
///OLED - fundamental function
void OLED_WriteCommand(uint8_t cmd){
	uint8_t data[2] = {0x00, cmd};
	HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR << 1, data, 2, 0xFFFF);
}
void OLED_WriteData(uint8_t data){
	uint8_t buffer[2] = {0x40, data};
	HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR << 1, buffer, 2, 0xFFFF);
}
///OLED - clear screen
void OLED_Clear(){                                // clears the OLED
	for (uint8_t page = 0; page < 8; page++){     // 8 pages, 8 rows per page
		OLED_WriteCommand(0xB0 + page);           // page 0 to page 7
		OLED_WriteCommand(0x00);                  // lower column address
		OLED_WriteCommand(0x10);                  // upper column address
		for (uint8_t col = 0; col < 128; col++){  // 128 columns
			uint8_t data[2] = {0x40, 0x00};       // change the data for each column to off
			HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR << 1, data, 2, 0xFFFF);
		}
	}
}
///use to clear 1 line in the OLED display
void OLED_ClearLine(uint8_t page) {
    if (page > 7) return;  // OLED has 8 pages: 0 to 7

    OLED_WriteCommand(0xB0 + page);  // Set page address
    OLED_WriteCommand(0x00);         // Set lower column address
    OLED_WriteCommand(0x10);         // Set upper column address

    for (uint8_t col = 0; col < 128; col++) {
        uint8_t data[2] = {0x40, 0x00};  // Clear column data
        HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDR << 1, data, 2, 0xFFFF);
    }
}
///OLED - run each character
void OLED_PutChar(char c){                   // ASCII only printable from 32-126
	if(c < 32 || c > 126){
		c = 32;                              // set to space if not printable
	}
    const uint8_t *bitmap = font5x7[c - 32]; // -32 as space starts from 32
    for (int i = 0; i < 5; i++) {            // each character is 5 wide
        OLED_WriteData(bitmap[i]);           // send each column of the character
    }
    OLED_WriteData(0x00); // space between characters
}
///OLED - display picture
void OLED_DisplayBitMap(const uint8_t *bitmap){
	for (uint8_t page = 0; page < 8; page++){
		OLED_WriteCommand(0xB0 + page);
		OLED_WriteCommand(0x00);
		OLED_WriteCommand(0x10);
		for (uint8_t col = 0; col < 128; col++){
			OLED_WriteData(bitmap[page * 128 + col]);
		}
	}
}
///OLED - display message
void OLED_PrintNormal(uint8_t page, const char* message){
    // Set the page address
    OLED_WriteCommand(0xB0 + page);                      // Set page start address
    OLED_WriteCommand(0x00 + (34 & 0x0F));              // Set low column address
    OLED_WriteCommand(0x10 + ((34>>4)& 0x0F));         // Set high column address
	// Loop through each character in the message
    while (*message) {
        OLED_PutChar(*message++);                      // Print
    }
}


/*============================ENHANCEMENT CODE END ==========================================*/
//for ease of typing
static void uart_send(const char *s) {
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* Constants */
const uint32_t STATE_PERIOD_MS = 10000;     // RLGL: 10 seconds per state
const uint32_t SAMPLE_MS = 2000;      // RLGL: print sensors every 2 s
const uint32_t LED_TOGGLE_MS = 500; // RLGL: LED blink during red light
const uint32_t DOUBLE_PRESS_MS = 1000; // PB double press
const uint32_t CR_ENV_MS = 1000; // CR: Monitor Environment every 1s
const uint32_t CR_RESPONSE_MS = 3000; // CR: PB within 3s

/* Game Status */
typedef enum { PHASE_GREEN = 0, PHASE_RED } phase_t;
typedef enum { GAME_RLGL = 0, GAME_CATCHRUN } game_mode_t;

game_mode_t game = GAME_RLGL;
phase_t phase =  PHASE_GREEN;

// flags
uint32_t t_now = 0, last_state = 0, last_sample = 0, last_led = 0;
uint32_t first_press = 0, proximity_start = 0;
uint8_t  waiting_press = 0;
uint8_t rlgl_game_over = 0;
uint8_t cr_game_over = 0;
uint8_t cr_proximity = 0;
const uint32_t WIN_MS = 30000; // win when >30s
uint32_t game_start = 0; // when the current game started/restarted

/* =======Sensor thresholds ======= */
const float DELTA_A_THRESH_MS2 = 0.5f;    // accel change (m/s^2)
const float DELTA_G_THRESH_DPS = 2.0f;   // gyro change (deg/s)
const float DELTA_B_THRESH     = 50.0f;   // magnetometer magnitude change
const float DELTA_T_C          = 1.0f;    // temperature change (°C)
const float DELTA_H_PCT        = 5.0f;    // humidity change (%)
const float DELTA_P_HPA        = 5.0f;    // pressure change (hPa)

/* --- Previous samples for delta comparisons --- */
float prev_ax=0, prev_ay=0, prev_az=0;
float prev_gx=0, prev_gy=0, prev_gz=0;
uint8_t have_prev_motion = 0;

float prev_b = 0;
uint8_t have_prev_b = 0;

float prev_t=0, prev_h=0, prev_p=0;
uint8_t have_prev_env = 0;

/* ======= Catch&Run blink thresholds ======= */
const float B_THRESH_LOW = 1500.0f;
const float B_THRESH_MID = 2000.0f;
const float B_THRESH_HIGH = 2500.0f;
volatile uint32_t blink_rate = 600; // updated from magnetometer

/* ===== Sensor Helpers ===== */
static void readMotion(float *a, float *g) {
    int16_t ar[3];
    float gr[3];
    BSP_ACCELERO_AccGetXYZ(ar);
    BSP_GYRO_GetXYZ(gr);
    a[0] = ar[0] * (9.8/1000.0f); a[1] = ar[1] * (9.8/1000.0f); a[2] = ar[2] * (9.8/1000.0f);
    g[0] = gr[0]*(0.07/1000.0f); g[1] = gr[1]*(0.07/1000.0f); g[2] = gr[2]*(0.07/1000.0f);
}

static float readMag(void) {
    int16_t mr[3]; BSP_MAGNETO_GetXYZ(mr);
    float mx = mr[0], my = mr[1], mz = mr[2];
    return sqrtf(mx*mx + my*my + mz*mz);
}

static void readEnv(float *t, float *h, float *p) {
    *t = BSP_TSENSOR_ReadTemp();
    *h = BSP_HSENSOR_ReadHumidity();
    *p = BSP_PSENSOR_ReadPressure();
}

/* ===== Game Logic ===== */
void runRLGL(void) {
	if (rlgl_game_over) { BSP_LED_Off(LED2); return; }

	/* WIN after 30s if still alive */
	if (elapsed(game_start, WIN_MS)) {
	    rlgl_game_over = 1;
	    BSP_LED_Off(LED2);
	    uart_send("Congrats, you won!\r\n");
	    OLED_Clear();
	    OLED_DisplayBitMap(custom_image);
	    LED_Matrix_DisplayVictory();
	    return;
	}

    if (elapsed(last_state, STATE_PERIOD_MS)) //if light has been in the state for more than 10s, change
    {
        phase = (phase == PHASE_GREEN) ? PHASE_RED : PHASE_GREEN;
        last_state = t_now;
        last_led = t_now;
        have_prev_motion = 0;
        if (phase == PHASE_GREEN) {
        	uart_send("Green Light!\r\n");
        	OLED_ClearLine(5);
        	OLED_PrintNormal(4, "GREEN LIGHT!");
        	LED_Matrix_DisplayCircle();
        } else {
        	uart_send("Red Light!\r\n");
        	OLED_ClearLine(4);
        	OLED_PrintNormal(5, "RED LIGHT!");
        	LED_Matrix_DisplayCross();
        }
    }

    /* LED Behaviour */
    if (phase == PHASE_GREEN) BSP_LED_On(LED2); // led always on in green
    else if (elapsed(last_led, LED_TOGGLE_MS))
    {
        last_led = t_now;
        BSP_LED_Toggle(LED2); // 0.5s blink
    }

    /* Sampling */
    if (elapsed(last_sample, SAMPLE_MS)) //print data every 2s
    {
        last_sample = t_now;
        if (phase == PHASE_GREEN)
        {
            float t,h,p; readEnv(&t,&h,&p);
            char msg[80];
            sprintf(msg, "Env => Temp:%.2fC, Humid:%.2f%%, Press:%.2f hPa\r\n", t,h,p);
            uart_send(msg);
        }
        else
        {
        	float a[3], g[3];
        	readMotion(a, g);
            char msg[120];
            sprintf(msg, "Motion => Accelero[%.2f,%.2f,%.2f], Gyro[%.1f,%.1f,%.1f]\r\n", a[0],a[1],a[2],g[0],g[1],g[2]);
            uart_send(msg);

            float da=0.0f, dg=0.0f; // delta of a ; delta of g
            if (!have_prev_motion) // set first set of readings a "prev reading"
            {
                prev_ax=a[0]; prev_ay=a[1]; prev_az=a[2];
                prev_gx=g[0]; prev_gy=g[1]; prev_gz=g[2];
                have_prev_motion = 1;
            }
            else
            {
                float dax = fabsf(a[0]-prev_ax), day=fabsf(a[1]-prev_ay), daz=fabsf(a[2]-prev_az); // change in a compared to prev reading
                float dgx = fabsf(g[0]-prev_gx), dgy=fabsf(g[1]-prev_gy), dgz=fabsf(g[2]-prev_gz); // change in g compared to prev reading
                da = fmaxf(dax, fmaxf(day, daz));
                dg = fmaxf(dgx, fmaxf(dgy, dgz));

                if (da > DELTA_A_THRESH_MS2 || dg > DELTA_G_THRESH_DPS) //if delta a or delta g exceed threshold , movement detected, game over
                {
                    uart_send("Game Over\r\n");
                    OLED_Clear();
                    OLED_PrintNormal(5,"GAME OVER!");
                	LED_Matrix_DisplaySadFace();
                    BSP_LED_Off(LED2);
                    Buzzer_PlayMelody();
                    rlgl_game_over = 1;
                }

                /* update previous */
                prev_ax=a[0]; prev_ay=a[1]; prev_az=a[2];
                prev_gx=g[0]; prev_gy=g[1]; prev_gz=g[2];
            }
        }
    }
}

void runCatchRun(void) {
	if (cr_game_over) {BSP_LED_Off(LED2);return;}

	/* WIN after 30s if still alive */
	if (elapsed(game_start, WIN_MS)) {
	    cr_game_over = 1;
	    BSP_LED_Off(LED2);
	    uart_send("Congrats, you won!\r\n");
	    OLED_Clear();
	    OLED_DisplayBitMap(custom_image);
	    LED_Matrix_DisplayVictory();
	    return;
	}

    float b = readMag();
    float db = 0.0f;

    /* Blink speed mapping */
    blink_rate = (b>B_THRESH_HIGH)?150:(b>B_THRESH_MID)?300:(b>B_THRESH_LOW)?600:1000; //higher mag value, faster led blinks

    if (!have_prev_b)
    {
        prev_b = b;
        have_prev_b = 1;
    }
    else { db = fabsf(b - prev_b); }

    /* Start proximity window when mag field changes beyond delta threshold */
    if (db > DELTA_B_THRESH && !cr_proximity) // when mag change detected
    {
        cr_proximity = 1;
        waiting_press = 1;
        proximity_start = t_now;
        uart_send("Enforcer nearby! Be careful.\r\n");
        OLED_PrintNormal(5,"WARNING!");

        last_led=t_now;
    }

    /* Blink while we consider proximity active; otherwise LED off */
    if (cr_proximity)
    {
        if (elapsed(last_led, blink_rate)) //blink according to blink rate
        {
            last_led = t_now;
            BSP_LED_Toggle(LED2);
            LED_Matrix_DisplayExclamation();
        }
        // 3s window upon proximity detected
        if (waiting_press && elapsed(proximity_start, CR_RESPONSE_MS)) //if no button press in 3s, game over
        {
        	waiting_press = 0;
        	cr_proximity = 0;
        	BSP_LED_Off(LED2);
        	uart_send("Game Over!\r\n");
            OLED_Clear();
            OLED_PrintNormal(5,"GAME OVER!");
            LED_Matrix_DisplaySadFace();
            Buzzer_PlayMelody();
        	cr_game_over = 1;
        }
    }
    else {
        BSP_LED_Off(LED2);
    }

    prev_b = b;

    /* Environment check every 1s */
    if (elapsed(last_sample, CR_ENV_MS))
    {
        last_sample = t_now;
        float t,h,p; readEnv(&t,&h,&p);

        if (!have_prev_env)
        {
            prev_t=t; prev_h=h; prev_p=p;
            have_prev_env = 1;
        }
        else
        {
            float dt = fabsf(t - prev_t); //check delta of env sensors
            float dh = fabsf(h - prev_h);
            float dp = fabsf(p - prev_p);

            if (dt > DELTA_T_C)
            {
                char m[64]; sprintf(m,"Temperature spike detected! T:%.1fC\r\n", t);
                uart_send(m);
            }
            if (dh > DELTA_H_PCT)
            {
                char m[64]; sprintf(m,"Humidity spike detected! H:%.1f%%\r\n", h);
                uart_send(m);
            }
            if (dp > DELTA_P_HPA)
            {
                char m[64]; sprintf(m,"Pressure change detected! P:%.1f hPa\r\n", p);
                uart_send(m);
            }

            /* update previous baseline */
            prev_t=t; prev_h=h; prev_p=p;
        }
    }
}

/* ===== Button: Double-Press Switch + Single Press ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin != USER_BUTTON_PIN) return;
    uint32_t now = HAL_GetTick();

    /* Single press (Catch & Run only) */
    if (game==GAME_CATCHRUN && waiting_press) {
        waiting_press = 0;
        cr_proximity = 0;
        // end proximity immediately
        BSP_LED_Off(LED2);
        LED_Matrix_Clear();
        OLED_ClearLine(5);
        uart_send("Player escaped, good job!\r\n");
        return; //return to prevent this press to be detected as part of a double press
    }
    /* Double Press */
    if (first_press && (now - first_press) <= DOUBLE_PRESS_MS) { //if two press detected within 1s
    	/* reset timers and flags*/
    	first_press = 0;
    	proximity_start = now;
    	waiting_press = 0;
    	last_state = last_sample = last_led = game_start = now;
    	rlgl_game_over = 0;
    	cr_game_over = 0;
    	cr_proximity = 0;
    	have_prev_b = 0;
    	have_prev_env = 0;
    	have_prev_motion = 0;
    	/* reinitialise sensors */
    	BSP_LED_Off(LED2);
    	BSP_MAGNETO_Init();
    	BSP_TSENSOR_Init();
    	BSP_HSENSOR_Init();
    	BSP_PSENSOR_Init();
    	I2C1_GPIO_Init();
    	HAL_I2C1_Init();
    	OLED_Init();
    	LED_Matrix_Init();
    	Buzzer_Init();
    	// clear all display
    	OLED_Clear();
    	LED_Matrix_Clear();


        game = (game==GAME_RLGL)?GAME_CATCHRUN:GAME_RLGL; //switch games
        if (game==GAME_RLGL) {
        	uart_send(">>> Switching to Red Light, Green Light\r\nEntering Red Light, Green Light as Player\r\nGreen Light!\r\n");
			OLED_Clear();
        	OLED_PrintNormal(0, "RLGL: PLAYER");
        	OLED_PrintNormal(4, "GREEN LIGHT!");
        	LED_Matrix_DisplayCircle();

        } else{
        	uart_send(">>> Switching to Catch and Run\r\nEntering Catch and Run as Player\r\n");
			OLED_Clear();
        	OLED_PrintNormal(0, "C & R: PLAYER");

        }
        phase = PHASE_GREEN; // start rlgl in green light

        return;
    }
    first_press = now; //detect time of first press

}

/* ===== Main ===== */

int main(void)
{
	HAL_Init();
    MX_GPIO_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init(); BSP_GYRO_Init(); BSP_MAGNETO_Init();
    BSP_TSENSOR_Init();  BSP_HSENSOR_Init(); BSP_PSENSOR_Init();
//	HAL_SYSTICK_Config(SystemCoreClock / 1000); // Configure SysTick

	//for BUZZER
	Buzzer_Init();

	//for OLED
	I2C1_GPIO_Init();
	HAL_I2C1_Init();
	OLED_Init();
	OLED_Clear();

	//FOR LED MATRIX
	LED_Matrix_Init();
	LED_Matrix_Clear();
	//start in RLGL green light
	OLED_PrintNormal(0, "RLGL: PLAYER");
	OLED_PrintNormal(4, "GREEN LIGHT!");
    uart_send("Entering Red Light, Green Light as Player\r\nGreen Light!\r\n");
	LED_Matrix_DisplayCircle();
    BSP_LED_On(LED2);
    last_state = last_sample = last_led = game_start = HAL_GetTick();


    while (1) {
        t_now = HAL_GetTick();
        if (game == GAME_RLGL) runRLGL();
        else runCatchRun();

        Buzzer_Update();
        Melody_Update();
        LED_Matrix_AnimUpdate();
    }
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = USER_BUTTON_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}
//
static void Buzzer_Init(void){
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


///pin8 and pin9
static void I2C1_GPIO_Init(void){
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

///keep
static void OLED_Init(void) {
	OLED_WriteCommand(0xAE); //off display
	OLED_WriteCommand(0x81); //set contrast
	OLED_WriteCommand(0x7F); //contrast value
	OLED_WriteCommand(0xA1); //set segment re-map
	OLED_WriteCommand(0xC8); //set COM output scan direction
	OLED_WriteCommand(0xA6); //display normal
	OLED_WriteCommand(0xAF); //on display
}
