/* Host-side stub of the STM32 HAL surface used by samplecode.c (for logic tests only) */
#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef enum { HAL_OK = 0, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { int id; } GPIO_TypeDef;
extern GPIO_TypeDef sim_gpiob, sim_gpioc, sim_gpiod;
#define GPIOB (&sim_gpiob)
#define GPIOC (&sim_gpioc)
#define GPIOD (&sim_gpiod)
#define GPIO_PIN_1 0x0002
#define GPIO_PIN_6 0x0040
#define GPIO_PIN_7 0x0080
#define GPIO_PIN_8 0x0100
#define GPIO_PIN_9 0x0200
#define GPIO_PIN_11 0x0800
#define GPIO_PIN_13 0x2000
#define BUTTON_EXTI13_Pin GPIO_PIN_13
#define BUTTON_EXTI13_GPIO_Port GPIOC
#define LSM6DSL_INT1_EXTI11_Pin GPIO_PIN_11
#define LSM6DSL_INT1_EXTI11_GPIO_Port GPIOD
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
enum { GPIO_MODE_AF_OD=1, GPIO_MODE_AF_PP, GPIO_MODE_IT_RISING, GPIO_MODE_IT_RISING_FALLING,
       GPIO_PULLUP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_SPEED_FREQ_LOW,
       GPIO_AF4_I2C1, GPIO_AF2_TIM3, GPIO_AF7_USART1 };
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin);
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
typedef struct { int id; } PERIPH;
extern PERIPH sim_i2c1, sim_usart1, sim_tim3;
#define I2C1 (&sim_i2c1)
#define USART1 (&sim_usart1)
#define TIM3 (&sim_tim3)
typedef struct { PERIPH *Instance; struct { uint32_t Timing, OwnAddress1, AddressingMode, DualAddressMode, OwnAddress2, GeneralCallMode, NoStretchMode; } Init; uint32_t ErrorCode; } I2C_HandleTypeDef;
enum { I2C_ADDRESSINGMODE_7BIT=1, I2C_DUALADDRESS_DISABLE, I2C_GENERALCALL_DISABLE, I2C_NOSTRETCH_DISABLE };
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t a, uint32_t t, uint32_t to);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to);
typedef struct { PERIPH *Instance; struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling, OneBitSampling; } Init; struct { uint32_t AdvFeatureInit; } AdvancedInit; } UART_HandleTypeDef;
enum { UART_WORDLENGTH_8B=1, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16, UART_ONE_BIT_SAMPLE_DISABLE, UART_ADVFEATURE_NO_INIT };
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *d, uint16_t n);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
void HAL_UART_IRQHandler(UART_HandleTypeDef *h);
typedef struct { PERIPH *Instance; struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, AutoReloadPreload; } Init; uint32_t arr, ccr, cnt; } TIM_HandleTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
enum { TIM_COUNTERMODE_UP=1, TIM_CLOCKDIVISION_DIV1, TIM_AUTORELOAD_PRELOAD_DISABLE, TIM_OCMODE_PWM1, TIM_OCPOLARITY_HIGH, TIM_OCFAST_DISABLE, TIM_CHANNEL_4 };
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *o, uint32_t c);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c);
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->ccr = (v))
#define __HAL_TIM_SET_AUTORELOAD(h,v) ((h)->arr = (v))
#define __HAL_TIM_GET_COUNTER(h) ((h)->cnt)
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->cnt = (v))
uint32_t HAL_RCC_GetPCLK1Freq(void);
typedef enum { EXTI15_10_IRQn = 40, USART1_IRQn = 37 } IRQn_Type;
void HAL_NVIC_SetPriority(IRQn_Type i, uint32_t a, uint32_t b);
void HAL_NVIC_EnableIRQ(IRQn_Type i);
void HAL_NVIC_DisableIRQ(IRQn_Type i);
enum { PWR_WAKEUP_PIN1=1, PWR_WAKEUP_PIN2, PWR_WAKEUP_PIN3, PWR_WAKEUP_PIN4, PWR_WAKEUP_PIN5, PWR_FLAG_WU };
void HAL_PWR_DisableWakeUpPin(uint32_t p);
#define __HAL_PWR_CLEAR_FLAG(f) ((void)0)
void HAL_PWR_EnterSTANDBYMode(void);
void HAL_SuspendTick(void);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_Init(void);
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __WFI(void) {}
typedef enum { LED2 = 0 } Led_TypeDef;
void BSP_LED_Init(Led_TypeDef l); void BSP_LED_On(Led_TypeDef l); void BSP_LED_Off(Led_TypeDef l);
enum { TSENSOR_OK = 0, HSENSOR_OK = 0, PSENSOR_OK = 0, ACCELERO_OK = 0, GYRO_OK = 0, MAGNETO_OK = 0 };
uint32_t BSP_TSENSOR_Init(void); uint32_t BSP_HSENSOR_Init(void); uint32_t BSP_PSENSOR_Init(void);
int BSP_ACCELERO_Init(void); uint8_t BSP_GYRO_Init(void); int BSP_MAGNETO_Init(void);
float BSP_TSENSOR_ReadTemp(void); float BSP_HSENSOR_ReadHumidity(void); float BSP_PSENSOR_ReadPressure(void);
void BSP_ACCELERO_AccGetXYZ(int16_t *p); void BSP_GYRO_GetXYZ(float *p); void BSP_MAGNETO_GetXYZ(int16_t *p);
/* --- additions for ADC / IR / I2C tools --- */
#define GPIO_PIN_3 0x0008
#define GPIO_PIN_4 0x0010
#define GPIO_PIN_5 0x0020
enum { GPIO_MODE_INPUT = 100, GPIO_MODE_ANALOG_ADC_CONTROL };
typedef struct { volatile uint32_t CR, ISR, CFGR, SMPR1, SMPR2, SQR1, DR; } ADC_TypeDef;
typedef struct { volatile uint32_t CCR; } ADC_Common_TypeDef;
typedef struct { volatile uint32_t AHB2ENR; } RCC_TypeDef;
extern ADC_TypeDef sim_adc1; extern ADC_Common_TypeDef sim_adc_common; extern RCC_TypeDef sim_rcc;
#define ADC1 (&sim_adc1)
#define ADC1_COMMON (&sim_adc_common)
#define RCC (&sim_rcc)
#define RCC_AHB2ENR_ADCEN (1u<<13)
#define ADC_CCR_CKMODE (3u<<16)
#define ADC_CCR_CKMODE_0 (1u<<16)
#define ADC_CR_DEEPPWD (1u<<29)
#define ADC_CR_ADVREGEN (1u<<28)
#define ADC_CR_ADCALDIF (1u<<30)
#define ADC_CR_ADCAL 0u            /* sim: calibration completes instantly */
#define ADC_CR_ADEN (1u<<0)
#define ADC_CR_ADSTART (1u<<2)
#define ADC_ISR_ADRDY (1u<<0)
#define ADC_ISR_EOC (1u<<2)
#define ADC_ISR_EOS (1u<<3)
#define ADC_ISR_OVR (1u<<4)
#define ADC_CFGR_OVRMOD (1u<<12)
#define ADC_SMPR1_SMP4_Pos 12u
#define ADC_SMPR2_SMP13_Pos 9u
#define ADC_SQR1_SQ1_Pos 6u
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to);
void HAL_Delay(uint32_t ms);
/* --- additions for flash black box / reset --- */
#include <stdint.h>
typedef struct { volatile uint32_t OPTR; } FLASH_TypeDef;
extern FLASH_TypeDef sim_flash_regs;
extern uint8_t sim_flash_page[8192];   /* [0..4K) = page 254 (config), [4K..8K) = page 255 (black box) */
#define FLASH (&sim_flash_regs)
#define FLASH_OPTR_DBANK (1u << 22)
#define FLASH_BASE ((uintptr_t)sim_flash_page + 0x2000u - 0x200000u)
typedef struct { uint32_t TypeErase, Banks, Page, NbPages; } FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_PAGES 0u
#define FLASH_BANK_1 1u
#define FLASH_BANK_2 2u
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0u
#define FLASH_FLAG_ALL_ERRORS 0u
#define __HAL_FLASH_CLEAR_FLAG(f) ((void)0)
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data);
void NVIC_SystemReset(void);

/* --- additions for I2C bus recovery --- */
#define RESET 0
#define I2C_FLAG_BUSY 0x8000u
extern int sim_i2c_busy;
#define __HAL_I2C_GET_FLAG(h, f) (sim_i2c_busy ? 1 : 0)
enum { GPIO_MODE_OUTPUT_OD = 200 };
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h);
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s);
enum { GPIO_PULLDOWN = 300 };
#define HAL_I2C_ERROR_BERR 0x01u
#define HAL_I2C_ERROR_ARLO 0x02u
#define HAL_I2C_ERROR_AF 0x04u
#define HAL_I2C_ERROR_OVR 0x08u
#define HAL_I2C_ERROR_TIMEOUT 0x20u
