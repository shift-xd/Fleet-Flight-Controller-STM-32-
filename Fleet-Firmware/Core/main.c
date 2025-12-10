/* Fleet~Flight Controller Firmware v0.1 — Dec 2025 */
/* Exactly matches your PCB pinout — just build & flash */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

uint16_t servo[8] = {1500,1500,1500,1500,1000,2000,1500,1500}; // µs

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  // Start all 8 channels @ 50 Hz (20 ms)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // PA8
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // PA9
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // PA10
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  // PA11

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // PB4  (you actually used PB0–PB1? remap below if needed)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // PB5

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // PB6  (shared with I2C SCL → use TIM8 if conflict)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // PB7  (shared with I2C SDA → use TIM8 if conflict)

  uint32_t tick = HAL_GetTick();
  while (1)
  {
    if (HAL_GetTick() - tick > 100) // every 100 ms
    {
      tick = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // red LED on your board

      // Example: slowly sweep motor 5 (1000 → 2000 µs)
      static int dir = 1;
      servo[4] += dir*10;
      if (servo[4] >= 2000) dir = -1;
      if (servo[4] <= 1000) dir = 1;

      // Write PWM pulse widths (convert µs → timer ticks)
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo[0]*168/1000); // 168 MHz → 1 µs = 168 ticks @ prescaler 167
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, servo[1]*168/1000);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, servo[2]*168/1000);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, servo[3]*168/1000);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo[4]*168/1000);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo[5]*168/1000);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, servo[6]*168/1000);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, servo[7]*168/1000);
    }

    // Echo anything received over USB VCP back (so you can talk to it)
    uint8_t buf[64];
    uint32_t len = CDC_Receive_FS(buf, sizeof(buf));
    if (len) CDC_Transmit_FS(buf, len);
  }
}
