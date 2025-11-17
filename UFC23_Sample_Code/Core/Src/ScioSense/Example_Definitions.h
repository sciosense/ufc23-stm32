#ifndef UFC23_EXAMPLE_BASIC_H
#define UFC23_EXAMPLE_BASIC_H

#include <inttypes.h>
#include "stm32u3xx_hal.h"

#ifndef UFC23_PIN_DEFINITIONS
#define SCK_Pin             GPIO_PIN_5
#define SCK_GPIO_Port       GPIOA
#define MISO_Pin            GPIO_PIN_6
#define MISO_GPIO_Port      GPIOA
#define MOSI_Pin            GPIO_PIN_7
#define MOSI_GPIO_Port      GPIOA
#define INTN_Pin            GPIO_PIN_7
#define INTN_GPIO_Port      GPIOC
#define SSN_Pin             GPIO_PIN_9
#define SSN_GPIO_Port       GPIOC
#define WAKE_UP_Pin         GPIO_PIN_5
#define WAKE_UP_GPIO_Port   GPIOB
#endif // UFC23_PIN_DEFINITIONS

uint8_t WaitOnInterrupt(uint16_t timeoutMs);

#ifdef __cplusplus
extern "C" {
#endif

void UFC23_Example_Setup(UART_HandleTypeDef *uart, SPI_HandleTypeDef *spi);
void UFC23_Example_Loop();
void UFC23_HandleGpioInterrupt(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif // UFC23_EXAMPLE_BASIC_H
