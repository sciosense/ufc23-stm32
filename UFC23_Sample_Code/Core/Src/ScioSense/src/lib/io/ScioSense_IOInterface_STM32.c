#ifndef SCIOSENSE_IO_INTERFACE_STM32_C
#define SCIOSENSE_IO_INTERFACE_STM32_C

#include "stddef.h"

#include "stm32u3xx_hal.h"
#include "stm32u3xx_hal_gpio.h"

#define STM32_SPI_TIMEOUT_MS    10
#define SET_LOW			GPIO_PIN_RESET
#define SET_HIGH		GPIO_PIN_SET

typedef struct ScioSense_Stm32_Spi_Config
{
    SPI_HandleTypeDef*  spi;
    uint16_t            cs_pin;
    GPIO_TypeDef*       port;
} ScioSense_Stm32_Spi_Config;

static inline int8_t ScioSense_STM32_Spi_Write_Data(void* config, uint8_t* data, const size_t size)
{
    SPI_HandleTypeDef*  spi     = ((ScioSense_Stm32_Spi_Config*)config)->spi;
    uint16_t            cs_pin  = ((ScioSense_Stm32_Spi_Config*)config)->cs_pin;
    GPIO_TypeDef*       port    = ((ScioSense_Stm32_Spi_Config*)config)->port;

    /* 1. Put CN low - Activate */
    HAL_GPIO_WritePin(port, cs_pin, SET_LOW);
    
    /* 2. Transmit register address */
    HAL_SPI_Transmit(spi, data, size, STM32_SPI_TIMEOUT_MS);
    
    /* 3. Put SSN high - Deactivate */
    HAL_GPIO_WritePin(port, cs_pin, SET_HIGH);

    return 0; // RESULT_OK;
}

static inline int8_t ScioSense_STM32_Spi_Transfer(void* config, uint8_t* dataToWrite, const size_t sizeToWrite, uint8_t* dataToRead, const size_t sizeToRead)
{
    SPI_HandleTypeDef*  spi     = ((ScioSense_Stm32_Spi_Config*)config)->spi;
    uint16_t            cs_pin  = ((ScioSense_Stm32_Spi_Config*)config)->cs_pin;
    GPIO_TypeDef*       port    = ((ScioSense_Stm32_Spi_Config*)config)->port;

    /* 1. Put SSN low - Activate */
    HAL_GPIO_WritePin(port, cs_pin, SET_LOW);
    
    /* 2. Transmit register address */
    HAL_SPI_Transmit(spi, dataToWrite, sizeToWrite, STM32_SPI_TIMEOUT_MS);

    /* 3. Read the data */
    HAL_SPI_Receive(spi, dataToRead, sizeToRead, STM32_SPI_TIMEOUT_MS);
    
    /* 4. Put SSN high - Deactivate */
    HAL_GPIO_WritePin(port, cs_pin, SET_HIGH);

    return 0; // RESULT_OK;
}

static inline void ScioSense_STM32_Wait(uint32_t ms)
{
    HAL_Delay(ms);
}

#undef SET_LOW
#undef SET_HIGH

#endif // SCIOSENSE_IO_INTERFACE_STM32_C
