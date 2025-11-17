#include <math.h>
#include <inttypes.h>
#include "stm32u3xx_hal.h"
#include "UFC23_Utils.h"

static UART_HandleTypeDef huart;

void InsertPointInArray(float newData, float data[], uint16_t* insertionPoint, uint16_t maxAmountPoints, uint8_t* arrayFull)
{
    data[*insertionPoint] = newData;

    *insertionPoint = *insertionPoint + 1;
    if( *insertionPoint >= maxAmountPoints )
    {
        *arrayFull = 1;
        *insertionPoint = *insertionPoint % maxAmountPoints;
    }
}

void CalculateStatistics(float data[], uint16_t points, float* mean, float* std)
{
    float sum = 0;
    for (uint16_t i = 0; i < points; i++) {
        sum += data[i];
    }

    *mean = sum / points;

    float values = 0;

    for (uint16_t i = 0; i < points; i++) {
        values += pow(data[i] - *mean, 2);
    }

    float variance = values / points;

    *std = sqrt(variance);
}


void SetUartHandle(UART_HandleTypeDef* uartHandle)
{
    huart = *uartHandle;
}

uint8_t SerialPrint(const char str[])
{
    uint16_t i = 0;
    while(str[i] != 0)
    {
        i++;
    }

    return (uint8_t)HAL_UART_Transmit(&huart, (uint8_t*)str, i, 10);
}

