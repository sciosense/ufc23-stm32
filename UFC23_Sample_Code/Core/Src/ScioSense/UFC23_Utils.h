#include <inttypes.h>
#include "stm32u3xx_hal.h"

#define UFC23_NS_TO_PS  1000

void    InsertPointInArray  (float newData, float data[], uint16_t* insertionPoint, uint16_t maxAmountPoints, uint8_t* arrayFull);
void    CalculateStatistics (float data[], uint16_t points, float* mean, float* std);
void    SetUartHandle       (UART_HandleTypeDef* uartHandle);
uint8_t SerialPrint         (const char str[]);