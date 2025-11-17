#include "Example_Definitions.h"
#include "UFC23_Utils.h"
#include "src/ScioSense_UFC23.h"
#include <cstdio>
#include <cstring>

#define AMOUNT_POINTS_STATISTICS    100

static char messageBuffer[128];         // Buffer for sending data through Serial
uint8_t interruptAsserted = 0;

UFC23 ufc23;

float temp1, temp2;
float tofAvgUp[UFC23_AMOUNT_BUNDLES_MAX], tofAvgDn[UFC23_AMOUNT_BUNDLES_MAX];
float tofDiff[AMOUNT_POINTS_STATISTICS];
uint16_t arrayInsertionPoint = 0;
uint8_t arrayFull = 0;

extern "C" void UFC23_Example_Setup(UART_HandleTypeDef *uart, SPI_HandleTypeDef *spi)
{
    SetUartHandle(uart);

    /* Wait to allow terminal software to capture the output */
    HAL_Delay(2000);

    SerialPrint("\nStarting UFC23 05_Temperature demo on STM32...\n");

    HAL_Delay(UFC23_T_RC_RLS_MS);

    ufc23.begin(spi, SSN_Pin, SSN_GPIO_Port);

    while( !ufc23.init())
    {
        SerialPrint("UFC23 initialization failed\n");
        HAL_Delay(1000);        
    }

    SerialPrint(ufc23.partIdToString(ufc23.partId));
    SerialPrint(" initialized properly\n");

    // Configuration for reading the single ended USM and two temperature ports
    uint32_t configRegisters[UFC23_AMOUNT_CONFIGURATION_REGISTERS] =
    {
        0x0000001C,     // A0
        0x00000FFF,     // A1
        0x000006DB,     // A2
        0x00000010,     // A3
        0x000017AF,     // A4
        0x0000B100,     // A5
        0x00011249,     // A6
        0x000194F4,     // A7
        0x04900000,     // A9
        0xC00F0034,     // AA
        0x0000140E,     // AB
        0x00000000,     // AC
        0x0808B00E,     // AD
        0x46301024,     // AE
        0x0FFFFFFF,     // AF
        0x0001424E,     // B0
        0x20412424,     // B1
        0x00000000      // B2
    };

    ufc23.setConfigurationRegisters(configRegisters);

    // Measure the High Speed Oscillator frequency
    float hsoFreqMHz[UFC23_AMOUNT_BUNDLES_MAX];
    ufc23.getHighSpeedOscillatorFrequencyMhz(hsoFreqMHz);
    sprintf(messageBuffer, "High Speed Clock Frequency: %0.3f MHz\n", hsoFreqMHz[0]);
    SerialPrint(messageBuffer);

    if( ufc23.writeConfig() == RESULT_OK )
    {
        SerialPrint("Configuration properly written\n");
    }
    else
    {
        SerialPrint("Error! Configuration read doesn't match the values written\n");
    }
    
    if( ufc23.startMeasurement() == RESULT_OK )
    {
        SerialPrint("Measurements started\n");
    }
    else
    {
        SerialPrint("Error! Measurements didn't start properly\n");
    }
}

extern "C" void UFC23_Example_Loop()
{
    while( 1 )
    {
        if( interruptAsserted )
        {
            if( ufc23.update() == RESULT_OK )
            {
                // Print the averaged hit sums
                uint8_t amountMultiHitMeas = ufc23.getAverageHitNs(tofAvgUp, tofAvgDn);
                if( amountMultiHitMeas )
                {
                    float tofDiffPs = (tofAvgUp[0] - tofAvgDn[0]) * UFC23_NS_TO_PS;
                    InsertPointInArray(tofDiffPs, tofDiff, &arrayInsertionPoint, AMOUNT_POINTS_STATISTICS, &arrayFull);
                    uint16_t amountOfPoints = arrayFull ? AMOUNT_POINTS_STATISTICS : arrayInsertionPoint;
                    float mean, std;
                    CalculateStatistics(tofDiff, amountOfPoints, &mean, &std);

                    sprintf(messageBuffer, "TofDiffMean[ps]:%0.0f\tTofDiffSTD[ps]:%0.0f\t", mean, std);
                    SerialPrint(messageBuffer);
                }

                // Print the measured temperatures
                uint8_t newTemperature = ufc23.getTemperaturesSeq1DegC(&temp1, &temp2);

                if( newTemperature )
                {
                    sprintf(messageBuffer, "\tTemp1[degC]:%0.2f\tTemp2[degC]:%0.2f\t", temp1, temp2);
                    SerialPrint(messageBuffer);
                }

                SerialPrint("\n");
            }
            interruptAsserted = 0;
        }
    }
}

uint8_t WaitOnInterrupt(uint16_t timeoutMs)
{
    uint8_t success = 0;
    uint32_t startMs   = HAL_GetTick();
    uint8_t timeoutElapsed = 0;

    while( !interruptAsserted && !timeoutElapsed )
    {
        timeoutElapsed = ( (HAL_GetTick() - startMs) > timeoutMs );
    }

    if( !timeoutElapsed )
    {
        success = 1;
    }
    
    return success;
}

void UFC23_HandleGpioInterrupt(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INTN_Pin)
    {
        interruptAsserted = 1;
    }
}
