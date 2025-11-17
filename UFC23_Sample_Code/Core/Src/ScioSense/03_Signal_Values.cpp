#include "Example_Definitions.h"
#include "UFC23_Utils.h"
#include "src/ScioSense_UFC23.h"
#include <cstdio>
#include <cstring>

static char messageBuffer[128];         // Buffer for sending data through Serial
uint8_t interruptAsserted = 0;

static UFC23 ufc23;

UFC23_AMP_V_TypeDef ampUp[UFC23_AMOUNT_BUNDLES_MAX], ampDn[UFC23_AMOUNT_BUNDLES_MAX];
UFC23_PW_Ps_TypeDef pwRatioUp[UFC23_AMOUNT_BUNDLES_MAX], pwRatioDn[UFC23_AMOUNT_BUNDLES_MAX];
uint8_t amountHitsUp[UFC23_AMOUNT_TOF_HITS_MEAS], amountHitsDn[UFC23_AMOUNT_TOF_HITS_MEAS];
float zeroCrossLevels[UFC23_AMOUNT_BUNDLES_MAX];

extern "C" void UFC23_Example_Setup(UART_HandleTypeDef *uart, SPI_HandleTypeDef *spi)
{
    SetUartHandle(uart);
    
    /* Wait to allow terminal software to capture the output */
    HAL_Delay(2000);

    SerialPrint("\nStarting UFC23 03_Signal_Values demo on STM32...\n");

    HAL_Delay(UFC23_T_RC_RLS_MS);

    ufc23.begin(spi, SSN_Pin, SSN_GPIO_Port);

    while( !ufc23.init())
    {
        SerialPrint("UFC23 initialization failed\n");
        HAL_Delay(1000);        
    }

    SerialPrint(ufc23.partIdToString(ufc23.partId));
    SerialPrint(" initialized properly\n");

    // Single ended configuration
    uint32_t configRegisters[UFC23_AMOUNT_CONFIGURATION_REGISTERS] =
    {
        0x0000001C,     // A0
        0x00000FF1,     // A1
        0x000006DB,     // A2
        0x00000010,     // A3
        0x000017AF,     // A4
        0x0000B100,     // A5
        0x00001249,     // A6
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

    // Measure the supply voltage
    ufc23.triggerSingleMeasurement(VCC_MEAS);
    interruptAsserted = 0;
    if( WaitOnInterrupt(30) )
    {
        ufc23.update();
        float vdd[UFC23_AMOUNT_BUNDLES_MAX], vcc[UFC23_AMOUNT_BUNDLES_MAX];
        if( ufc23.getVddVcc(vdd, vcc) )
        {
            sprintf(messageBuffer, "VDD: %0.3f mV\nVCC: %0.3f mV\n", vdd[0], vcc[0]);
            SerialPrint(messageBuffer);
        }
    }

    // Check the spool connection
    interruptAsserted = 0;
    ufc23.triggerSpoolPortOpenCheck();
    if( WaitOnInterrupt(100) )
    {
        if( ufc23.isSpoolWorkingWell() )
        {
          SerialPrint("Spool working well\n");
        }
        else
        {
          SerialPrint("Spool is not working well. Please check the connections\n");
        }
    }

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
            uint8_t dataPrinted = 0;
            if( ufc23.update() == RESULT_OK )
            {
                // Print the amount of received hits
                uint8_t amountMultiHitMeas = ufc23.getMultiHitCount(amountHitsUp, amountHitsDn);
                if( amountMultiHitMeas )
                {
                    sprintf(messageBuffer, "HitsUp:%d\tHitsDn:%d\t", amountHitsUp[0], amountHitsDn[0]);
                    SerialPrint(messageBuffer);
                    dataPrinted = 1;
                }
                
                if( ufc23.getZeroCrossLevelV(zeroCrossLevels) )
                {
                    sprintf(messageBuffer, "ZeroCrossLevel[V]:%0.3f\t", zeroCrossLevels[0]);
                    SerialPrint(messageBuffer);
                    dataPrinted = 1;
                }
    
                // Print the received amplitudes
                uint8_t amountAmplitudeMeasurementsV = ufc23.getAmplitudeMeasurementResultsV(ampUp, ampDn);
                if( amountAmplitudeMeasurementsV )
                {
                    sprintf(messageBuffer, "AmpUp1[mV]:%0.0f\tAmpUp2[mV]:%0.0f\tAmpUp3[mV]:%0.0f\t", ampUp[0].AMPL1*1000, ampUp[0].AMPL2*1000, ampUp[0].AMPL3*1000);
                    SerialPrint(messageBuffer);
                    sprintf(messageBuffer, "AmpDn1[mV]:%0.0f\tAmpDn2[mV]:%0.0f\tAmpDn3[mV]:%0.0f\t", ampDn[0].AMPL1*1000, ampDn[0].AMPL2*1000, ampDn[0].AMPL3*1000);
                    SerialPrint(messageBuffer);
                    dataPrinted = 1;
                }
                
                // Print the received pulse widths
                uint8_t amountPwRatioMeasurements = ufc23.getPulseWidthMeasurementResultsRatio(pwRatioUp, pwRatioDn);
                if( amountPwRatioMeasurements )
                {
                    sprintf(messageBuffer, "PWRatioUp1:%0.1f\tPWRatioUp2:%0.1f\t", pwRatioUp[0].PW1_FHL, pwRatioUp[0].PW2_FHL);
                    SerialPrint(messageBuffer);
                    sprintf(messageBuffer, "PWRatioDn1:%0.1f\tPWRatioDn2:%0.1f\t", pwRatioDn[0].PW1_FHL, pwRatioDn[0].PW2_FHL);
                    SerialPrint(messageBuffer);
                    dataPrinted = 1;
                }
            }
            else
            {
                if( ufc23.hasError() )
                {
                    char errorStrings[UFC23_AMOUNT_FRONTEND_ERROR_FLAGS][ERROR_STRING_LENGTH];
                    uint8_t amountFoundErrors = ufc23.errorToStrings(ufc23.getErrors(), errorStrings);
                    for( uint8_t error = 0; error < amountFoundErrors; error++ )
                    {
                        SerialPrint(errorStrings[error]);
                        SerialPrint("\n");
                    }
                    dataPrinted = 1;
                }
            }

            if( dataPrinted )
            {
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
