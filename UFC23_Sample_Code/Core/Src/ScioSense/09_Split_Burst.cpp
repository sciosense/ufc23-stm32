#include "Example_Definitions.h"
#include "UFC23_Utils.h"
#include "src/ScioSense_UFC23.h"
#include <cstdio>
#include <cstring>

#define TOF_SHIFT_THRESHOLD_NS  10.0    // How much the time between zero crossings have to change for it to be considered a product of the split burst

static char messageBuffer[128];         // Buffer for sending data through Serial
uint8_t interruptAsserted = 0;

UFC23 ufc23;

uint8_t amountHitsUp[UFC23_AMOUNT_TOF_HITS_MEAS], amountHitsDn[UFC23_AMOUNT_TOF_HITS_MEAS];
float tofAvgUp[UFC23_AMOUNT_TOF_HITS_MEAS], tofAvgDn[UFC23_AMOUNT_TOF_HITS_MEAS];
float tofHitsUp[UFC23_AMOUNT_TOF_HITS_MEAS], tofHitsDn[UFC23_AMOUNT_TOF_HITS_MEAS];

uint8_t totalAmountHits;
uint8_t nominalIdxPhaseShift;
uint8_t tofSumHits;
uint8_t tofMultihitStart;

extern "C" void UFC23_Example_Setup(UART_HandleTypeDef *uart, SPI_HandleTypeDef *spi)
{
    SetUartHandle(uart);

    /* Wait to allow terminal software to capture the output */
    HAL_Delay(2000);

    SerialPrint("\nStarting UFC23 09_Split_Burst demo on STM32...\n");

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
        0x0000170F,     // A4
        0x0000B100,     // A5
        0x00001249,     // A6
        0x000194F4,     // A7
        0x00000000,     // A8
        0x04900000,     // A9
        0xC00D0034,     // AA
        0x0002140E,     // AB
        0x00000000,     // AC
        0x0808B00E,     // AD
        0x46301024,     // AE
        0x0FFFFFFF,     // AF
        0x00014268,     // B0
        0x20412424,     // B1
        0x00000000      // B2
    };

    ufc23.setConfigurationRegisters(configRegisters);

    tofMultihitStart        = ufc23.Param.CR_B0.C_TOF_MULTIHIT_START;
    tofSumHits              = ufc23.Param.CR_B0.C_TOF_MULTIHIT_NO;
    totalAmountHits         = ufc23.Param.CR_B0.C_TOF_HIT_NO;
    nominalIdxPhaseShift    = ufc23.Param.CR_AB.C_FBG_FBSP;

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
                    // Print the individual hits
                    if( ufc23.getIndividualTofHitsNs(tofHitsUp, tofHitsDn, amountHitsUp, amountHitsDn) == RESULT_OK )
                    {
                        uint8_t amountHitsAvgTofStep = tofSumHits;
                        uint8_t minAmountReceivedHits = amountHitsUp[0];
                        if( amountHitsDn[0] < amountHitsUp[0] )
                        {
                            minAmountReceivedHits = amountHitsDn[0];
                        }
                        if( minAmountReceivedHits < tofSumHits )
                        {
                            amountHitsAvgTofStep = minAmountReceivedHits;
                        }

                        float avgTofStepUp = ( tofHitsUp[amountHitsAvgTofStep - 1] - tofHitsUp[0] ) / (float)(amountHitsAvgTofStep - 1);
                        float avgTofStepDn = ( tofHitsDn[amountHitsAvgTofStep - 1] - tofHitsDn[0] ) / (float)(amountHitsAvgTofStep - 1);

                        uint8_t phaseShiftIdxUp = 0;
                        for( uint8_t hitIdx = 1; hitIdx < amountHitsUp[0]; hitIdx++ )
                        {
                            if( abs(tofHitsUp[hitIdx] - tofHitsUp[hitIdx - 1] - avgTofStepUp) > (TOF_SHIFT_THRESHOLD_NS) )
                            {
                                phaseShiftIdxUp = hitIdx;
                                break;
                            }
                        }
                        uint8_t phaseShiftIdxDn = 0;
                        for( uint8_t hitIdx = 1; hitIdx < amountHitsDn[0]; hitIdx++ )
                        {
                            if( abs(tofHitsDn[hitIdx] - tofHitsDn[hitIdx - 1] - avgTofStepDn) > (TOF_SHIFT_THRESHOLD_NS) )
                            {
                                phaseShiftIdxDn = hitIdx;
                                break;
                            }
                        }

                        float stepsFromAvgToStart = (float)(tofSumHits - 1) / 2.0 + (float)tofMultihitStart;
                        float stepsCorrectionFromPhaseUp = (float)(nominalIdxPhaseShift - phaseShiftIdxUp);
                        float stepsCorrectionFromPhaseDn = (float)(nominalIdxPhaseShift - phaseShiftIdxDn);

                        float correctedTofUp = tofAvgUp[0] - (stepsFromAvgToStart + stepsCorrectionFromPhaseUp) * avgTofStepUp;
                        float correctedTofDn = tofAvgDn[0] - (stepsFromAvgToStart + stepsCorrectionFromPhaseDn) * avgTofStepDn;

                        sprintf(messageBuffer, "AvgTofUp[ns]:%0.2f\tCorrectedTofUp[ns]:%0.2f\t", tofAvgUp[0], correctedTofUp);
                        SerialPrint(messageBuffer);
                        sprintf(messageBuffer, "AvgTofDn[ns]:%0.2f\tCorrectedTofDn[ns]:%0.2f\tTofDiff[ns]:%0.3f\n", tofAvgDn[0], correctedTofDn, correctedTofUp - correctedTofDn);
                        SerialPrint(messageBuffer);

                        for( uint8_t hitIdx = 0; hitIdx < amountHitsUp[0]; hitIdx++ )
                        {
                            float deltaUp = 0;
                            float deltaDn = 0;
                            if( hitIdx > 0 )
                            {
                                deltaUp = tofHitsUp[hitIdx] - tofHitsUp[hitIdx-1];
                                deltaDn = tofHitsDn[hitIdx] - tofHitsDn[hitIdx-1];
                            }
                            sprintf(messageBuffer, "Hit:%d\tHitUp[ns]:%0.2f\tDeltaUp[ns]:%0.2f\t", hitIdx, tofHitsUp[0], deltaUp);
                            SerialPrint(messageBuffer);
                            sprintf(messageBuffer, "HitDn[ns]:%0.2f\tDeltaDn[ns]:%0.2f", tofHitsDn[hitIdx], deltaDn);
                            SerialPrint(messageBuffer);
                            
                            if( hitIdx == phaseShiftIdxUp )
                            {
                                SerialPrint("\tPhaseShiftUp");
                            }
                            if( hitIdx == phaseShiftIdxDn )
                            {
                                SerialPrint("\tPhaseShiftDn");
                            }
                            SerialPrint("\n");
                        }
                    }
                }
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
