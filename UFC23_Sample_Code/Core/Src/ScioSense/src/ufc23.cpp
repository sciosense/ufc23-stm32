#ifndef SCIOSENSE_UFC23_CPP
#define SCIOSENSE_UFC23_CPP

#include "ufc23.h"
#include "lib/ufc23/ScioSense_Ufc23.h"
#include "lib/io/ScioSense_IOInterface_STM32.c"
#include <cstring>

UFC23::~UFC23() { }

UFC23::UFC23()
{
    State                   = UFC23_STATE_NOT_CONNECTED;
    partId                  = NOT_INITIALIZED;
    correctionFactorHso     = 1;
    amountCycles            = 0;
    frontendStatusFlags     = 0;
    frontendErrorFlags      = 0;
    pwLsbNs                 = 0;
    tofLsbNs                = 0;
    zeroCrossCalibration    = 0;
    Ufc23_InitializeConfiguration(this);
}

void UFC23::begin(SPI_HandleTypeDef* spi, uint16_t cs_pin, GPIO_TypeDef* port)
{
    spiConfig           = { 0 };
    spiConfig.spi       = spi;
    spiConfig.cs_pin    = cs_pin;
    spiConfig.port      = port;

    io.transfer         = ScioSense_STM32_Spi_Transfer;
    io.write            = ScioSense_STM32_Spi_Write_Data;
    io.wait             = ScioSense_STM32_Wait;
    io.config           = &spiConfig;
}

static const char* ufc23_ErrorNames[UFC23_AMOUNT_FRONTEND_ERROR_FLAGS] =
{
    "TDC timeout in HCC measurement",
    "TDC timeout in temperature measurement",
    "Temperature measurement open circuit",
    "Temperature measurement short circuit",
    "USM HW error in up direction",
    "USM HW error in down direction",
    "TDC timeout in Pulse Width measurement in up direction",
    "TDC timeout in Pulse Width measurement in down direction",
    "TDC timeout in ToF measurement in up direction",
    "TDC timeout in ToF measurement in down direction",
    "Not received expected number of ToF hits in up direction",
    "Not received expected number of ToF hits in down direction",
};


static const char* ufc23_PartIds[UFC23_AMOUNT_PART_ID_TYPES] =
{
    "NOT INITIALIZED",
    "UFC18",
    "UFC23",
    "UNKNOWN DEVICE",
};

Result UFC23::reset()
{
    return Ufc23_Reset(this);
}

bool UFC23::init()
{
    Ufc23_Init(this);

    return isConnected();
}

bool UFC23::isConnected()
{
    return Ufc23_IsConnected(this);
}

Result UFC23::readConfig()
{
    return Ufc23_ReadConfig(this);
}

Result UFC23::writeConfig()
{
    return Ufc23_WriteConfig(this);
}

Result UFC23::startMeasurement()
{
    return Ufc23_StartCyclingMeasurement(this);
}

Result UFC23::stopMeasurement()
{
    return Ufc23_SetStandbyState(this);
}

Result UFC23::update()
{
    return Ufc23_Update(this);
}

uint8_t UFC23::getAmplitudeMeasurementResultsRaw(UFC23_AMP_Raw_TypeDef* amplitudesRawUp, UFC23_AMP_Raw_TypeDef* amplitudesRawDn)
{
    return Ufc23_ParseBatchAmplitudeRaw(this, amplitudesRawUp, amplitudesRawDn);
}

uint8_t UFC23::getAmplitudeMeasurementResultsV(UFC23_AMP_V_TypeDef* amplitudesVUp, UFC23_AMP_V_TypeDef* amplitudesVDn)
{
    return Ufc23_ParseBatchAmplitudeV(this, amplitudesVUp, amplitudesVDn);
}

uint8_t UFC23::getPulseWidthMeasurementResultsRaw(UFC23_PW_Raw_TypeDef* pulseWidthsRawUp, UFC23_PW_Raw_TypeDef* pulseWidthsRawDn)
{
    return Ufc23_ParseBatchPulseWidthRaw(this, pulseWidthsRawUp, pulseWidthsRawDn);
}

uint8_t UFC23::getPulseWidthMeasurementResultsRatio(UFC23_PW_Ps_TypeDef* pulseWidthsRatioUp, UFC23_PW_Ps_TypeDef* pulseWidthsRatioDn)
{
    return Ufc23_ParseBatchPulseWidthRatio(this, pulseWidthsRatioUp, pulseWidthsRatioDn);
}

uint8_t UFC23::getMultiHitSumNs(float* tofMultiHitNsUp, float* tofMultiHitNsDn)
{
    return Ufc23_ParseBatchTofMultiHitNs(this, tofMultiHitNsUp, tofMultiHitNsDn);
}

uint8_t UFC23::getMultiHitCount(uint8_t* multiHitCountUp, uint8_t* multiHitCountDn)
{
    return Ufc23_ParseBatchTofMultiHitsCount(this, multiHitCountUp, multiHitCountDn);
}

uint8_t UFC23::getVddVcc(float* vdd, float* vcc)
{
    return Ufc23_ParseBatchVddV(this, vdd, vcc);
}

uint8_t UFC23::getHighSpeedOscillatorFrequencyMhz(float* hsoMhz)
{
    return Ufc23_ParseBatchHsoMhz(this, hsoMhz);
}

uint8_t UFC23::getZeroCrossLevelV(float* zeroCrossLevel)
{
    return Ufc23_ParseBatchZcLvlV(this, zeroCrossLevel);
}

uint8_t UFC23::getTemperaturesSeq1DegC(float* temperature1DegC, float* temperature2DegC)
{
    return Ufc23_ParseTemperatureSeq1degC(this, temperature1DegC, temperature2DegC);
}

uint8_t UFC23::getTemperaturesSeq2DegC(float* temperature1DegC, float* temperature2DegC)
{
    return Ufc23_ParseTemperatureSeq2degC(this, temperature1DegC, temperature2DegC);
}

Result UFC23::getIndividualTofHitsNs(float* hitsUpNs, float* hitsDnNs, uint8_t* amountHitsUp, uint8_t* amountHitsDn)
{
    Result result = RESULT_INVALID;
    if( Ufc23_GetAmountMeasurementsInBatch(this) == 1 )
    {
        Ufc23_ParseSingleCycleUsTofHitsNs(this, hitsUpNs, hitsDnNs, amountHitsUp, amountHitsDn);
        result = RESULT_OK;
    }
    
    return result;
}

float UFC23::getSupplyVoltageV()
{
    float vdd[UFC23_AMOUNT_BUNDLES_MAX];
    float vcc[UFC23_AMOUNT_BUNDLES_MAX];
    uint8_t amountMeasurements =  Ufc23_ParseBatchVddV(this, vdd, vcc);
    
    float latestVdd = 0;
    if( amountMeasurements )
    {
        latestVdd = vdd[amountMeasurements-1];
    }
    return latestVdd;
}

uint8_t UFC23::readCommunicationFlags()
{
    uint8_t dataOut = 0;
    Result result = Ufc23_ReadRemoteCommand(this, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_CMF_RD, &dataOut, UFC23_COMMAND_RESPONSE_RC_CMF_RD_LENGTH);
    if( result != RESULT_OK )
    {
        dataOut = 1;
    }
    return dataOut;
}

uint8_t UFC23::readInterruptFlags()
{
    uint8_t dataOut = 0;
    Result result = Ufc23_ReadRemoteCommand(this, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_IF_RD, &dataOut, UFC23_COMMAND_RESPONSE_RC_IF_RD_LENGTH);
    if( result != RESULT_OK )
    {
        dataOut = 0;
    }
    return dataOut;
}

uint16_t UFC23::readFrontendErrorFlags()
{
    uint16_t dataOut = 0;
    uint8_t dataReceived[2];
    Result result = Ufc23_ReadRemoteCommand(this, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_FES_RD, dataReceived, UFC23_COMMAND_RESPONSE_RC_FES_RD_LENGTH);
    if( result == RESULT_OK )
    {
        dataOut = ((uint16_t)dataReceived[0] << 16) + ((uint16_t)dataReceived[1]);
    }
    return dataOut;
}

uint16_t UFC23::readFrontendStatusFlags()
{
    uint8_t dataOut = 0;
    Result result = Ufc23_ReadRemoteCommand(this, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_EF_RD, &dataOut, UFC23_COMMAND_RESPONSE_RC_EF_RD_LENGTH);
    if( result != RESULT_OK )
    {
        dataOut = 0;
    }
    return dataOut;
}

uint8_t UFC23::readSystemStatusFlags()
{
    uint8_t dataOut = 0;
    Result result = Ufc23_ReadRemoteCommand(this, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_SSF_RD, &dataOut, UFC23_COMMAND_RESPONSE_RC_SSF_RD_LENGTH);
    if( result != RESULT_OK )
    {
        dataOut = 0;
    }
    return dataOut;
}

uint8_t UFC23::getAverageHitNs(float* tofAvgHitNsUp, float* tofAvgHitNsDn)
{
    float tofMultiHitNsUp[UFC23_AMOUNT_BUNDLES_MAX], tofMultiHitNsDn[UFC23_AMOUNT_BUNDLES_MAX];
    uint8_t amountMultiHitSums = Ufc23_ParseBatchTofMultiHitNs(this, tofMultiHitNsUp, tofMultiHitNsDn);
    uint8_t multiHitCountUp[UFC23_AMOUNT_BUNDLES_MAX], multiHitCountDn[UFC23_AMOUNT_BUNDLES_MAX];
    Ufc23_ParseBatchTofMultiHitsCount(this, multiHitCountUp, multiHitCountDn);

    for( uint8_t idx = 0; idx < amountMultiHitSums; idx++ )
    {
        float amountHits = (float)this->Param.CR_B0.C_TOF_MULTIHIT_NO;
        if( multiHitCountUp[idx] < this->Param.CR_B0.C_TOF_MULTIHIT_NO )
        {
            amountHits = (float)multiHitCountUp[idx];
        }
        tofAvgHitNsUp[idx] = tofMultiHitNsUp[idx] / amountHits;

        amountHits = (float)this->Param.CR_B0.C_TOF_MULTIHIT_NO;
        if( multiHitCountDn[idx] < this->Param.CR_B0.C_TOF_MULTIHIT_NO )
        {
            amountHits = (float)multiHitCountDn[idx];
        }
        tofAvgHitNsDn[idx] = tofMultiHitNsDn[idx] / amountHits;
    }

    return amountMultiHitSums;
}

uint16_t UFC23::clearFlags()
{
    return Ufc23_ClearFlagRegisters(this);
}

Result UFC23::triggerSingleMeasurement(Ufc23_CycleTaskRequest measurementsToTrigger)
{
    return Ufc23_TriggerSingleMeasurement(this, measurementsToTrigger);
}

Result UFC23::sendRemoteCommand(uint8_t remoteCommand, uint16_t extendedCommand)
{
    return Ufc23_WriteRemoteCommand(this, remoteCommand, extendedCommand);
}

uint32_t UFC23::readRamAddress(uint16_t ramAddress)
{
    uint32_t dataOut = 0;
    uint8_t registerContents[4];
    if( Ufc23_ReadDWordRAM(this, ramAddress, registerContents, 1) == RESULT_OK )
    {
        dataOut = Ufc23_ByteArrayToDWord(registerContents, 0);
    }
    return dataOut;
}

uint32_t UFC23::writeRamAddress(uint16_t ramAddress, uint32_t registerContent)
{
    return Ufc23_WriteDWordRAM(this, ramAddress, registerContent);
}

float UFC23::getHighSpeedClockFrequencyHz()
{
    float hsoMhz[UFC23_AMOUNT_BUNDLES_MAX];
    uint8_t amountMeasurements =  Ufc23_ParseBatchHsoMhz(this, hsoMhz);
    
    float latestHsoMhz = 0;
    if( amountMeasurements )
    {
        latestHsoMhz = hsoMhz[amountMeasurements-1];
    }
    return latestHsoMhz;
}

Result UFC23::triggerSpoolPortOpenCheck()
{
    return Ufc23_TriggerTransducerPortOpenMeasurement(this);
}

bool UFC23::isMeasuring()
{
    bool isMeasuring = false;
    if( Ufc23_GetMode(this) == RESULT_OK )
    {
        isMeasuring = ( this->State == UFC23_STATE_MEAS );
    }
    return isMeasuring;
}

bool UFC23::hasError()
{
    return (bool)Ufc23_ErrorsPresentInLastUpdate(this);
}

uint16_t UFC23::getErrors()
{
    return Ufc23_ErrorsPresentInLastUpdate(this);
}

const char* UFC23::partIdToString(Ufc23_PartID partId)
{
    uint8_t partIdIdx = (uint8_t)partId;
    if( partIdIdx >= UFC23_AMOUNT_PART_ID_TYPES )
    {
        partIdIdx = UFC23_AMOUNT_PART_ID_TYPES - 1;
    }
    return ufc23_PartIds[partIdIdx];
}

uint8_t UFC23::errorToStrings(uint32_t errorRegisterContent, char errorStrings[UFC23_AMOUNT_FRONTEND_ERROR_FLAGS][ERROR_STRING_LENGTH])
{
    uint8_t amountErrors = 0;
    for( uint8_t idx=0; idx<UFC23_AMOUNT_FRONTEND_ERROR_FLAGS; idx++ )
    {
        if( (1<<idx) & errorRegisterContent )
        {
            strcpy(errorStrings[amountErrors], ufc23_ErrorNames[idx]);
            amountErrors++;
        }
    }
    return amountErrors;
}

bool UFC23::isSpoolWorkingWell()
{
    bool isSpoolWorking = 0; 
    if( Ufc23_CheckTransducerPortOpenMeasurement(this) == RESULT_OK )
    {
        isSpoolWorking = 1;
    }
    return isSpoolWorking;
}

void UFC23::updateConfiguration()
{
    Ufc23_UpdateConfiguration(this);
}

void UFC23::setConfigurationRegisters(uint32_t* configurationRegisters)
{
    Ufc23_SetConfigurationRegisters(this, configurationRegisters);
}

#endif //SCIOSENSE_UFC23_CPP
