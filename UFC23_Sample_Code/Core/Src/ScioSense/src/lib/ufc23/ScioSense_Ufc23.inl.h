#ifndef SCIOSENSE_UFC23_INL_C_H
#define SCIOSENSE_UFC23_INL_C_H

#include "ScioSense_Ufc23.h"
#include "ScioSense_Ufc23_defines.h"
#include "ScioSense_Ufc23_Macros.h"

#define read(dataToWrite, lenToWrite, dataToRead, lenToRead)    ufc23->io.read(ufc23->io.config, (uint8_t*)(dataToWrite), (lenToWrite), (uint8_t*)(dataToRead), (lenToRead))
#define write(data, len)                                        ufc23->io.write(ufc23->io.config, (uint8_t*)(data), (len))
#define wait(ms)                                                ufc23->io.wait(ms)

static inline Result Ufc23_ReadRemoteCommand(ScioSense_Ufc23* ufc23, uint8_t remoteCommand, uint16_t extendedCommand, uint8_t* dataToRead, uint16_t dataToReadSize)
{
    uint8_t valuesToWrite[2];
    valuesToWrite[0] =  ( (remoteCommand    << UFC23_REMOTE_COMMAND_INDEX)      & UFC23_REMOTE_COMMAND_MASK     ) | 
                        ( (extendedCommand  >> UFC23_EXTENDED_COMMAND_INDEX_0)  & UFC23_REMOTE_EXTENDED_MASK_0  );
    valuesToWrite[1] =  ( (extendedCommand  << UFC23_EXTENDED_COMMAND_INDEX_1)  & UFC23_REMOTE_EXTENDED_MASK_1  );

    return (Result)read(valuesToWrite, 2, dataToRead, dataToReadSize);
}

static inline Result Ufc23_WriteRemoteCommand(ScioSense_Ufc23* ufc23, uint8_t remoteCommand, uint16_t extendedCommand)
{
    uint8_t valuesToWrite[2];
    valuesToWrite[0] =  ( (remoteCommand    << UFC23_REMOTE_COMMAND_INDEX)      & UFC23_REMOTE_COMMAND_MASK     ) | 
                        ( (extendedCommand  >> UFC23_EXTENDED_COMMAND_INDEX_0)  & UFC23_REMOTE_EXTENDED_MASK_0  );
    valuesToWrite[1] =  ( (extendedCommand  << UFC23_EXTENDED_COMMAND_INDEX_1)  & UFC23_REMOTE_EXTENDED_MASK_1  );

    return (Result)write(valuesToWrite, 2);
}

static inline Result Ufc23_WriteDWordRAM(ScioSense_Ufc23* ufc23, uint16_t RAMAddress, UFC23_REG_SIZE registerContent)
{
    Result result = RESULT_IO_ERROR;
    
    if( (RAMAddress >= UFC23_RAM_USM_ADDRESS_START) && (RAMAddress <= UFC23_RAM_CONFIG_REGISTER_ADDRESS_END) )
    {
        uint8_t arrayToWrite[6];
    
        arrayToWrite[0] =   ( (UFC23_REMOTE_COMMAND_RC_RAA_WR   << UFC23_REMOTE_COMMAND_INDEX)  & UFC23_REMOTE_COMMAND_MASK ) | 
                            ( (RAMAddress                       >> UFC23_RAM_ADDRESS_INDEX_0)   & UFC23_RAM_ADDRESS_MASK_0 );
        arrayToWrite[1] =   ( (RAMAddress                       << UFC23_RAM_ADDRESS_INDEX_1)   & UFC23_RAM_ADDRESS_MASK_1 );
        
        // Format the array as Big Endian for the UFC23
        arrayToWrite[2] = (uint8_t)( (registerContent >> 24 ) & 0x000000FF );
        arrayToWrite[3] = (uint8_t)( (registerContent >> 16 ) & 0x000000FF );
        arrayToWrite[4] = (uint8_t)( (registerContent >> 8  ) & 0x000000FF );
        arrayToWrite[5] = (uint8_t)( (registerContent       ) & 0x000000FF );
    
        result = write(arrayToWrite, sizeof(arrayToWrite) / sizeof(arrayToWrite[0]));
    }
    else
    {
        result = RESULT_NOT_ALLOWED;
    }

    return result;
}

static inline uint32_t Ufc23_ByteArrayToDWord(uint8_t* byteArray, uint16_t startIndex)
{
    uint32_t dWordContent = 
            ( ((uint32_t)(byteArray[startIndex]))      << 24   ) |
            ( ((uint32_t)(byteArray[startIndex + 1]))  << 16   ) |
            ( ((uint32_t)(byteArray[startIndex + 2]))  << 8    ) |
            ( ((uint32_t)(byteArray[startIndex + 3]))          );
    
    return dWordContent;
}

static inline uint32_t Ufc23_ParseUsmBatchDWordValue(ScioSense_Ufc23* ufc23, uint8_t bundleIdx, uint8_t dWordIdxInBatch)
{
    uint32_t dWordContent = 0;
    if( (bundleIdx < UFC23_AMOUNT_BUNDLES_MAX) && (dWordIdxInBatch < UFC23_BUNDLE_CYCLE_LENGTH) )
    {
        uint16_t usmIdx = bundleIdx * UFC23_BUNDLE_CYCLE_LENGTH + dWordIdxInBatch;
        dWordContent = Ufc23_ParseUsmDWordValue(ufc23, usmIdx);
    }
    return dWordContent;
}

static inline uint32_t Ufc23_ParseUsmDWordValue(ScioSense_Ufc23* ufc23, uint16_t dWordIdx)
{
    uint32_t dWordContent = 0;
    if( dWordIdx < UFC23_AMOUNT_USM_BATCH_REGISTERS )
    {
        uint16_t byteArrayIdx = 4 * dWordIdx;
        // Format the array as Big Endian from the UFC23
        dWordContent = Ufc23_ByteArrayToDWord(ufc23->DataBuffer, byteArrayIdx);
    }
    return dWordContent;
}

static inline Result Ufc23_ReadDWordRAM(ScioSense_Ufc23* ufc23, uint16_t RAMAddress, UFC23_COM_SIZE* registerContents, uint16_t registersToRead)
{
    Result result = RESULT_IO_ERROR;

    if( ((RAMAddress >= UFC23_RAM_USM_ADDRESS_START) && (RAMAddress <= UFC23_RAM_CONFIG_REGISTER_ADDRESS_END)) || (RAMAddress == UFC23_CR_SR_DEVICE_ID_ADDRESS) )
    {
        uint8_t valuesToWrite[UFC23_COMMAND_BYTES];
        valuesToWrite[0] =  ( (UFC23_REMOTE_COMMAND_RC_RAA_RD    << UFC23_REMOTE_COMMAND_INDEX)      & UFC23_REMOTE_COMMAND_MASK ) | 
        ( (RAMAddress  >> UFC23_RAM_ADDRESS_INDEX_0)  & UFC23_RAM_ADDRESS_MASK_0 );
        valuesToWrite[1] =  ( (RAMAddress  << UFC23_RAM_ADDRESS_INDEX_1)  & UFC23_RAM_ADDRESS_MASK_1 );

        result = read(valuesToWrite, UFC23_COMMAND_BYTES, registerContents, 4 * registersToRead);
    }
    else
    {
        result = RESULT_NOT_ALLOWED;
    }
    
    return result;
}

static inline uint16_t Ufc23_GetConfigurationRegisterAddress(ScioSense_Ufc23* ufc23, uint8_t idx)
{
    if ( idx >= UFC23_AMOUNT_CONFIGURATION_REGISTERS)
    {
        return 0;
    }
    return ufc23->Addresses[idx];
}

static inline uint32_t Ufc23_GetConfigurationRegisterSetting(ScioSense_Ufc23* ufc23, uint8_t idx)
{
    if ( idx >= UFC23_AMOUNT_CONFIGURATION_REGISTERS)
    {
        return 0;
    }
    return ufc23->CR[idx];
}

static inline Result Ufc23_Reset(ScioSense_Ufc23* ufc23)
{
    Result result = Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_RM_REQ, UFC23_EXTENDED_COMMAND_EC_SYS_RST);
    
    if( result == RESULT_OK )
    {
        ufc23->State = UFC23_STATE_RESET;
    }

    return result;
}

static inline Result Ufc23_Init(ScioSense_Ufc23* ufc23)
{
    Result result = Ufc23_Reset(ufc23);

    if( result == RESULT_OK )
    {
        wait(UFC23_T_RC_RLS_MS);
    
        result = Ufc23_DetectEndBootLoadSequence(ufc23);
        
        if( result == RESULT_OK )
        {
            Ufc23_GetPartId(ufc23);

            Ufc23_ReadConfig(ufc23);

            uint8_t retry       = 0;
            uint8_t maxRetries  = 2;
            uint8_t hsoMeasured = 0;
            while( (!hsoMeasured) && (retry < maxRetries) )
            {
                wait(UFC23_T_MM_RLS_MS);

                result = Ufc23_GetUSMData(ufc23);

                if( result == RESULT_OK )
                {
                    float fHsoMhz[UFC23_AMOUNT_BUNDLES_MAX];
                    hsoMeasured = Ufc23_ParseBatchHsoMhz(ufc23, fHsoMhz);
                }
                retry++;
            }

            if( hsoMeasured == 0 )
            {
                result = RESULT_TIMEOUT;
            }
        }
    }

    return result;
}

static inline uint8_t Ufc23_IsConnected(ScioSense_Ufc23* ufc23)
{
    return Ufc23_IsPartIdValid(ufc23);
}

static inline UFC23_FR_SIZE Ufc23_GetCommunicationFlagRegister(ScioSense_Ufc23* ufc23)
{
    uint8_t dataRead[UFC23_COMMAND_RESPONSE_RC_CMF_RD_LENGTH];
    Result result = Ufc23_ReadRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_CMF_RD, dataRead, UFC23_COMMAND_RESPONSE_RC_CMF_RD_LENGTH);
    if ( result == RESULT_OK )
    {
        return dataRead[0];
    }
    else
    {
        return 0;
    }
}

static inline UFC23_FR_SIZE Ufc23_GetInterruptFlagRegister(ScioSense_Ufc23* ufc23)
{
    uint8_t dataRead[UFC23_COMMAND_RESPONSE_RC_IF_RD_LENGTH];
    Result result = Ufc23_ReadRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_IF_RD, dataRead, UFC23_COMMAND_RESPONSE_RC_IF_RD_LENGTH);
    if ( result == RESULT_OK )
    {
        return dataRead[0];
    }
    else
    {
        return 0;
    }
}

static inline UFC23_FR_FE_SIZE Ufc23_GetFrontendErrorFlagRegister(ScioSense_Ufc23* ufc23)
{
    UFC23_FR_FE_SIZE errorFlags = 0xFFFF;
    uint8_t dataRead[UFC23_COMMAND_RESPONSE_RC_FES_RD_LENGTH];
    Result result = Ufc23_ReadRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_FES_RD, dataRead, UFC23_COMMAND_RESPONSE_RC_FES_RD_LENGTH);
    
    if ( result == RESULT_OK )
    {
        errorFlags = (((uint16_t)dataRead[0]) << 8) + (uint16_t)dataRead[1];
        ufc23->frontendErrorFlags = errorFlags;
    }
    return errorFlags;
}

static inline UFC23_FR_SIZE Ufc23_GetFrontendStatusFlagRegister(ScioSense_Ufc23* ufc23)
{
    uint8_t dataRead[UFC23_COMMAND_RESPONSE_RC_EF_RD_LENGTH];
    Result result = Ufc23_ReadRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_EF_RD, dataRead, UFC23_COMMAND_RESPONSE_RC_EF_RD_LENGTH);
    if ( result == RESULT_OK )
    {
        return dataRead[0];
    }
    else
    {
        return 0;
    }
}

static inline UFC23_FR_SIZE Ufc23_GetSystemStatusFlagRegister(ScioSense_Ufc23* ufc23)
{
    uint8_t dataRead[UFC23_COMMAND_RESPONSE_RC_SSF_RD_LENGTH];
    Result result = Ufc23_ReadRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_RD, UFC23_EXTENDED_COMMAND_RC_SSF_RD, dataRead, UFC23_COMMAND_RESPONSE_RC_SSF_RD_LENGTH);
    if ( result == RESULT_OK )
    {
        return dataRead[0];
    }
    else
    {
        return 0;
    }
}

static inline Result Ufc23_ClearFlagRegisters(ScioSense_Ufc23* ufc23)
{
    uint16_t extendedCommand =  UFC23_EXTENDED_COMMAND_EC_CMF_CLR   | 
                                UFC23_EXTENDED_COMMAND_EC_IF_CLR    | 
                                UFC23_EXTENDED_COMMAND_EC_EF_CLR    | 
                                UFC23_EXTENDED_COMMAND_EC_FES_CLR;
    
    // Clear the flags    
    Result result = Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_CLR, extendedCommand);
    wait(UFC23_PAUSE_BETWEEN_COMMANDS_MS);
    // Set the register to zero so that the flags can be asserted again
    result |= Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_FRU_CLR, 0);
    return result;
}

static inline Result Ufc23_DetectEndBootLoadSequence(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;

    UFC23_FR_SIZE communicationErrors   = Ufc23_GetCommunicationFlagRegister(ufc23);
    UFC23_FR_SIZE interruptFlagRegister = Ufc23_GetInterruptFlagRegister(ufc23);

    uint8_t bootCompleted = interruptFlagRegister & UFC23_IF_BOOTLOAD_SEQUENCE_DONE;
    uint8_t errorsMeasurement = interruptFlagRegister & (UFC23_IF_USM_PAUSE_ERR | UFC23_IF_TASK_TIMEOUT | UFC23_IF_ERROR_DETECTED);
    
    if( bootCompleted && !(communicationErrors || errorsMeasurement) )
    {
        result = RESULT_OK;
        ufc23->State = UFC23_STATE_STANDBY;
    }

    Ufc23_ClearFlagRegisters(ufc23);

    return result;
}

static inline Result Ufc23_TriggerSingleMeasurement(ScioSense_Ufc23* ufc23, Ufc23_CycleTaskRequest requestType)
{
    Result result = Ufc23_SetStandbyState(ufc23);
    if( ufc23->State == UFC23_STATE_STANDBY )
    {
        // Enable interrupts on Measure Cycle Done
        uint32_t enabledInterrupts = UFC23_C_IRQ_EN_MCS_DONE_SET(UFC23_C_IRQ_EN_MCS_DONE_ENABLED);
        result = Ufc23_WriteDWordRAM(ufc23, UFC23_CR_FRU_IFH_ADDRESS, enabledInterrupts);
        
        if( result == RESULT_OK )
        {
            result |= Ufc23_EnableMeasureModeTimerHalted(ufc23);
            result |= Ufc23_ClearFlagRegisters(ufc23);
            result |= Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_CTASK_REQ, (uint16_t)requestType);
        }
    }

    return result;
}

static inline Result Ufc23_TriggerTransducerPortOpenMeasurement(ScioSense_Ufc23* ufc23)
{
    Result result = Ufc23_SetStandbyState(ufc23);
    if( result == RESULT_OK )
    {
        // Enable interrupts on Service Task Done
        uint32_t enabledInterrupts = UFC23_C_IRQ_EN_STASK_DONE_SET(UFC23_C_IRQ_EN_STASK_DONE_ENABLED);
        result |= Ufc23_WriteDWordRAM(ufc23, UFC23_CR_FRU_IFH_ADDRESS, enabledInterrupts);
        
        // Enable hardware error flags
        uint32_t hardwareError = UFC23_C_EF_EN_USM_HW_UP_ERR_SET(UFC23_C_EF_EN_USM_HW_UP_ERR_ENABLED) | UFC23_C_EF_EN_USM_HW_DN_ERR_SET(UFC23_C_EF_EN_USM_HW_UP_ERR_ENABLED);
        result |= Ufc23_WriteDWordRAM(ufc23, UFC23_CR_FRU_EFH_ADDRESS, hardwareError);

        // Configure the resistance of the transducer
        uint32_t crFepAnaCtrl2 = 0x57F01024;
        uint8_t configuredResistanceRx = UFC23_C_RMSET_RX_GET(crFepAnaCtrl2);
        uint8_t configuredResistanceTx = UFC23_C_RMSET_TX_GET(crFepAnaCtrl2);
        
        uint8_t spoolCheckResistanceRx = configuredResistanceRx >> 1;
        uint8_t spoolCheckResistanceTx = configuredResistanceTx >> 1;
        
        crFepAnaCtrl2 &= ~(UFC23_C_RMSET_RX_Msk | UFC23_C_RMSET_TX_Msk);
        crFepAnaCtrl2 |= UFC23_C_RMSET_TX_SET(spoolCheckResistanceRx) | UFC23_C_RMSET_RX_SET(spoolCheckResistanceTx);

        result |= Ufc23_WriteDWordRAM(ufc23, UFC23_CR_USM_ANA_CTRL2_ADDRESS, crFepAnaCtrl2);

        // Configure the burst pulse
        uint32_t usmFbgMctrlConfig = ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] | UFC23_C_FBG_FBNUM_SET(UFC23_C_FBG_FBNUM_MAX_PULSES);
        result |= Ufc23_WriteDWordRAM(ufc23, UFC23_CR_USM_FBG_MCTRL_ADDRESS, usmFbgMctrlConfig);

        if( result == RESULT_OK )
        {
            result |= Ufc23_ClearFlagRegisters(ufc23);
            result |= Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_STASK_REQ, UFC23_EXTENDED_COMMAND_EC_EHSP_REQ);
        }
    }
    
    return result;
}

static inline Result Ufc23_CheckTransducerPortOpenMeasurement(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;

    if( Ufc23_GetInterruptFlagRegister(ufc23) & UFC23_IF_SERVICE_TASK_REQUEST_DONE )
    {
        UFC23_FR_FE_SIZE frontEndErrorFlags = Ufc23_GetFrontendErrorFlagRegister(ufc23);
        uint8_t spoolNotDetected =  frontEndErrorFlags & (UFC23_FES_USM_HW_ERR_UP | UFC23_FES_USM_HW_ERR_DN);

        // Rewrite the resistance values for normal operation
        Ufc23_WriteDWordRAM(ufc23, UFC23_CR_USM_ANA_CTRL2_ADDRESS, ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]);

        if( spoolNotDetected )
        {
            result = RESULT_TIMEOUT;
        }
        else
        {
            result = RESULT_OK;
        }
    }
    
    Ufc23_ClearFlagRegisters(ufc23);

    return result;
}

static inline Result Ufc23_SetStandbyState(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;
    Ufc23_GetMode(ufc23);
    uint8_t maxRetries = 2;
    uint8_t retry = 0;
    while( (ufc23->State != UFC23_STATE_STANDBY) && (retry < maxRetries) )
    {
        result &= Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MM_ENA_DISABLED);

        if( result == RESULT_OK )
        {
            wait(UFC23_SWITCH_MEAS_MODE_MS);
    
            Ufc23_GetMode(ufc23);
        }
        retry++;
    }

    if( ufc23->State == UFC23_STATE_STANDBY )
    {
        result = RESULT_OK;
    }

    return result;
}

static inline Result Ufc23_EnableMeasureMode(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;
    Ufc23_GetMode(ufc23);

    switch(ufc23->State)
    {
        case UFC23_STATE_MEAS:
            result = RESULT_OK;
            break;
        case UFC23_STATE_STANDBY:
            Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MM_ENA_ENABLED);
            wait(UFC23_SWITCH_MEAS_MODE_MS);

            Ufc23_GetMode(ufc23);
            if( ufc23->State == UFC23_STATE_MEAS )
            {
                result = RESULT_OK;
            }
            break;
        default:
            result = RESULT_INVALID; 
    }
    
    return result;
}

static inline Result Ufc23_DisableMeasureMode(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;
    Ufc23_GetMode(ufc23);

    switch(ufc23->State)
    {
        case UFC23_STATE_STANDBY:
            result = RESULT_OK;
            break;
        case UFC23_STATE_MEAS:
            Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MM_ENA_DISABLED);
            wait(UFC23_SWITCH_MEAS_MODE_MS);

            Ufc23_GetMode(ufc23);
            if( ufc23->State == UFC23_STATE_STANDBY )
            {
                result = RESULT_OK;
            }
            break;
        default:
            result = RESULT_INVALID; 
    }
    
    return result;
}

static inline Result Ufc23_EnableMeasureModeTimerHalted(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;

    Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MCT_HALT_HALTED | UFC23_EXTENDED_COMMAND_EC_MM_ENA_ENABLED );
    wait(UFC23_SWITCH_MEAS_MODE_MS);
    uint8_t statusFlags = Ufc23_GetSystemStatusFlagRegister(ufc23);

    if( (statusFlags & UFC23_SSF_MCT_HALT_STATE) && (statusFlags & UFC23_SSF_MCYCLE_REQ) )
    {
        result = RESULT_OK;
    }

    return result;
}

static inline Result Ufc23_HaltMeasureTimer(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;

    uint8_t statusFlags = Ufc23_GetSystemStatusFlagRegister(ufc23);
    
    if( !(statusFlags & UFC23_SSF_MCT_HALT_STATE) )
    {
        Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MCT_HALT_HALTED);
        wait(UFC23_SWITCH_MEAS_MODE_MS);
        statusFlags = Ufc23_GetSystemStatusFlagRegister(ufc23);
    }

    if( statusFlags & UFC23_SSF_MCT_HALT_STATE )
    {
        result = RESULT_OK;
    }

    return result;
}

static inline Result Ufc23_ReleaseHaltMeasureTimer(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;

    uint8_t statusFlags = Ufc23_GetSystemStatusFlagRegister(ufc23);

    
    if( (statusFlags & UFC23_SSF_MCT_HALT_STATE) )
    {
        Ufc23_GetMode(ufc23);
        if( ufc23->State == UFC23_STATE_STANDBY )
        {
            Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MM_ENA_DISABLED);
        }
        if( ufc23->State == UFC23_STATE_MEAS )
        {
            Ufc23_WriteRemoteCommand(ufc23, UFC23_REMOTE_COMMAND_RC_MM_CTRL, UFC23_EXTENDED_COMMAND_EC_MM_ENA_ENABLED);
        }
        wait(UFC23_SWITCH_MEAS_MODE_MS);
        statusFlags = Ufc23_GetSystemStatusFlagRegister(ufc23);
    }

    if( !(statusFlags & UFC23_SSF_MCT_HALT_STATE) )
    {
        result = RESULT_OK;
    }

    return result;
}

static inline Result Ufc23_GetMode(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;
    Ufc23_StateTypeDef oldState = ufc23->State;

    uint8_t statusFlags = Ufc23_GetSystemStatusFlagRegister(ufc23);
    
    uint8_t lsoSettling     = !(statusFlags & UFC23_SSF_LSO_SETTLED);
    uint8_t bootingUp       =  statusFlags & UFC23_SSF_BL_REQ;
    uint8_t standbyState    = (statusFlags & UFC23_SSF_MAIN_STATE) && !(statusFlags & UFC23_SSF_MCYCLE_REQ);
    uint8_t measState       = (statusFlags & UFC23_SSF_MAIN_STATE) && (statusFlags & UFC23_SSF_MCYCLE_REQ);
    
    uint8_t detectedStates = 0;

    if( lsoSettling & !(bootingUp || standbyState || measState))
    {
        ufc23->State = UFC23_STATE_RESET;
        detectedStates++;
    }

    if( bootingUp & !(lsoSettling || standbyState || measState))
    {
        ufc23->State = UFC23_STATE_BOOTLOAD;
        detectedStates++;
    }

    if( standbyState & !(lsoSettling || bootingUp || measState) )
    {
        ufc23->State = UFC23_STATE_STANDBY;
        detectedStates++;
    }

    if( measState & !(lsoSettling || bootingUp || standbyState) )
    {
        ufc23->State = UFC23_STATE_MEAS;
        detectedStates++;
    }

    if( detectedStates == 1 )
    {
        result = RESULT_OK;
    }
    else
    {
        ufc23->State = oldState;
        result = RESULT_INVALID;
    }

    return result;
}

static inline Result Ufc23_StartCyclingMeasurement(ScioSense_Ufc23* ufc23)
{
    Ufc23_ClearFlagRegisters(ufc23);
    Result result = Ufc23_EnableMeasureMode(ufc23);

    return result;
}

static inline Result Ufc23_WriteConfig(ScioSense_Ufc23* ufc23)
{
    Result result = Ufc23_SetStandbyState(ufc23);

    if( result == RESULT_OK )
    {
        if( ufc23->State == UFC23_STATE_STANDBY )
        {
            for ( uint8_t idx=0; idx<UFC23_AMOUNT_CONFIGURATION_REGISTERS; idx++ )
            {
                uint16_t registerAddress = Ufc23_GetConfigurationRegisterAddress(ufc23, idx);
                Ufc23_WriteDWordRAM(ufc23, registerAddress, Ufc23_GetConfigurationRegisterSetting(ufc23, idx));
            }
    
            uint8_t configurationCorrect = 1;
            for ( uint8_t idx=0; idx<UFC23_AMOUNT_CONFIGURATION_REGISTERS; idx++ )
            {
                uint16_t registerAddress = Ufc23_GetConfigurationRegisterAddress(ufc23, idx);
                uint8_t registerContent[4];
                Ufc23_ReadDWordRAM(ufc23, registerAddress, registerContent, 1);
                configurationCorrect &= ( Ufc23_GetConfigurationRegisterSetting(ufc23, idx) == Ufc23_ByteArrayToDWord(registerContent, 0) );
            }
    
            if( configurationCorrect )
            {
                Ufc23_SetMeasureCycleTimeUs(ufc23);
                result = RESULT_OK;
            }
            else
            {
                result = RESULT_CHECKSUM_ERROR;
            }
        }
        else
        {
            result = RESULT_NOT_ALLOWED;
        }
    }

    return result;
}

static inline Result Ufc23_GetUSMData(ScioSense_Ufc23* ufc23)
{
    return Ufc23_ReadDWordRAM(ufc23, UFC23_RAM_USM_RESULTS_ADDRESS, ufc23->DataBuffer, UFC23_AMOUNT_USM_BATCH_REGISTERS);
}

static inline Result Ufc23_Update(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;
    
    uint8_t communicationFlags = Ufc23_GetCommunicationFlagRegister(ufc23);
    if( communicationFlags == UFC23_CMF_NO_ERROR )
    {
        ufc23->frontendStatusFlags = Ufc23_GetFrontendStatusFlagRegister(ufc23);
        UFC23_FR_SIZE interruptFlagRegister = Ufc23_GetInterruptFlagRegister(ufc23);
        uint8_t measurementCompleted = interruptFlagRegister & (UFC23_IF_MEASURE_CYCLE_SEQUENCE_DONE | UFC23_IF_MEASURE_CYCLE_BATCH_DONE);
        uint8_t errorsMeasurement = interruptFlagRegister & (UFC23_IF_USM_PAUSE_ERR | UFC23_IF_TASK_TIMEOUT | UFC23_IF_ERROR_DETECTED);
        
        if( measurementCompleted && !errorsMeasurement )
        {
            result = Ufc23_GetUSMData(ufc23);
            communicationFlags = Ufc23_GetCommunicationFlagRegister(ufc23);
        }
    }

    if( communicationFlags & UFC23_CMF_SYSTEM_BUS_MASTER_STATE)
    {
        result = RESULT_NOT_ALLOWED;
    }

    if( communicationFlags & (UFC23_CMF_SYSTEM_BUS_COLLISION | UFC23_CMF_CR_UPDATE_ERROR))
    {
        result = RESULT_IO_ERROR;
    }
    else
    {
        if( communicationFlags & UFC23_CMF_ERROR_FLAG)
        {
            uint16_t frontendErrorFlags = Ufc23_GetFrontendErrorFlagRegister(ufc23);
            if( frontendErrorFlags & (UFC23_FES_TDC_TO_HCC | UFC23_FES_TDC_TO_TM | UFC23_FES_TDC_TO_PW_UP | UFC23_FES_TDC_TO_PW_DN | UFC23_FES_TDC_TO_TOF_UP | UFC23_FES_TDC_TO_TOF_DN) )
            {
                result = RESULT_TIMEOUT;
            }
            else
            {
                result = RESULT_IO_ERROR;
            }
        }
    }

    Ufc23_ClearFlagRegisters(ufc23);
    return result;
}

static inline void Ufc23_UpdateAmountBundlesInBatch(ScioSense_Ufc23* ufc23)
{
    switch( ufc23->Param.CR_AA.C_USM_MHIT_BATCH )
    {
        case UFC23_C_USM_MHIT_BATCH_1_USM_BUNDLE:
            ufc23->cyclesInBatch = 1;
            break;
        case UFC23_C_USM_MHIT_BATCH_2_USM_BUNDLES:
            ufc23->cyclesInBatch = 2;
            break;
        case UFC23_C_USM_MHIT_BATCH_4_USM_BUNDLES:
            ufc23->cyclesInBatch = 4;
            break;
        case UFC23_C_USM_MHIT_BATCH_6_USM_BUNDLES:
            ufc23->cyclesInBatch = 6;
            break;
        case UFC23_C_USM_MHIT_BATCH_8_USM_BUNDLES:
            ufc23->cyclesInBatch = 8;
            break;
        case UFC23_C_USM_MHIT_BATCH_10_USM_BUNDLES:
            ufc23->cyclesInBatch = 10;
            break;
        case UFC23_C_USM_MHIT_BATCH_12_USM_BUNDLES:
            ufc23->cyclesInBatch = 12;
            break;
        default:
          ufc23->cyclesInBatch = 12;
    }
}

static inline UFC23_FR_FE_SIZE Ufc23_ErrorsPresentInLastUpdate(ScioSense_Ufc23* ufc23)
{
    return ufc23->frontendErrorFlags;
}

static inline uint32_t Ufc23_ParseErrorFlags(ScioSense_Ufc23* ufc23, uint8_t batchIndex)
{
    return Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_STATUS_ADDRESS);
}

static inline uint8_t Ufc23_ParseAmplitudeRaw(ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_AMP_Raw_TypeDef* amplitudesRawUp, UFC23_AMP_Raw_TypeDef* amplitudesRawDn)
{
    uint8_t newValues = 0;

    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);

    uint32_t errorFlags = UFC23_FES_USM_HW_ERR_UP | UFC23_FES_USM_HW_ERR_DN;

    if( !(batchStatus & errorFlags) )
    {
        if( batchStatus & UFC23_FES_USM_AM_UPDATED )
        {
            uint32_t amplitudeValuesUp = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_AMPL_UPX3_ADDRESS);
            uint32_t amplitudeValuesDn = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_AMPL_DNX3_ADDRESS);

            amplitudesRawUp->AMPL1 = (amplitudeValuesUp & UFC23_USM_BUNDLE_AMPL_1_Msk) >> UFC23_USM_BUNDLE_AMPL_1_Pos;
            amplitudesRawUp->AMPL2 = (amplitudeValuesUp & UFC23_USM_BUNDLE_AMPL_2_Msk) >> UFC23_USM_BUNDLE_AMPL_2_Pos;
            amplitudesRawUp->AMPL3 = (amplitudeValuesUp & UFC23_USM_BUNDLE_AMPL_3_Msk) >> UFC23_USM_BUNDLE_AMPL_3_Pos;

            amplitudesRawDn->AMPL1 = (amplitudeValuesDn & UFC23_USM_BUNDLE_AMPL_1_Msk) >> UFC23_USM_BUNDLE_AMPL_1_Pos;
            amplitudesRawDn->AMPL2 = (amplitudeValuesDn & UFC23_USM_BUNDLE_AMPL_2_Msk) >> UFC23_USM_BUNDLE_AMPL_2_Pos;
            amplitudesRawDn->AMPL3 = (amplitudeValuesDn & UFC23_USM_BUNDLE_AMPL_3_Msk) >> UFC23_USM_BUNDLE_AMPL_3_Pos;
            newValues++;
        }
    }

    return newValues;
}

static inline uint8_t Ufc23_ParseAmplitudeV(ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_AMP_V_TypeDef* amplitudesVUp, UFC23_AMP_V_TypeDef* amplitudesVDn)
{
    // Update zero cross calibration in case it wasn't done before
    float zcc[UFC23_AMOUNT_BUNDLES_MAX];
    Ufc23_ParseBatchZcLvlV(ufc23, zcc);

    UFC23_AMP_Raw_TypeDef amplitudesRawUp[UFC23_AMOUNT_AMP_MEAS];
    UFC23_AMP_Raw_TypeDef amplitudesRawDn[UFC23_AMOUNT_AMP_MEAS];
    uint8_t readValues = Ufc23_ParseAmplitudeRaw(ufc23, batchIndex, amplitudesRawUp, amplitudesRawDn);
    
    if( readValues )
    {
        amplitudesVUp->AMPL1 = (float)(amplitudesRawUp->AMPL1) * UFC23_SAR_LSB_V - ufc23->zeroCrossCalibration;
        amplitudesVUp->AMPL2 = (float)(amplitudesRawUp->AMPL2) * UFC23_SAR_LSB_V - ufc23->zeroCrossCalibration;
        amplitudesVUp->AMPL3 = (float)(amplitudesRawUp->AMPL3) * UFC23_SAR_LSB_V - ufc23->zeroCrossCalibration;
        
        amplitudesVDn->AMPL1 = (float)(amplitudesRawDn->AMPL1) * UFC23_SAR_LSB_V - ufc23->zeroCrossCalibration;
        amplitudesVDn->AMPL2 = (float)(amplitudesRawDn->AMPL2) * UFC23_SAR_LSB_V - ufc23->zeroCrossCalibration;
        amplitudesVDn->AMPL3 = (float)(amplitudesRawDn->AMPL3) * UFC23_SAR_LSB_V - ufc23->zeroCrossCalibration;
    }

    return readValues;
}

static inline uint8_t Ufc23_ParseBatchAmplitudeRaw(ScioSense_Ufc23* ufc23, UFC23_AMP_Raw_TypeDef* amplitudesRawUp, UFC23_AMP_Raw_TypeDef* amplitudesRawDn)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t ampMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        ampMeasurements += Ufc23_ParseAmplitudeRaw(ufc23, batchIndex, &amplitudesRawUp[ampMeasurements], &amplitudesRawDn[ampMeasurements]);
    }
    return ampMeasurements;
}

static inline uint8_t Ufc23_ParseBatchAmplitudeV(ScioSense_Ufc23* ufc23, UFC23_AMP_V_TypeDef* amplitudesVUp, UFC23_AMP_V_TypeDef* amplitudesVDn)
{
    uint8_t ampMeasurements = 0;
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        ampMeasurements += Ufc23_ParseAmplitudeV(ufc23, batchIndex, &amplitudesVUp[ampMeasurements], &amplitudesVDn[ampMeasurements]);
    }
    return ampMeasurements;
}

static inline uint8_t Ufc23_ParsePulseWidthRaw(ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_PW_Raw_TypeDef* pulseWidthsRawUp, UFC23_PW_Raw_TypeDef* pulseWidthsRawDn)
{
    uint8_t newValues = 0;

    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);

    uint32_t errorFlags = 
        UFC23_FES_USM_HW_ERR_UP | 
        UFC23_FES_USM_HW_ERR_DN | 
        UFC23_FES_TDC_TO_PW_UP  |
        UFC23_FES_TDC_TO_PW_DN;

    if( !(batchStatus & errorFlags) )
    {
        if( batchStatus & UFC23_FES_USM_PWD_UPDATED )
        {
            uint32_t pulseWidthValuesFhlUp = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_UP_FHL_ADDRESS);
            uint32_t pulseWidthValuesFhlDn = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_DN_FHL_ADDRESS);

            pulseWidthsRawUp->PW1_FHL = (pulseWidthValuesFhlUp & UFC23_USM_BUNDLE_PW1_FHL_Msk) >> UFC23_USM_BUNDLE_PW1_FHL_Pos;
            pulseWidthsRawUp->PW2_FHL = (pulseWidthValuesFhlUp & UFC23_USM_BUNDLE_PW2_FHL_Msk) >> UFC23_USM_BUNDLE_PW2_FHL_Pos;
            
            pulseWidthsRawDn->PW1_FHL = (pulseWidthValuesFhlDn & UFC23_USM_BUNDLE_PW1_FHL_Msk) >> UFC23_USM_BUNDLE_PW1_FHL_Pos;
            pulseWidthsRawDn->PW2_FHL = (pulseWidthValuesFhlDn & UFC23_USM_BUNDLE_PW2_FHL_Msk) >> UFC23_USM_BUNDLE_PW2_FHL_Pos;

            uint32_t pulseWidthValuesZclUp = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_UP_ZCL_ADDRESS);
            uint32_t pulseWidthValuesZclDn = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_DN_ZCL_ADDRESS);
            
            pulseWidthsRawUp->PW_ZCL = (pulseWidthValuesZclUp & UFC23_USM_BUNDLE_PW_ZCL_Msk) >> UFC23_USM_BUNDLE_PW_ZCL_Pos;
            pulseWidthsRawDn->PW_ZCL = (pulseWidthValuesZclDn & UFC23_USM_BUNDLE_PW_ZCL_Msk) >> UFC23_USM_BUNDLE_PW_ZCL_Pos;
            newValues++;
        }
    }

    return newValues;
}

static inline uint8_t Ufc23_ParsePulseWidthRatio(ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_PW_Ps_TypeDef* pulseWidthsRatioUp, UFC23_PW_Ps_TypeDef* pulseWidthsRatioDn)
{
    UFC23_PW_Raw_TypeDef pulseWidthsRawUp[UFC23_AMOUNT_PW_MEAS];
    UFC23_PW_Raw_TypeDef pulseWidthsRawDn[UFC23_AMOUNT_PW_MEAS];
    uint8_t readValues = Ufc23_ParsePulseWidthRaw(ufc23, batchIndex, pulseWidthsRawUp, pulseWidthsRawDn);

    if( readValues )
    {
        pulseWidthsRatioUp->PW1_FHL = ((float)pulseWidthsRawUp->PW1_FHL) / ((float)pulseWidthsRawUp->PW_ZCL);
        pulseWidthsRatioUp->PW2_FHL = ((float)pulseWidthsRawUp->PW2_FHL) / ((float)pulseWidthsRawUp->PW_ZCL);
        
        pulseWidthsRatioDn->PW1_FHL = ((float)pulseWidthsRawDn->PW1_FHL) / ((float)pulseWidthsRawDn->PW_ZCL);
        pulseWidthsRatioDn->PW2_FHL = ((float)pulseWidthsRawDn->PW2_FHL) / ((float)pulseWidthsRawDn->PW_ZCL);
    }

    return readValues;
}

static inline uint8_t Ufc23_ParseBatchPulseWidthRaw(ScioSense_Ufc23* ufc23, UFC23_PW_Raw_TypeDef* pulseWidthsRawUp, UFC23_PW_Raw_TypeDef* pulseWidthsRawDn)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t pwMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        pwMeasurements += Ufc23_ParsePulseWidthRaw(ufc23, batchIndex, &pulseWidthsRawUp[pwMeasurements], &pulseWidthsRawDn[pwMeasurements]);
    }
    return pwMeasurements;
}

static inline uint8_t Ufc23_ParseBatchPulseWidthRatio(ScioSense_Ufc23* ufc23, UFC23_PW_Ps_TypeDef* pulseWidthsVUp, UFC23_PW_Ps_TypeDef* pulseWidthsVDn)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t pwMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        pwMeasurements += Ufc23_ParsePulseWidthRatio(ufc23, batchIndex, &pulseWidthsVUp[pwMeasurements], &pulseWidthsVDn[pwMeasurements]);
    }
    return pwMeasurements;
}

static inline uint8_t Ufc23_ParseTofMultiHitSumRaw(ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint64_t* tofMultiHitUp, uint64_t* tofMultiHitDn)
{
    uint8_t newValues = 0;

    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);

    uint32_t errorFlags = 
        UFC23_FES_USM_HW_ERR_UP | 
        UFC23_FES_USM_HW_ERR_DN |
        UFC23_FES_USM_TO_TOF_UP |
        UFC23_FES_USM_TO_TOF_DN;

    if( !(batchStatus & errorFlags) )
    {
        if( batchStatus & UFC23_FES_USM_TOF_MULTI_UPDATED )
        {
            uint32_t tofMultiHitValuesMsbUp = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_UP_ZCL_ADDRESS);
            uint64_t tofMultiHitMsbUp = (tofMultiHitValuesMsbUp & UFC23_USM_BUNDLE_TOF_MULTIHIT_Msk) >> UFC23_USM_BUNDLE_TOF_MULTIHIT_Pos;
            uint32_t tofMultiHitValuesLsbUp = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_TOF_MULTIHIT_UP_LSB_ADDRESS);
            uint64_t tofMultiHitLsbUp = (tofMultiHitValuesLsbUp & UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Msk) >> UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Pos;
            
            uint32_t tofMultiHitValuesMsbDn = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_DN_ZCL_ADDRESS);
            uint64_t tofMultiHitMsbDn = (tofMultiHitValuesMsbDn & UFC23_USM_BUNDLE_TOF_MULTIHIT_Msk) >> UFC23_USM_BUNDLE_TOF_MULTIHIT_Pos;
            uint32_t tofMultiHitValuesLsbDn = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_TOF_MULTIHIT_DN_LSB_ADDRESS);
            uint64_t tofMultiHitLsbDn = (tofMultiHitValuesLsbDn & UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Msk) >> UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Pos;
            
            *tofMultiHitUp =    ( (tofMultiHitMsbUp << UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Pos) &  UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Msk);
            *tofMultiHitUp |=   ( (tofMultiHitLsbUp << UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Pos) &  UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Msk);

            *tofMultiHitDn =    ( (tofMultiHitMsbDn << UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Pos) &  UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Msk);
            *tofMultiHitDn |=   ( (tofMultiHitLsbDn << UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Pos) &  UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Msk);

            newValues++;
        }
    }
    
    return newValues;
}

static inline uint8_t Ufc23_ParseTofMultiHitNs(ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* tofMultiHitUp, float* tofMultiHitDn)
{
    uint64_t tofMultiHitRawUp;
    uint64_t tofMultiHitRawDn;
    uint8_t readValues = Ufc23_ParseTofMultiHitSumRaw(ufc23, batchIndex, &tofMultiHitRawUp, &tofMultiHitRawDn);
    
    if( readValues)
    {
        *tofMultiHitUp = ((float)tofMultiHitRawUp) * ufc23->tofLsbNs;
        *tofMultiHitDn = ((float)tofMultiHitRawDn) * ufc23->tofLsbNs;
    }

    return readValues;
}

static inline uint8_t Ufc23_ParseBatchTofMultiHitSumRaw(ScioSense_Ufc23* ufc23, uint64_t* tofMultiHitUp_Raw, uint64_t* tofMultiHitDn_Raw)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t multiHitsMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        multiHitsMeasurements += Ufc23_ParseTofMultiHitSumRaw(ufc23, batchIndex, &tofMultiHitUp_Raw[multiHitsMeasurements], &tofMultiHitDn_Raw[multiHitsMeasurements]);
    }
    return multiHitsMeasurements;
}

static inline uint8_t Ufc23_ParseBatchTofMultiHitNs(ScioSense_Ufc23* ufc23, float* tofMultiHitPsUp, float* tofMultiHitPsDn)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t multiHitsMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        multiHitsMeasurements += Ufc23_ParseTofMultiHitNs(ufc23, batchIndex, &tofMultiHitPsUp[multiHitsMeasurements], &tofMultiHitPsDn[multiHitsMeasurements]);
    }
    return multiHitsMeasurements;
}

static inline uint8_t Ufc23_ParseTofMultiHitsCount(ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint8_t* multiHitCountUp, uint8_t* multiHitCountDn)
{
    uint8_t newValues = 0;

    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);

    uint32_t errorFlags = 
        UFC23_FES_USM_HW_ERR_UP | 
        UFC23_FES_USM_HW_ERR_DN;

    if( !(batchStatus & errorFlags) )
    {
        if( batchStatus & UFC23_FES_USM_TOF_MULTI_UPDATED )
        {
            uint32_t multiHitCountValuesUp = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_UP_ZCL_ADDRESS);
            uint8_t summedHitsUp = (multiHitCountValuesUp & UFC23_USM_BUNDLE_TOF_HIT_NUM_Msk) >> UFC23_USM_BUNDLE_TOF_HIT_NUM_Pos;
            
            uint32_t multiHitCountValuesDn = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_PW_DN_ZCL_ADDRESS);
            uint8_t summedHitsDn = (multiHitCountValuesDn & UFC23_USM_BUNDLE_TOF_HIT_NUM_Msk) >> UFC23_USM_BUNDLE_TOF_HIT_NUM_Pos;
            
            multiHitCountUp[newValues] = summedHitsUp;
            multiHitCountDn[newValues] = summedHitsDn;
            
            newValues++;
        }
    }

    return newValues;
}

static inline uint8_t Ufc23_ParseBatchTofMultiHitsCount(ScioSense_Ufc23* ufc23, uint8_t* multiHitCountUp, uint8_t* multiHitCountDn)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t multiHitsCountMeas = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        multiHitsCountMeas += Ufc23_ParseTofMultiHitsCount(ufc23, batchIndex, &multiHitCountUp[multiHitsCountMeas], &multiHitCountDn[multiHitsCountMeas]);
    }
    return multiHitsCountMeas;
}

static inline uint8_t Ufc23_ParseVddRaw(ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint16_t* vdd, uint16_t* vcc)
{
    uint8_t newValues = 0;

    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);

    if( batchStatus & UFC23_FES_USM_VCC_UPDATED )
    {
        uint32_t vddVccValues = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_VCC_ADDRESS);
        
        vdd[newValues] = (vddVccValues & UFC23_USM_BUNDLE_VDD_Msk) >> UFC23_USM_BUNDLE_VDD_Pos;
        vcc[newValues] = (vddVccValues & UFC23_USM_BUNDLE_VCC_Msk) >> UFC23_USM_BUNDLE_VCC_Pos;
        newValues++;
    }

    return newValues;
}

static inline uint8_t Ufc23_ParseVddV(ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* vdd, float* vcc)
{
    uint16_t vddRaw[1];
    uint16_t vccRaw[1];
    uint8_t readValues = Ufc23_ParseVddRaw(ufc23, batchIndex, vddRaw, vccRaw);

    for( uint8_t idx = 0; idx < readValues; idx++ )
    {
        vdd[idx] = ((float)vddRaw[idx]) * 2.0 * UFC23_SAR_LSB_V;
        vcc[idx] = ((float)vccRaw[idx]) * 3.0 * UFC23_SAR_LSB_V;
    }

    return readValues;
}

static inline uint8_t Ufc23_ParseBatchVddV(ScioSense_Ufc23* ufc23, float* vdd, float* vcc)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t vddMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        vddMeasurements += Ufc23_ParseVddV(ufc23, batchIndex, &vdd[vddMeasurements], &vcc[vddMeasurements]);
    }
    return vddMeasurements;
}

static inline uint8_t Ufc23_ParseHccCalibRaw(ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint32_t* hccCalibration)
{
    uint8_t newValues = 0;

    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);

    uint32_t errorFlags = UFC23_FES_TDC_TO_HCC;

    if( !(batchStatus & errorFlags) )
    {
        if( batchStatus & UFC23_FES_USM_HCC_UPDATED )
        {
            uint32_t hccCalibrationValues = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_HCC_CALIB_ADDRESS);
            *hccCalibration = (hccCalibrationValues & UFC23_USM_BUNDLE_HCC_CALIB_Msk) >> UFC23_USM_BUNDLE_HCC_CALIB_Pos;
            Ufc23_UpdateCorrectionFactorHso(ufc23, *hccCalibration, UFC23_LSO_NOMINAL_FREQUENCY_HZ);
            
            newValues++;
        }
    }

    return newValues;
}

static inline uint8_t Ufc23_ParseHsoMhz(ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* fHsoMhz)
{
    uint32_t hccCalibrationRaw;
    uint8_t readValues = Ufc23_ParseHccCalibRaw(ufc23, batchIndex, &hccCalibrationRaw);
    
    if( readValues )
    {
        *fHsoMhz = hccCalibrationRaw / UFC23_HCC_FHSO_RATIO_MHZ;
    }

    return readValues;
}

static inline uint8_t Ufc23_ParseBatchHsoMhz(ScioSense_Ufc23* ufc23, float* fHsoMhz)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t hsoMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        hsoMeasurements += Ufc23_ParseHsoMhz(ufc23, batchIndex, fHsoMhz + hsoMeasurements);
    }
    return hsoMeasurements;
}

static inline uint8_t Ufc23_ParseZcLvlRaw(ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint16_t* zcLvl)
{
    uint8_t newValues = 0;
    
    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, batchIndex);
    
    if( batchStatus & UFC23_FES_USM_ZCC_UPDATED )
    {
        uint32_t zcLvlValues = Ufc23_ParseUsmBatchDWordValue(ufc23, batchIndex, UFC23_USM_BUNDLE_ZC_LVL_ADDRESS);
        
        zcLvl[newValues] = (zcLvlValues & UFC23_USM_BUNDLE_ZC_LVL_Msk) >> UFC23_USM_BUNDLE_ZC_LVL_Pos;
        newValues++;
    }
    
    return newValues;
}

static inline uint8_t Ufc23_ParseZcLvlV(ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* zcLvl)
{
    uint16_t zcLvlRaw;
    uint8_t readValues = Ufc23_ParseZcLvlRaw(ufc23, batchIndex, &zcLvlRaw);
    
    if( readValues )
    {
        *zcLvl = ((float)zcLvlRaw) * UFC23_SAR_LSB_V;
        ufc23->zeroCrossCalibration = *zcLvl;
    }

    return readValues;
}

static inline uint8_t Ufc23_ParseBatchZcLvlV(ScioSense_Ufc23* ufc23, float* zcLvl)
{
    uint8_t amountMeasurements = Ufc23_GetAmountMeasurementsInBatch(ufc23);
    uint8_t zclMeasurements = 0;
    for (uint8_t batchIndex=0; batchIndex<amountMeasurements; batchIndex++)
    {
        zclMeasurements += Ufc23_ParseZcLvlV(ufc23, batchIndex, &zcLvl[zclMeasurements]);
    }
    return zclMeasurements;
}

static inline uint8_t Ufc23_ParseSingleCycleUsTofHitsRaw(ScioSense_Ufc23* ufc23, uint32_t* usTofHitUp, uint32_t* usTofHitDn, uint8_t* amountHitsUp, uint8_t* amountHitsDn)
{
    uint8_t newValues = 0;
    uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, 0);

    uint32_t errorFlags = 
        UFC23_FES_USM_HW_ERR_UP | 
        UFC23_FES_USM_HW_ERR_DN;

    if( !(batchStatus & errorFlags) )
    {
        if( batchStatus & UFC23_FES_USM_TOF_MULTI_UPDATED )
        {
            Ufc23_ParseTofMultiHitsCount(ufc23, 0, amountHitsUp, amountHitsDn);

            for( uint8_t newValuesUp = 0; newValuesUp < *amountHitsUp; newValuesUp++)
            {
                usTofHitUp[newValuesUp] = Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_SINGLE_CYCLE_US_TOF_HIT_UP_1 + newValuesUp);
            }

            for( uint8_t newValuesDn = 0; newValuesDn < *amountHitsDn; newValuesDn++)
            {
                usTofHitDn[newValuesDn] = Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_SINGLE_CYCLE_US_TOF_HIT_DN_1 + newValuesDn);
            }

            if( *amountHitsUp >= *amountHitsDn )
            {
                newValues = *amountHitsDn;
            }
            else
            {
                newValues = *amountHitsUp;
            }
        }
    }
    return newValues;
}

static inline uint8_t Ufc23_ParseSingleCycleUsTofHitsNs(ScioSense_Ufc23* ufc23, float* usTofHitUp, float* usTofHitDn, uint8_t* amountHitsUp, uint8_t* amountHitsDn)
{
    uint32_t usTofHitRawUp[UFC23_AMOUNT_TOF_HITS_MEAS];
    uint32_t usTofHitRawDn[UFC23_AMOUNT_TOF_HITS_MEAS];
    uint8_t readValues = Ufc23_ParseSingleCycleUsTofHitsRaw(ufc23,usTofHitRawUp, usTofHitRawDn, amountHitsUp, amountHitsDn);
    
    for( uint8_t idx = 0; idx < *amountHitsUp; idx++ )
    {
        usTofHitUp[idx] = (float)usTofHitRawUp[idx] * ufc23->tofLsbNs;
    }

    for( uint8_t idx = 0; idx < *amountHitsDn; idx++ )
    {
        usTofHitDn[idx] = (float)usTofHitRawDn[idx] * ufc23->tofLsbNs;
    }
    
    return readValues;
}

static inline uint8_t Ufc23_ParseTemperatureSeq1degC(ScioSense_Ufc23* ufc23, float* temperature1DegC, float* temperature2DegC)
{
    uint8_t newValues = 0;

    if( ufc23->frontendStatusFlags & UFC23_EF_TEMPERATURE_RESULTS_UPDATED)
    {
        uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, 0);
    
        uint32_t errorFlags = 
            UFC23_FES_TDC_TO_TM | 
            UFC23_FES_TM_OPEN   |
            UFC23_FES_TM_SHORT;
    
        if( !(batchStatus & errorFlags) )
        {
            float tG_s1     = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_GAIN_COMPENSATION_SEQ1);
            float tRdson_s1 = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_RDSON_COMPENSATION_SEQ1);
            float tRef_s1   = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_REFERENCE_PORT_SEQ1);
            float tPT1_s1   = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_MEASURE_PORT_1_SEQ1);
            float tPT2_s1   = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_MEASURE_PORT_2_SEQ1);
            
            float tRo       = tRdson_s1 - tRef_s1;
            float deltaT    = 2 * ( tG_s1 - (tPT1_s1 * tRef_s1) / (tPT1_s1 + tRef_s1) );
            
            float tR        = tRef_s1 - tRo - deltaT;
            float t1        = tPT1_s1 - tRo - deltaT;
            float t2        = tPT2_s1 - tRo - deltaT;

            float tRatioC   = t1 / tR;
            float tRatioH   = t2 / tR;

            *temperature1DegC    = UFC23_PT_POLY_SQUARE_TERM * tRatioC * tRatioC + UFC23_PT_POLY_LINEAR_TERM * tRatioC - UFC23_PT_POLY_CONST_TERM;
            *temperature2DegC     = UFC23_PT_POLY_SQUARE_TERM * tRatioH * tRatioH + UFC23_PT_POLY_LINEAR_TERM * tRatioH - UFC23_PT_POLY_CONST_TERM;

            newValues++;
        }
    }
    return newValues;
}

static inline uint8_t Ufc23_ParseTemperatureSeq2degC(ScioSense_Ufc23* ufc23, float* temperature1DegC, float* temperature2DegC)
{
    uint8_t newValues = 0;

    if( ufc23->frontendStatusFlags & UFC23_EF_TEMPERATURE_RESULTS_UPDATED)
    {
        uint32_t batchStatus = Ufc23_ParseErrorFlags(ufc23, 0);
    
        uint32_t errorFlags = 
            UFC23_FES_TDC_TO_TM | 
            UFC23_FES_TM_OPEN   |
            UFC23_FES_TM_SHORT;
    
        if( !(batchStatus & errorFlags) )
        {
            float tG_s1     = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_GAIN_COMPENSATION_SEQ2);
            float tRdson_s1 = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_RDSON_COMPENSATION_SEQ2);
            float tRef_s1   = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_REFERENCE_PORT_SEQ2);
            float tPT1_s1   = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_MEASURE_PORT_1_SEQ2);
            float tPT2_s1   = (float)Ufc23_ParseUsmDWordValue(ufc23, UFC23_USM_BUNDLE_MEASURE_PORT_2_SEQ2);
            
            float tRo       = tRdson_s1 - tRef_s1;
            float deltaT    = 2 * ( tG_s1 - (tPT1_s1 * tRef_s1) / (tPT1_s1 + tRef_s1) );
            
            float tR        = tRef_s1 - tRo - deltaT;
            float t1        = tPT1_s1 - tRo - deltaT;
            float t2        = tPT2_s1 - tRo - deltaT;

            float tRatioC   = t1 / tR;
            float tRatioH   = t2 / tR;

            *temperature1DegC    = UFC23_PT_POLY_SQUARE_TERM * tRatioC * tRatioC + UFC23_PT_POLY_LINEAR_TERM * tRatioC - UFC23_PT_POLY_CONST_TERM;
            *temperature2DegC     = UFC23_PT_POLY_SQUARE_TERM * tRatioH * tRatioH + UFC23_PT_POLY_LINEAR_TERM * tRatioH - UFC23_PT_POLY_CONST_TERM;

            newValues++;
        }
    }
    return newValues;
}

static inline void Ufc23_SetMeasureCycleTimeUs(ScioSense_Ufc23* ufc23)
{
    uint32_t measureCycleTimeUs = UFC23_C_MCYCLE_TIME_LSB_US * ufc23->Param.CR_A5.C_MCT_EN;
    if( measureCycleTimeUs < UFC23_MINIMUM_CYCLE_TIME_US )
    {
        measureCycleTimeUs = UFC23_MINIMUM_CYCLE_TIME_US;
    }
    ufc23->measureCycleTimeUs = measureCycleTimeUs;
}

static inline UFC23_CYCLE_TIME_SIZE Ufc23_GetMeasureCycleTimeUs(ScioSense_Ufc23* ufc23)
{
    return ufc23->measureCycleTimeUs;
}

static inline UFC23_BATCH_AMOUNT_SIZE Ufc23_GetAmountMeasurementsInBatch(ScioSense_Ufc23* ufc23)
{
    return ufc23->cyclesInBatch;
}

static inline void Ufc23_UpdateCorrectionFactorHso(ScioSense_Ufc23* ufc23, uint32_t rmHsoCalib, float lsoNominalFrequencyHz)
{
    float hsoNominalFrequencyHz = UFC23_HSO_FREQUENCY_CONVERSION_FACTOR_MHZ * ufc23->Param.CR_A7.C_FEP_4M_CLK_DIV;
    float correctionFactor = 4.0 * hsoNominalFrequencyHz * 65536.0 / ((float)rmHsoCalib) / lsoNominalFrequencyHz;
    ufc23->correctionFactorHso = correctionFactor;
    Ufc23_UpdatePulseWidthLsb(ufc23, correctionFactor, hsoNominalFrequencyHz);
    Ufc23_UpdateTimeOfFlightLsb(ufc23, correctionFactor, hsoNominalFrequencyHz);
}

static inline void Ufc23_UpdatePulseWidthLsb(ScioSense_Ufc23* ufc23, float correctionFactor, float nominalFrequencyHz)
{
    ufc23->pwLsbNs = UFC23_NANOSECONDS_IN_A_SECOND / ( correctionFactor * nominalFrequencyHz * UFC23_PW_LSB_PRESCALER );
}

static inline void Ufc23_UpdateTimeOfFlightLsb(ScioSense_Ufc23* ufc23, float correctionFactor, float nominalFrequencyHz)
{
    ufc23->tofLsbNs = UFC23_NANOSECONDS_IN_A_SECOND / ( correctionFactor * nominalFrequencyHz * UFC23_TOF_LSB_PRESCALER );
}

static inline void Ufc23_SetConfigurationRegisters(ScioSense_Ufc23* ufc23, uint32_t* registerConfiguration)
{
    for( uint8_t registerIdx=0; registerIdx<UFC23_AMOUNT_CONFIGURATION_REGISTERS; registerIdx++ )
    {
        ufc23->CR[registerIdx] = registerConfiguration[registerIdx];
    }
    Ufc23_UpdateParameters(ufc23);
}

static inline void Ufc23_InitializeConfiguration(ScioSense_Ufc23* ufc23)
{
    uint8_t registersToWrite[] = {
        0xA0,   0xA1,   0xA2,   0xA3,   0xA4,   0xA5,   0xA6,   0xA7,           0xA9,   0xAA,   0xAB,   0xAC,   0xAD,   0xAE,   0xAF,
        0xB0,   0xB1,   0xB2
    };

    for( uint8_t idx=0; idx<UFC23_AMOUNT_CONFIGURATION_REGISTERS; idx++ )
    {
        ufc23->Addresses[idx] = registersToWrite[idx];
    }

    ufc23->State = UFC23_STATE_NOT_CONNECTED;

    uint32_t ufc23DefaultConfiguration[] = {
        0x0000007F,     // 0xA0
        0x00000000,     // 0xA1
        0x000006DB,     // 0xA2
        0x00000010,     // 0xA3
        0x000015AF,     // 0xA4
        0x00003080,     // 0xA5
        0x00000E79,     // 0xA6
        0x00008385,     // 0xA7

        0x04900000,     // 0xA9
        0xC0090002,     // 0xAA
        0x00000502,     // 0xAB
        0x00000000,     // 0xAC
        0x0800000F,     // 0xAD
        0x00000000,     // 0xAE
        0x03000000,     // 0xAF
        0x00002201,     // 0xB0
        0x20410000,     // 0xB1
        0x00000000      // 0xB2
    };

    Ufc23_SetConfigurationRegisters(ufc23, ufc23DefaultConfiguration);
}

static inline uint8_t Ufc23_IsPartIdValid(ScioSense_Ufc23* ufc23)
{
    return ( ( ufc23->partId == UFC18_SENSOR ) || ( ufc23->partId == UFC23_SENSOR ) );
}

static inline Result Ufc23_GetPartId(ScioSense_Ufc23* ufc23)
{
    ufc23->partId = UNKNOWN;
    
    UFC23_COM_SIZE deviceIdBytes[4];
    Result result = Ufc23_ReadDWordRAM(ufc23, UFC23_CR_SR_DEVICE_ID_ADDRESS, deviceIdBytes, 1);
    if( result == RESULT_OK )
    {
        uint32_t deviceIdReg = Ufc23_ByteArrayToDWord(deviceIdBytes, 0);
        uint32_t deviceId = UFC23_SR_DEVICE_ID_GET( deviceIdReg );
        if( deviceId == UFC23_C_SR_DEVICE_ID_UFC18 )
        {
            ufc23->partId = UFC18_SENSOR;
        }
        else if ( deviceId == UFC23_C_SR_DEVICE_ID_UFC23 )
        {
            ufc23->partId = UFC23_SENSOR;
        }
        else
        {
            result = RESULT_INVALID;
        }
    }
    
    return result;
}

static inline Result Ufc23_ReadConfig(ScioSense_Ufc23* ufc23)
{
    Result result = RESULT_IO_ERROR;
    uint32_t currentConfig[UFC23_AMOUNT_CONFIGURATION_REGISTERS];

    for( uint8_t idx=0; idx<UFC23_AMOUNT_CONFIGURATION_REGISTERS; idx++ )
    {
        uint8_t registerContent[4];
        result |= Ufc23_ReadDWordRAM(ufc23, ufc23->Addresses[idx], registerContent, 1);
        currentConfig[idx] = Ufc23_ByteArrayToDWord(registerContent, 0);
    }

    if( result == RESULT_OK )
    {
        Ufc23_SetConfigurationRegisters(ufc23, currentConfig);
    }

    return result;
}

static inline void Ufc23_UpdateParameters(ScioSense_Ufc23* ufc23)
{
    // Parameter of CR[0]
    ufc23->Param.CR_A0.C_IRQ_EN_BL_DONE         = UFC23_C_IRQ_EN_BL_DONE_GET(       ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    ufc23->Param.CR_A0.C_IRQ_EN_MIS_DONE        = UFC23_C_IRQ_EN_MIS_DONE_GET(      ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    ufc23->Param.CR_A0.C_IRQ_EN_MCS_DONE        = UFC23_C_IRQ_EN_MCS_DONE_GET(      ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    ufc23->Param.CR_A0.C_IRQ_EN_MC_BATCH_DONE   = UFC23_C_IRQ_EN_MC_BATCH_DONE_GET( ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    ufc23->Param.CR_A0.C_IRQ_EN_STASK_DONE      = UFC23_C_IRQ_EN_STASK_DONE_GET(    ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    ufc23->Param.CR_A0.C_IRQ_EN_USM_PAUSE_ERR   = UFC23_C_IRQ_EN_USM_PAUSE_ERR_GET( ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    ufc23->Param.CR_A0.C_IRQ_EN_TSC_TMO         = UFC23_C_IRQ_EN_TSC_TMO_GET(       ufc23->CR[UFC23_CR_FRU_IFH_INDEX] );
    // Parameter of CR[UFC23_CR_FRU_EFH_INDEX]
    ufc23->Param.CR_A1.C_EF_EN_HCC_TDC_TMO      = UFC23_C_EF_EN_HCC_TDC_TMO_GET(    ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_TM_TDC_TMO       = UFC23_C_EF_EN_TM_TDC_TMO_GET(     ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_TM_OC_ERR        = UFC23_C_EF_EN_TM_OC_ERR_GET(      ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_TM_SC_ERR        = UFC23_C_EF_EN_TM_SC_ERR_GET(      ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_USM_HW_UP_ERR    = UFC23_C_EF_EN_USM_HW_UP_ERR_GET(  ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_USM_HW_DN_ERR    = UFC23_C_EF_EN_USM_HW_DN_ERR_GET(  ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_PW_UP_TDC_TMO    = UFC23_C_EF_EN_PW_UP_TDC_TMO_GET(  ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_PW_DN_TDC_TMO    = UFC23_C_EF_EN_PW_DN_TDC_TMO_GET(  ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_TOF_UP_TDC_TMO   = UFC23_C_EF_EN_TOF_UP_TDC_TMO_GET( ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_TOF_DN_TDC_TMO   = UFC23_C_EF_EN_TOF_DN_TDC_TMO_GET( ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_USM_UP_TMO       = UFC23_C_EF_EN_USM_UP_TMO_GET(     ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    ufc23->Param.CR_A1.C_EF_EN_USM_DN_TMO       = UFC23_C_EF_EN_USM_DN_TMO_GET(     ufc23->CR[UFC23_CR_FRU_EFH_INDEX] );
    // Parameter of CR[UFC23_CR_GP_CTRL_INDEX]
    ufc23->Param.CR_A2.C_GPIO0_MODE             = UFC23_C_GPIO0_MODE_GET(           ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    ufc23->Param.CR_A2.C_GPIO1_MODE             = UFC23_C_GPIO1_MODE_GET(           ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    ufc23->Param.CR_A2.C_GPIO2_MODE             = UFC23_C_GPIO2_MODE_GET(           ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    ufc23->Param.CR_A2.C_GPIO3_MODE             = UFC23_C_GPIO3_MODE_GET(           ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    ufc23->Param.CR_A2.C_PRB_SEL                = UFC23_C_PRB_SEL_GET(              ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    ufc23->Param.CR_A2.C_TST_STM                = UFC23_C_TST_STM_GET(              ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    ufc23->Param.CR_A2.C_MISO_HZ_DIS            = UFC23_C_MISO_HZ_DIS_GET(          ufc23->CR[UFC23_CR_GP_CTRL_INDEX] );
    // Parameter of CR[UFC23_CR_PM_INDEX]
    ufc23->Param.CR_A3.C_LDO_RF_RATE            = UFC23_C_LDO_RF_RATE_GET(          ufc23->CR[UFC23_CR_PM_INDEX] );
    ufc23->Param.CR_A3.C_VDD18_SW_MODE          = UFC23_C_VDD18_SW_MODE_GET(        ufc23->CR[UFC23_CR_PM_INDEX] );
    // Parameter of CR[UFC23_CR_TSC_INDEX]
    ufc23->Param.CR_A4.C_USM_PAUSE_TSEL         = UFC23_C_USM_PAUSE_TSEL_GET(       ufc23->CR[UFC23_CR_TSC_INDEX] );
    ufc23->Param.CR_A4.C_USM_REPEAT             = UFC23_C_USM_REPEAT_GET(           ufc23->CR[UFC23_CR_TSC_INDEX] );
    ufc23->Param.CR_A4.C_USM_DIR_MODE           = UFC23_C_USM_DIR_MODE_GET(         ufc23->CR[UFC23_CR_TSC_INDEX] );
    ufc23->Param.CR_A4.C_USM_EDGE_MODE          = UFC23_C_USM_EDGE_MODE_GET(        ufc23->CR[UFC23_CR_TSC_INDEX] );
    ufc23->Param.CR_A4.C_LDO_STUP_TSEL          = UFC23_C_LDO_STUP_TSEL_GET(        ufc23->CR[UFC23_CR_TSC_INDEX] );
    ufc23->Param.CR_A4.C_LDO_FEP_MODE           = UFC23_C_LDO_FEP_MODE_GET(         ufc23->CR[UFC23_CR_TSC_INDEX] );
    ufc23->Param.CR_A4.C_TM_SQC_MODE            = UFC23_C_TM_SQC_MODE_GET(          ufc23->CR[UFC23_CR_TSC_INDEX] );
    // Parameter of CR[UFC23_CR_MCT_INDEX]
    ufc23->Param.CR_A5.C_MCYCLE_TIME            = UFC23_C_MCYCLE_TIME_GET(          ufc23->CR[UFC23_CR_MCT_INDEX] );
    ufc23->Param.CR_A5.C_MCYCLE_TAIL_SEL        = UFC23_C_MCYCLE_TAIL_SEL_GET(      ufc23->CR[UFC23_CR_MCT_INDEX] );
    ufc23->Param.CR_A5.C_MCT_EN                 = UFC23_C_MCT_EN_GET(               ufc23->CR[UFC23_CR_MCT_INDEX] );
    // Parameter of CR[UFC23_CR_MRG_INDEX]
    ufc23->Param.CR_A6.C_USM_RATE               = UFC23_C_USM_RATE_GET(             ufc23->CR[UFC23_CR_MRG_INDEX] );
    ufc23->Param.CR_A6.C_ZCC_RATE               = UFC23_C_ZCC_RATE_GET(             ufc23->CR[UFC23_CR_MRG_INDEX] );
    ufc23->Param.CR_A6.C_VCCM_RATE              = UFC23_C_VCCM_RATE_GET(            ufc23->CR[UFC23_CR_MRG_INDEX] );
    ufc23->Param.CR_A6.C_HCC_RATE               = UFC23_C_HCC_RATE_GET(             ufc23->CR[UFC23_CR_MRG_INDEX] );
    ufc23->Param.CR_A6.C_FBC_RATE               = UFC23_C_FBC_RATE_GET(             ufc23->CR[UFC23_CR_MRG_INDEX] );
    ufc23->Param.CR_A6.C_TM_RATE                = UFC23_C_TM_RATE_GET(              ufc23->CR[UFC23_CR_MRG_INDEX] );
    // Parameter of CR[UFC23_CR_FEP_MCTRL_INDEX]
    ufc23->Param.CR_A7.C_FEP_4M_CLK_DIV         = UFC23_C_FEP_4M_CLK_DIV_GET(       ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_ADC_ST                 = UFC23_C_ADC_ST_GET(               ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_FEP_STUP_TSEL          = UFC23_C_FEP_STUP_TSEL_GET(        ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_USM_TMO_SEL            = UFC23_C_USM_TMO_SEL_GET(          ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_TM_PORT_NO             = UFC23_C_TM_PORT_NO_GET(           ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_TM_PORT_MODE           = UFC23_C_TM_PORT_MODE_GET(         ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_TM_CYCLE_SEL           = UFC23_C_TM_CYCLE_SEL_GET(         ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_TM_SQC_NO              = UFC23_C_TM_SQC_NO_GET(            ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_HF_CLB_SEL             = UFC23_C_HF_CLB_SEL_GET(           ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_HF_SERIAL              = UFC23_C_HF_SERIAL_GET(            ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_HF_TRIM                = UFC23_C_HF_TRIM_GET(              ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_HF_CALIB_MODE          = UFC23_C_HF_CALIB_MODE_GET(        ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    ufc23->Param.CR_A7.C_HF_DCO_ENA             = UFC23_C_HF_DCO_ENA_GET(           ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX] );
    // Parameter of CR[UFC23_CR_FEP_TDC_TRIM_INDEX]
    ufc23->Param.CR_A9.C_TDC_SEL_QHA1           = UFC23_C_TDC_SEL_QHA1_GET(         ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX] );
    ufc23->Param.CR_A9.C_TDC_SEL_QHA2           = UFC23_C_TDC_SEL_QHA2_GET(         ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX] );
    ufc23->Param.CR_A9.C_TDC_PHS_MODE           = UFC23_C_TDC_PHS_MODE_GET(         ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX] );
    ufc23->Param.CR_A9.C_PHS_CELLS              = UFC23_C_PHS_CELLS_GET(            ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX] );
    ufc23->Param.CR_A9.C_PHS_INCR               = UFC23_C_PHS_INCR_GET(             ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX] );
    ufc23->Param.CR_A9.C_TDC_HR_ADJUST          = UFC23_C_TDC_HR_ADJUST_GET(        ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX] );
    // Parameter of CR[UFC23_CR_USM_PROC_INDEX]
    ufc23->Param.CR_AA.C_USM_MASK_WIN           = UFC23_C_USM_MASK_WIN_GET(         ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_USM_MHIT_BATCH         = UFC23_C_USM_MHIT_BATCH_GET(       ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_USM_SENSOR_MODE        = UFC23_C_USM_SENSOR_MODE_GET(      ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_USM_AM_MODE            = UFC23_C_USM_AM_MODE_GET(          ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_USM_PWD_MODE           = UFC23_C_USM_PWD_MODE_GET(         ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_ZCD_LVL                = UFC23_C_ZCD_LVL_GET(              ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_ZCC_MODE               = UFC23_C_ZCC_MODE_GET(             ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    ufc23->Param.CR_AA.C_ZCC_INIT_EN            = UFC23_C_ZCC_INIT_EN_GET(          ufc23->CR[UFC23_CR_USM_PROC_INDEX] );
    // Parameter of CR[UFC23_CR_USM_FBG_MCTRL_INDEX]
    ufc23->Param.CR_AB.C_FBG_SEL                = UFC23_C_FBG_SEL_GET(              ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] );
    ufc23->Param.CR_AB.C_FBG_LR_CLK_DIV         = UFC23_C_FBG_LR_CLK_DIV_GET(       ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] );
    ufc23->Param.CR_AB.C_FBG_FBNUM              = UFC23_C_FBG_FBNUM_GET(            ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] );
    ufc23->Param.CR_AB.C_FBG_FBSP               = UFC23_C_FBG_FBSP_GET(             ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] );
    ufc23->Param.CR_AB.C_FSPLITWID              = UFC23_C_FSPLITWID_GET(            ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] );
    ufc23->Param.CR_AB.C_FBG_HR_CLK_DIV         = UFC23_C_FBG_HR_CLK_DIV_GET(       ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX] );
    // Parameter of CR[UFC23_CR_USM_FBG_HRC_INDEX]
    ufc23->Param.CR_AC.C_FBG_HR_CALIB           = UFC23_C_FBG_HR_CALIB_GET(         ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX] );
    ufc23->Param.CR_AC.C_FBG_HR_CLB_SEL         = UFC23_C_FBG_HR_CLB_SEL_GET(       ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX] );
    ufc23->Param.CR_AC.C_FBG_HR_TRIM            = UFC23_C_FBG_HR_TRIM_GET(          ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX] );
    ufc23->Param.CR_AC.C_FBG_CALIB_MODE         = UFC23_C_FBG_CALIB_MODE_GET(       ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX] );
    // Parameter of CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]
    ufc23->Param.CR_AD.C_HS_OSC_TRIM            = UFC23_C_HS_OSC_TRIM_GET(          ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_HS_OSC_CFG             = UFC23_C_HS_OSC_CFG_GET(           ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_LS_OSC_CFG             = UFC23_C_LS_OSC_CFG_GET(           ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_PMU_BG_TRIM            = UFC23_C_PMU_BG_TRIM_GET(          ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_PMU_BIAS_TRIM          = UFC23_C_PMU_BIAS_TRIM_GET(        ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_PMU_LDO_SEL            = UFC23_C_PMU_LDO_SEL_GET(          ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_ZCD_IBSEL              = UFC23_C_ZCD_IBSEL_GET(            ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_ZCD_DAC_VREFN_SEL      = UFC23_C_ZCD_DAC_VREFN_SEL_GET(    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_PGA_GPIO_SEL           = UFC23_C_PGA_GPIO_SEL_GET(         ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_TX_CAP_MODE            = UFC23_C_TX_CAP_MODE_GET(          ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    ufc23->Param.CR_AD.C_USVREF_CAP_EN          = UFC23_C_USVREF_CAP_EN_GET(        ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX] );
    // Parameter of CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]
    ufc23->Param.CR_AE.C_PGA_ST2_GAIN           = UFC23_C_PGA_ST2_GAIN_GET(         ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ST2_CBYP           = UFC23_C_PGA_ST2_CBYP_GET(         ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ST1_GAIN           = UFC23_C_PGA_ST1_GAIN_GET(         ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ST1_CBYP           = UFC23_C_PGA_ST1_CBYP_GET(         ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_G1_OPEN            = UFC23_C_PGA_G1_OPEN_GET(          ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ISEL               = UFC23_C_PGA_ISEL_GET(             ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ST1_OPAN_ENA       = UFC23_C_PGA_ST1_OPAN_ENA_GET(     ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ST1_OPAP_ENA       = UFC23_C_PGA_ST1_OPAP_ENA_GET(     ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_PGA_ST2_OPA_ENA        = UFC23_C_PGA_ST2_OPA_ENA_GET(      ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_COMPSEL_SEL            = UFC23_C_COMPSEL_SEL_GET(          ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_R_COMPSEL              = UFC23_C_R_COMPSEL_GET(            ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_C_COMPSEL              = UFC23_C_C_COMPSEL_GET(            ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_RMSET_RX               = UFC23_C_RMSET_RX_GET(             ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_RMSET_TX               = UFC23_C_RMSET_TX_GET(             ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    ufc23->Param.CR_AE.C_SE_ENABLE              = UFC23_C_SE_ENABLE_GET(            ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX] );
    // Parameter of CR[UFC23_CR_USM_RCV_INIT_INDEX]
    ufc23->Param.CR_AF.C_US_VR_INIT             = UFC23_C_US_VR_INIT_GET(           ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    ufc23->Param.CR_AF.C_PGA_INIT               = UFC23_C_PGA_INIT_GET(             ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    ufc23->Param.CR_AF.C_COMP_INIT              = UFC23_C_COMP_INIT_GET(            ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    ufc23->Param.CR_AF.C_ADC_VR_INIT            = UFC23_C_ADC_VR_INIT_GET(          ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    ufc23->Param.CR_AF.C_ADC_INIT               = UFC23_C_ADC_INIT_GET(             ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    ufc23->Param.CR_AF.C_USM_INIT_MODE          = UFC23_C_USM_INIT_MODE_GET(        ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    ufc23->Param.CR_AF.C_DCO_INIT               = UFC23_C_DCO_INIT_GET(             ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX] );
    // Parameter of CR[UFC23_CR_USM_HIT_CTRL_INDEX]
    ufc23->Param.CR_B0.C_TOF_HIT_NO             = UFC23_C_TOF_HIT_NO_GET(           ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX] );
    ufc23->Param.CR_B0.C_TOF_HIT_RLS_MODE       = UFC23_C_TOF_HIT_RLS_MODE_GET(     ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX] );
    ufc23->Param.CR_B0.C_TOF_HIT_IGN_MODE       = UFC23_C_TOF_HIT_IGN_MODE_GET(     ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX] );
    ufc23->Param.CR_B0.C_TOF_MULTIHIT_START     = UFC23_C_TOF_MULTIHIT_START_GET(   ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX] );
    ufc23->Param.CR_B0.C_TOF_MULTIHIT_NO        = UFC23_C_TOF_MULTIHIT_NO_GET(      ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX] );
    // Parameter of CR[UFC23_CR_USM_WVM_INDEX]
    ufc23->Param.CR_B1.C_USM_FHL_UP             = UFC23_C_USM_FHL_UP_GET(           ufc23->CR[UFC23_CR_USM_WVM_INDEX] );
    ufc23->Param.CR_B1.C_USM_FHL_DN             = UFC23_C_USM_FHL_DN_GET(           ufc23->CR[UFC23_CR_USM_WVM_INDEX] );
    ufc23->Param.CR_B1.C_USM_AM_PD_1            = UFC23_C_USM_AM_PD_1_GET(          ufc23->CR[UFC23_CR_USM_WVM_INDEX] );
    ufc23->Param.CR_B1.C_USM_AM_PD_2            = UFC23_C_USM_AM_PD_2_GET(          ufc23->CR[UFC23_CR_USM_WVM_INDEX] );
    ufc23->Param.CR_B1.C_USM_AM_PD_3            = UFC23_C_USM_AM_PD_3_GET(          ufc23->CR[UFC23_CR_USM_WVM_INDEX] );
    ufc23->Param.CR_B1.C_AM_VEXTSEL             = UFC23_C_AM_VEXTSEL_GET(           ufc23->CR[UFC23_CR_USM_WVM_INDEX] );
    // Parameter of CR[UFC23_CR_USM_MASK_HR_WIN_INDEX]
    ufc23->Param.CR_B2.C_USM_MASK_HR_WIN_UP     = UFC23_C_USM_MASK_HR_WIN_UP_GET(   ufc23->CR[UFC23_CR_USM_MASK_HR_WIN_INDEX] );
    ufc23->Param.CR_B2.C_USM_MASK_HR_WIN_DN     = UFC23_C_USM_MASK_HR_WIN_DN_GET(   ufc23->CR[UFC23_CR_USM_MASK_HR_WIN_INDEX] );

    Ufc23_UpdateAmountBundlesInBatch(ufc23);
}

static inline void Ufc23_UpdateConfiguration(ScioSense_Ufc23* ufc23)
{
    // Parameter for CR[0]
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           = 0;
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_BL_DONE_SET       (ufc23->Param.CR_A0.C_IRQ_EN_BL_DONE      );
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_MIS_DONE_SET      (ufc23->Param.CR_A0.C_IRQ_EN_MIS_DONE     );
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_MCS_DONE_SET      (ufc23->Param.CR_A0.C_IRQ_EN_MCS_DONE     );
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_MC_BATCH_DONE_SET (ufc23->Param.CR_A0.C_IRQ_EN_MC_BATCH_DONE);
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_STASK_DONE_SET    (ufc23->Param.CR_A0.C_IRQ_EN_STASK_DONE   );
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_USM_PAUSE_ERR_SET (ufc23->Param.CR_A0.C_IRQ_EN_USM_PAUSE_ERR);
    ufc23->CR[UFC23_CR_FRU_IFH_INDEX]           |= UFC23_C_IRQ_EN_TSC_TMO_SET       (ufc23->Param.CR_A0.C_IRQ_EN_TSC_TMO      );
    // Parameter for CR[UFC23_CR_FRU_EFH_INDEX]
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           = 0;
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_HCC_TDC_TMO_SET    (ufc23->Param.CR_A1.C_EF_EN_HCC_TDC_TMO   );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_TM_TDC_TMO_SET     (ufc23->Param.CR_A1.C_EF_EN_TM_TDC_TMO    );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_TM_OC_ERR_SET      (ufc23->Param.CR_A1.C_EF_EN_TM_OC_ERR     );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_TM_SC_ERR_SET      (ufc23->Param.CR_A1.C_EF_EN_TM_SC_ERR     );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_USM_HW_UP_ERR_SET  (ufc23->Param.CR_A1.C_EF_EN_USM_HW_UP_ERR );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_USM_HW_DN_ERR_SET  (ufc23->Param.CR_A1.C_EF_EN_USM_HW_DN_ERR );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_PW_UP_TDC_TMO_SET  (ufc23->Param.CR_A1.C_EF_EN_PW_UP_TDC_TMO );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_PW_DN_TDC_TMO_SET  (ufc23->Param.CR_A1.C_EF_EN_PW_DN_TDC_TMO );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_TOF_UP_TDC_TMO_SET (ufc23->Param.CR_A1.C_EF_EN_TOF_UP_TDC_TMO);
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_TOF_DN_TDC_TMO_SET (ufc23->Param.CR_A1.C_EF_EN_TOF_DN_TDC_TMO);
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_USM_UP_TMO_SET     (ufc23->Param.CR_A1.C_EF_EN_USM_UP_TMO    );
    ufc23->CR[UFC23_CR_FRU_EFH_INDEX]           |= UFC23_C_EF_EN_USM_DN_TMO_SET     (ufc23->Param.CR_A1.C_EF_EN_USM_DN_TMO    );
    // Parameter for CR[UFC23_CR_GP_CTRL_INDEX]
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           = 0;
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_GPIO0_MODE_SET           (ufc23->Param.CR_A2.C_GPIO0_MODE          );
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_GPIO1_MODE_SET           (ufc23->Param.CR_A2.C_GPIO1_MODE          );
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_GPIO2_MODE_SET           (ufc23->Param.CR_A2.C_GPIO2_MODE          );
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_GPIO3_MODE_SET           (ufc23->Param.CR_A2.C_GPIO3_MODE          );
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_PRB_SEL_SET              (ufc23->Param.CR_A2.C_PRB_SEL             );
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_TST_STM_SET              (ufc23->Param.CR_A2.C_TST_STM             );
    ufc23->CR[UFC23_CR_GP_CTRL_INDEX]           |= UFC23_C_MISO_HZ_DIS_SET          (ufc23->Param.CR_A2.C_MISO_HZ_DIS         );
    // Parameter for CR[UFC23_CR_PM_INDEX]
    ufc23->CR[UFC23_CR_PM_INDEX]                = 0;
    ufc23->CR[UFC23_CR_PM_INDEX]                |= UFC23_C_LDO_RF_RATE_SET          (ufc23->Param.CR_A3.C_LDO_RF_RATE         );
    ufc23->CR[UFC23_CR_PM_INDEX]                |= UFC23_C_VDD18_SW_MODE_SET        (ufc23->Param.CR_A3.C_VDD18_SW_MODE       );
    // Parameter for CR[UFC23_CR_TSC_INDEX]
    ufc23->CR[UFC23_CR_TSC_INDEX]               = 0;
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_USM_PAUSE_TSEL_SET       (ufc23->Param.CR_A4.C_USM_PAUSE_TSEL      );
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_USM_REPEAT_SET           (ufc23->Param.CR_A4.C_USM_REPEAT          );
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_USM_DIR_MODE_SET         (ufc23->Param.CR_A4.C_USM_DIR_MODE        );
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_USM_EDGE_MODE_SET        (ufc23->Param.CR_A4.C_USM_EDGE_MODE       );
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_LDO_STUP_TSEL_SET        (ufc23->Param.CR_A4.C_LDO_STUP_TSEL       );
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_LDO_FEP_MODE_SET         (ufc23->Param.CR_A4.C_LDO_FEP_MODE        );
    ufc23->CR[UFC23_CR_TSC_INDEX]               |= UFC23_C_TM_SQC_MODE_SET          (ufc23->Param.CR_A4.C_TM_SQC_MODE         );
    // Parameter for CR[UFC23_CR_MCT_INDEX]
    ufc23->CR[UFC23_CR_MCT_INDEX]               = 0;
    ufc23->CR[UFC23_CR_MCT_INDEX]               |= UFC23_C_MCYCLE_TIME_SET          (ufc23->Param.CR_A5.C_MCYCLE_TIME         );
    ufc23->CR[UFC23_CR_MCT_INDEX]               |= UFC23_C_MCYCLE_TAIL_SEL_SET      (ufc23->Param.CR_A5.C_MCYCLE_TAIL_SEL     );
    ufc23->CR[UFC23_CR_MCT_INDEX]               |= UFC23_C_MCT_EN_SET               (ufc23->Param.CR_A5.C_MCT_EN              );
    // Parameter for CR[UFC23_CR_MRG_INDEX]
    ufc23->CR[UFC23_CR_MRG_INDEX]               = 0;
    ufc23->CR[UFC23_CR_MRG_INDEX]               |= UFC23_C_USM_RATE_SET             (ufc23->Param.CR_A6.C_USM_RATE            );
    ufc23->CR[UFC23_CR_MRG_INDEX]               |= UFC23_C_ZCC_RATE_SET             (ufc23->Param.CR_A6.C_ZCC_RATE            );
    ufc23->CR[UFC23_CR_MRG_INDEX]               |= UFC23_C_VCCM_RATE_SET            (ufc23->Param.CR_A6.C_VCCM_RATE           );
    ufc23->CR[UFC23_CR_MRG_INDEX]               |= UFC23_C_HCC_RATE_SET             (ufc23->Param.CR_A6.C_HCC_RATE            );
    ufc23->CR[UFC23_CR_MRG_INDEX]               |= UFC23_C_FBC_RATE_SET             (ufc23->Param.CR_A6.C_FBC_RATE            );
    ufc23->CR[UFC23_CR_MRG_INDEX]               |= UFC23_C_TM_RATE_SET              (ufc23->Param.CR_A6.C_TM_RATE             );
    // Parameter for CR[UFC23_CR_FEP_MCTRL_INDEX]
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         = 0;
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_FEP_4M_CLK_DIV_SET       (ufc23->Param.CR_A7.C_FEP_4M_CLK_DIV      );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_ADC_ST_SET               (ufc23->Param.CR_A7.C_ADC_ST              );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_FEP_STUP_TSEL_SET        (ufc23->Param.CR_A7.C_FEP_STUP_TSEL       );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_USM_TMO_SEL_SET          (ufc23->Param.CR_A7.C_USM_TMO_SEL         );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_TM_PORT_NO_SET           (ufc23->Param.CR_A7.C_TM_PORT_NO          );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_TM_PORT_MODE_SET         (ufc23->Param.CR_A7.C_TM_PORT_MODE        );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_TM_CYCLE_SEL_SET         (ufc23->Param.CR_A7.C_TM_CYCLE_SEL        );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_TM_SQC_NO_SET            (ufc23->Param.CR_A7.C_TM_SQC_NO           );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_HF_CLB_SEL_SET           (ufc23->Param.CR_A7.C_HF_CLB_SEL          );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_HF_SERIAL_SET            (ufc23->Param.CR_A7.C_HF_SERIAL           );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_HF_TRIM_SET              (ufc23->Param.CR_A7.C_HF_TRIM             );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_HF_CALIB_MODE_SET        (ufc23->Param.CR_A7.C_HF_CALIB_MODE       );
    ufc23->CR[UFC23_CR_FEP_MCTRL_INDEX]         |= UFC23_C_HF_DCO_ENA_SET           (ufc23->Param.CR_A7.C_HF_DCO_ENA          );
    // Parameter for CR[UFC23_CR_FEP_TDC_TRIM_INDEX]
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      = 0;
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      |= UFC23_C_TDC_SEL_QHA1_SET         (ufc23->Param.CR_A9.C_TDC_SEL_QHA1        );
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      |= UFC23_C_TDC_SEL_QHA2_SET         (ufc23->Param.CR_A9.C_TDC_SEL_QHA2        );
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      |= UFC23_C_TDC_PHS_MODE_SET         (ufc23->Param.CR_A9.C_TDC_PHS_MODE        );
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      |= UFC23_C_PHS_CELLS_SET            (ufc23->Param.CR_A9.C_PHS_CELLS           );
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      |= UFC23_C_PHS_INCR_SET             (ufc23->Param.CR_A9.C_PHS_INCR            );
    ufc23->CR[UFC23_CR_FEP_TDC_TRIM_INDEX]      |= UFC23_C_TDC_HR_ADJUST_SET        (ufc23->Param.CR_A9.C_TDC_HR_ADJUST       );
    // Parameter for CR[UFC23_CR_USM_PROC_INDEX]
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          = 0;
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_USM_MASK_WIN_SET         (ufc23->Param.CR_AA.C_USM_MASK_WIN        );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_USM_MHIT_BATCH_SET       (ufc23->Param.CR_AA.C_USM_MHIT_BATCH      );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_USM_SENSOR_MODE_SET      (ufc23->Param.CR_AA.C_USM_SENSOR_MODE     );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_USM_AM_MODE_SET          (ufc23->Param.CR_AA.C_USM_AM_MODE         );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_USM_PWD_MODE_SET         (ufc23->Param.CR_AA.C_USM_PWD_MODE        );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_ZCD_LVL_SET              (ufc23->Param.CR_AA.C_ZCD_LVL             );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_ZCC_MODE_SET             (ufc23->Param.CR_AA.C_ZCC_MODE            );
    ufc23->CR[UFC23_CR_USM_PROC_INDEX]          |= UFC23_C_ZCC_INIT_EN_SET          (ufc23->Param.CR_AA.C_ZCC_INIT_EN         );
    // Parameter for CR[UFC23_CR_USM_FBG_MCTRL_INDEX]
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     = 0;
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     |= UFC23_C_FBG_SEL_SET              (ufc23->Param.CR_AB.C_FBG_SEL            );
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     |= UFC23_C_FBG_LR_CLK_DIV_SET       (ufc23->Param.CR_AB.C_FBG_LR_CLK_DIV     );
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     |= UFC23_C_FBG_FBNUM_SET            (ufc23->Param.CR_AB.C_FBG_FBNUM          );
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     |= UFC23_C_FBG_FBSP_SET             (ufc23->Param.CR_AB.C_FBG_FBSP           );
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     |= UFC23_C_FSPLITWID_SET            (ufc23->Param.CR_AB.C_FSPLITWID          );
    ufc23->CR[UFC23_CR_USM_FBG_MCTRL_INDEX]     |= UFC23_C_FBG_HR_CLK_DIV_SET       (ufc23->Param.CR_AB.C_FBG_HR_CLK_DIV     );
    // Parameter for CR[UFC23_CR_USM_FBG_HRC_INDEX]
    ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX]       = 0;
    ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX]       |= UFC23_C_FBG_HR_CALIB_SET         (ufc23->Param.CR_AC.C_FBG_HR_CALIB       );
    ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX]       |= UFC23_C_FBG_HR_CLB_SEL_SET       (ufc23->Param.CR_AC.C_FBG_HR_CLB_SEL     );
    ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX]       |= UFC23_C_FBG_HR_TRIM_SET          (ufc23->Param.CR_AC.C_FBG_HR_TRIM        );
    ufc23->CR[UFC23_CR_USM_FBG_HRC_INDEX]       |= UFC23_C_FBG_CALIB_MODE_SET       (ufc23->Param.CR_AC.C_FBG_CALIB_MODE     );
    // Parameter for CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     = 0;
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_HS_OSC_TRIM_SET          (ufc23->Param.CR_AD.C_HS_OSC_TRIM        );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_HS_OSC_CFG_SET           (ufc23->Param.CR_AD.C_HS_OSC_CFG         );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_LS_OSC_CFG_SET           (ufc23->Param.CR_AD.C_LS_OSC_CFG         );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_PMU_BG_TRIM_SET          (ufc23->Param.CR_AD.C_PMU_BG_TRIM        );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_PMU_BIAS_TRIM_SET        (ufc23->Param.CR_AD.C_PMU_BIAS_TRIM      );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_PMU_LDO_SEL_SET          (ufc23->Param.CR_AD.C_PMU_LDO_SEL        );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_ZCD_IBSEL_SET            (ufc23->Param.CR_AD.C_ZCD_IBSEL          );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_ZCD_DAC_VREFN_SEL_SET    (ufc23->Param.CR_AD.C_ZCD_DAC_VREFN_SEL  );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_PGA_GPIO_SEL_SET         (ufc23->Param.CR_AD.C_PGA_GPIO_SEL       );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_TX_CAP_MODE_SET          (ufc23->Param.CR_AD.C_TX_CAP_MODE        );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL1_INDEX]     |= UFC23_C_USVREF_CAP_EN_SET        (ufc23->Param.CR_AD.C_USVREF_CAP_EN      );
    // Parameter for CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     = 0;
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST2_GAIN_SET         (ufc23->Param.CR_AE.C_PGA_ST2_GAIN       );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST2_CBYP_SET         (ufc23->Param.CR_AE.C_PGA_ST2_CBYP       );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST1_GAIN_SET         (ufc23->Param.CR_AE.C_PGA_ST1_GAIN       );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST1_CBYP_SET         (ufc23->Param.CR_AE.C_PGA_ST1_CBYP       );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_G1_OPEN_SET          (ufc23->Param.CR_AE.C_PGA_G1_OPEN        );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ISEL_SET             (ufc23->Param.CR_AE.C_PGA_ISEL           );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST1_OPAN_ENA_SET     (ufc23->Param.CR_AE.C_PGA_ST1_OPAN_ENA   );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST1_OPAP_ENA_SET     (ufc23->Param.CR_AE.C_PGA_ST1_OPAP_ENA   );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_PGA_ST2_OPA_ENA_SET      (ufc23->Param.CR_AE.C_PGA_ST2_OPA_ENA    );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_COMPSEL_SEL_SET          (ufc23->Param.CR_AE.C_COMPSEL_SEL        );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_R_COMPSEL_SET            (ufc23->Param.CR_AE.C_R_COMPSEL          );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_C_COMPSEL_SET            (ufc23->Param.CR_AE.C_C_COMPSEL          );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_RMSET_RX_SET             (ufc23->Param.CR_AE.C_RMSET_RX           );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_RMSET_TX_SET             (ufc23->Param.CR_AE.C_RMSET_TX           );
    ufc23->CR[UFC23_CR_FEP_ANA_CTRL2_INDEX]     |= UFC23_C_SE_ENABLE_SET            (ufc23->Param.CR_AE.C_SE_ENABLE          );
    // Parameter for CR[UFC23_CR_USM_RCV_INIT_INDEX]
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      = 0;
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_US_VR_INIT_SET           (ufc23->Param.CR_AF.C_US_VR_INIT         );
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_PGA_INIT_SET             (ufc23->Param.CR_AF.C_PGA_INIT           );
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_COMP_INIT_SET            (ufc23->Param.CR_AF.C_COMP_INIT          );
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_ADC_VR_INIT_SET          (ufc23->Param.CR_AF.C_ADC_VR_INIT        );
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_ADC_INIT_SET             (ufc23->Param.CR_AF.C_ADC_INIT           );
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_USM_INIT_MODE_SET        (ufc23->Param.CR_AF.C_USM_INIT_MODE      );
    ufc23->CR[UFC23_CR_USM_RCV_INIT_INDEX]      |= UFC23_C_DCO_INIT_SET             (ufc23->Param.CR_AF.C_DCO_INIT           );
    // Parameter for CR[UFC23_CR_USM_HIT_CTRL_INDEX]
    ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX]      = 0;
    ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX]      |= UFC23_C_TOF_HIT_NO_SET           (ufc23->Param.CR_B0.C_TOF_HIT_NO         );
    ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX]      |= UFC23_C_TOF_HIT_RLS_MODE_SET     (ufc23->Param.CR_B0.C_TOF_HIT_RLS_MODE   );
    ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX]      |= UFC23_C_TOF_HIT_IGN_MODE_SET     (ufc23->Param.CR_B0.C_TOF_HIT_IGN_MODE   );
    ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX]      |= UFC23_C_TOF_MULTIHIT_START_SET   (ufc23->Param.CR_B0.C_TOF_MULTIHIT_START );
    ufc23->CR[UFC23_CR_USM_HIT_CTRL_INDEX]      |= UFC23_C_TOF_MULTIHIT_NO_SET      (ufc23->Param.CR_B0.C_TOF_MULTIHIT_NO    );
    // Parameter for CR[UFC23_CR_USM_WVM_INDEX]
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           = 0;
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           |= UFC23_C_USM_FHL_UP_SET           (ufc23->Param.CR_B1.C_USM_FHL_UP         );
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           |= UFC23_C_USM_FHL_DN_SET           (ufc23->Param.CR_B1.C_USM_FHL_DN         );
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           |= UFC23_C_USM_AM_PD_1_SET          (ufc23->Param.CR_B1.C_USM_AM_PD_1        );
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           |= UFC23_C_USM_AM_PD_2_SET          (ufc23->Param.CR_B1.C_USM_AM_PD_2        );
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           |= UFC23_C_USM_AM_PD_3_SET          (ufc23->Param.CR_B1.C_USM_AM_PD_3        );
    ufc23->CR[UFC23_CR_USM_WVM_INDEX]           |= UFC23_C_AM_VEXTSEL_SET           (ufc23->Param.CR_B1.C_AM_VEXTSEL         );
    // Parameter for CR[UFC23_CR_USM_MASK_HR_WIN_INDEX]
    ufc23->CR[UFC23_CR_USM_MASK_HR_WIN_INDEX]   = 0;
    ufc23->CR[UFC23_CR_USM_MASK_HR_WIN_INDEX]   |= UFC23_C_USM_MASK_HR_WIN_UP_SET   (ufc23->Param.CR_B2.C_USM_MASK_HR_WIN_UP );
    ufc23->CR[UFC23_CR_USM_MASK_HR_WIN_INDEX]   |= UFC23_C_USM_MASK_HR_WIN_DN_SET   (ufc23->Param.CR_B2.C_USM_MASK_HR_WIN_DN );

    Ufc23_UpdateAmountBundlesInBatch(ufc23);
}

#undef wait
#undef hasAnyFlag
#undef hasFlag
#undef memcpy

#endif // SCIOSENSE_ENS21X_INL_C_H
