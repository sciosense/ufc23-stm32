#ifndef SCIOSENSE_UFC23_C_H
#define SCIOSENSE_UFC23_C_H

#include "ScioSense_Ufc23_defines.h"

#include <stdbool.h>
#include <inttypes.h>
#include "stddef.h"

typedef struct ScioSense_Ufc23_IO
{
    Result  (*transfer) (void* config, uint8_t* dataToWrite, const size_t sizeToWrite, uint8_t* dataToRead, const size_t sizeToRead);
    Result  (*write)    (void* config, uint8_t* data, const size_t size);
    void    (*wait)     (const uint32_t ms);
    void*   config;
} ScioSense_Ufc23_IO;

typedef enum _Ufc23_StateTypeDef
{
    UFC23_STATE_RESET           = 0x00U,    // Device not Initialized
    UFC23_STATE_BOOTLOAD        = 0x01U,    // Device bootloading not completed
    UFC23_STATE_STANDBY         = 0x02U,    // An internal process is ongoing
    UFC23_STATE_INIT            = 0x03U,    // Device interrupt state
    UFC23_STATE_MEAS            = 0x05U,    // Device measuring
    UFC23_STATE_NOT_CONNECTED   = 0xFFU,    // Device not connect
} Ufc23_StateTypeDef;

typedef enum _UFC23_OperationModeTypeDef
{
    UFC23_SINGLE_CYCLE_MODE     = 0x00U,    // Only one measurement transmitted. Can read individual hits
    UFC23_BUNDLE_2_USM_MODE     = 0x01U,    // Reads 2 measurements in one transmission
    UFC23_BUNDLE_4_USM_MODE     = 0x02U,    // Reads 4 measurements in one transmission
    UFC23_BUNDLE_6_USM_MODE     = 0x03U,    // Reads 6 measurements in one transmission
    UFC23_BUNDLE_8_USM_MODE     = 0x04U,    // Reads 8 measurements in one transmission
    UFC23_BUNDLE_10_USM_MODE    = 0x05U,    // Reads 10 measurements in one transmission
    UFC23_BUNDLE_12_USM_MODE    = 0x06U     // Reads 12 measurements in one transmission
} Ufc23_OpModeTypeDef;

typedef enum _UFC23_FrontendErrorFlagTypeDef
{
    TDC_TO_HCC      = UFC23_C_EF_EN_HCC_TDC_TMO_Pos,
    TDC_TO_TM       = UFC23_C_EF_EN_TM_TDC_TMO_Pos,
    TM_OPEN         = UFC23_C_EF_EN_TM_OC_ERR_Pos,
    TM_SHORT        = UFC23_C_EF_EN_TM_SC_ERR_Pos,
    USM_HW_ERR_UP   = UFC23_C_EF_EN_USM_HW_UP_ERR_Pos,
    USM_HW_ERR_DN   = UFC23_C_EF_EN_USM_HW_DN_ERR_Pos,
    TDC_TO_PW_UP    = UFC23_C_EF_EN_PW_UP_TDC_TMO_Pos,
    TDC_TO_PW_DN    = UFC23_C_EF_EN_PW_DN_TDC_TMO_Pos,
    TDC_TO_TOF_UP   = UFC23_C_EF_EN_TOF_UP_TDC_TMO_Pos,
    TDC_TO_TOF_DN   = UFC23_C_EF_EN_TOF_DN_TDC_TMO_Pos,
    USM_TO_TOF_UP   = UFC23_C_EF_EN_USM_UP_TMO_Pos,
    USM_TO_TOF_DN   = UFC23_C_EF_EN_USM_DN_TMO_Pos
} Ufc23_FrontendErrorFlag;

typedef enum
{
    ULTRASONIC_MEAS     = UFC23_EXTENDED_COMMAND_EC_USM_REQ,
    ZERO_CROSS_CAL      = UFC23_EXTENDED_COMMAND_EC_ZCC_REQ,
    VCC_MEAS            = UFC23_EXTENDED_COMMAND_EC_VCCM_REQ,
    HIGH_SPEED_OSC_CAL  = UFC23_EXTENDED_COMMAND_EC_HCC_REQ,
    FIRE_BURST_CAL      = UFC23_EXTENDED_COMMAND_EC_FBC_REQ,
    TEMPERATURE_MEAS    = UFC23_EXTENDED_COMMAND_EC_TM_REQ
} Ufc23_CycleTaskRequest;

typedef enum
{
    NOT_INITIALIZED,
    UFC18_SENSOR,
    UFC23_SENSOR,
    UNKNOWN
} Ufc23_PartID;

typedef struct {
    uint16_t AMPL1;
    uint16_t AMPL2;
    uint16_t AMPL3;
} UFC23_AMP_Raw_TypeDef;

typedef struct {
    float AMPL1;
    float AMPL2;
    float AMPL3;
} UFC23_AMP_V_TypeDef;

typedef struct {
    uint16_t PW1_FHL;
    uint16_t PW2_FHL;
    uint16_t PW_ZCL;
} UFC23_PW_Raw_TypeDef;

typedef struct {
    float PW1_FHL;
    float PW2_FHL;
} UFC23_PW_Ps_TypeDef;

typedef struct {
    uint8_t C_IRQ_EN_BL_DONE;
    uint8_t C_IRQ_EN_MIS_DONE;
    uint8_t C_IRQ_EN_MCS_DONE;
    uint8_t C_IRQ_EN_MC_BATCH_DONE;
    uint8_t C_IRQ_EN_STASK_DONE;
    uint8_t C_IRQ_EN_USM_PAUSE_ERR;
    uint8_t C_IRQ_EN_TSC_TMO;
} UFC23_CR_A0_TypeDef;

typedef struct {
    uint8_t C_EF_EN_HCC_TDC_TMO;
    uint8_t C_EF_EN_TM_TDC_TMO;
    uint8_t C_EF_EN_TM_OC_ERR;
    uint8_t C_EF_EN_TM_SC_ERR;
    uint8_t C_EF_EN_USM_HW_UP_ERR;
    uint8_t C_EF_EN_USM_HW_DN_ERR;
    uint8_t C_EF_EN_PW_UP_TDC_TMO;
    uint8_t C_EF_EN_PW_DN_TDC_TMO;
    uint8_t C_EF_EN_TOF_UP_TDC_TMO;
    uint8_t C_EF_EN_TOF_DN_TDC_TMO;
    uint8_t C_EF_EN_USM_UP_TMO;
    uint8_t C_EF_EN_USM_DN_TMO;
} UFC23_CR_A1_TypeDef;

typedef struct {
    uint8_t C_GPIO0_MODE;
    uint8_t C_GPIO1_MODE;
    uint8_t C_GPIO2_MODE;
    uint8_t C_GPIO3_MODE;
    uint8_t C_PRB_SEL;
    uint8_t C_TST_STM;
    uint8_t C_MISO_HZ_DIS;
} UFC23_CR_A2_TypeDef;

typedef struct {
    uint8_t C_LDO_RF_RATE;
    uint8_t C_VDD18_SW_MODE;
} UFC23_CR_A3_TypeDef;

typedef struct {
    uint8_t C_USM_PAUSE_TSEL;
    uint8_t C_USM_REPEAT;
    uint8_t C_USM_DIR_MODE;
    uint8_t C_USM_EDGE_MODE;
    uint8_t C_LDO_STUP_TSEL;
    uint8_t C_LDO_FEP_MODE;
    uint8_t C_TM_SQC_MODE;
} UFC23_CR_A4_TypeDef;

typedef struct {
    uint16_t    C_MCYCLE_TIME;
    uint8_t     C_MCYCLE_TAIL_SEL;
    uint8_t     C_MCT_EN;
} UFC23_CR_A5_TypeDef;

typedef struct {
    uint8_t     C_USM_RATE;
    uint8_t     C_ZCC_RATE;
    uint8_t     C_VCCM_RATE;
    uint8_t     C_HCC_RATE;
    uint8_t     C_FBC_RATE;
    uint16_t    C_TM_RATE;
} UFC23_CR_A6_TypeDef;

typedef struct {
    uint8_t     C_FEP_4M_CLK_DIV;
    uint8_t     C_ADC_ST;
    uint8_t     C_FEP_STUP_TSEL;
    uint8_t     C_USM_TMO_SEL;
    uint8_t     C_TM_PORT_NO;
    uint8_t     C_TM_PORT_MODE;
    uint8_t     C_TM_CYCLE_SEL;
    uint8_t     C_TM_SQC_NO;
    uint8_t     C_HF_CLB_SEL;
    uint8_t     C_HF_SERIAL;
    uint16_t    C_HF_TRIM;
    uint8_t     C_HF_CALIB_MODE;
    uint8_t     C_HF_DCO_ENA;
} UFC23_CR_A7_TypeDef;

typedef struct {
    uint8_t     C_TDC_SEL_QHA1;
    uint8_t     C_TDC_SEL_QHA2;
    uint8_t     C_TDC_PHS_MODE;
    uint8_t     C_PHS_CELLS;
    uint8_t     C_PHS_INCR;
    uint16_t    C_TDC_HR_ADJUST;
} UFC23_CR_A9_TypeDef;

typedef struct {
    uint16_t    C_USM_MASK_WIN;
    uint8_t     C_USM_MHIT_BATCH;
    uint8_t     C_USM_SENSOR_MODE;
    uint8_t     C_USM_AM_MODE;
    uint8_t     C_USM_PWD_MODE;
    uint16_t    C_ZCD_LVL;
    uint8_t     C_ZCC_MODE;
    uint8_t     C_ZCC_INIT_EN;
} UFC23_CR_AA_TypeDef;

typedef struct {
    uint8_t C_FBG_SEL;
    uint8_t C_FBG_LR_CLK_DIV;
    uint8_t C_FBG_FBNUM;
    uint8_t C_FBG_FBSP;
    uint8_t C_FSPLITWID;
    uint8_t C_FBG_HR_CLK_DIV;
} UFC23_CR_AB_TypeDef;

typedef struct {
    uint16_t    C_FBG_HR_CALIB;
    uint8_t     C_FBG_HR_CLB_SEL;
    uint16_t    C_FBG_HR_TRIM;
    uint8_t     C_FBG_CALIB_MODE;
} UFC23_CR_AC_TypeDef;

typedef struct {
    uint8_t C_HS_OSC_TRIM;
    uint8_t C_HS_OSC_CFG;
    uint8_t C_LS_OSC_CFG;
    uint8_t C_PMU_BG_TRIM;
    uint8_t C_PMU_BIAS_TRIM;
    uint8_t C_PMU_LDO_SEL;
    uint8_t C_ZCD_IBSEL;
    uint8_t C_ZCD_DAC_VREFN_SEL;
    uint8_t C_PGA_GPIO_SEL;
    uint8_t C_TX_CAP_MODE;
    uint8_t C_USVREF_CAP_EN;
} UFC23_CR_AD_TypeDef;

typedef struct {
    uint8_t C_PGA_ST2_GAIN;
    uint8_t C_PGA_ST2_CBYP;
    uint8_t C_PGA_ST1_GAIN;
    uint8_t C_PGA_ST1_CBYP;
    uint8_t C_PGA_G1_OPEN;
    uint8_t C_PGA_ISEL;
    uint8_t C_PGA_ST1_OPAN_ENA;
    uint8_t C_PGA_ST1_OPAP_ENA;
    uint8_t C_PGA_ST2_OPA_ENA;
    uint8_t C_COMPSEL_SEL;
    uint8_t C_R_COMPSEL;
    uint8_t C_C_COMPSEL;
    uint8_t C_RMSET_RX;
    uint8_t C_RMSET_TX;
    uint8_t C_SE_ENABLE;
} UFC23_CR_AE_TypeDef;

typedef struct {
    uint8_t C_US_VR_INIT;
    uint8_t C_PGA_INIT;
    uint8_t C_COMP_INIT;
    uint8_t C_ADC_VR_INIT;
    uint8_t C_ADC_INIT;
    uint8_t C_USM_INIT_MODE;
    uint8_t C_DCO_INIT;
} UFC23_CR_AF_TypeDef;

typedef struct {
    uint8_t C_TOF_HIT_NO;
    uint8_t C_TOF_HIT_RLS_MODE;
    uint8_t C_TOF_HIT_IGN_MODE;
    uint8_t C_TOF_MULTIHIT_START;
    uint8_t C_TOF_MULTIHIT_NO;
} UFC23_CR_B0_TypeDef;

typedef struct {
    uint8_t C_USM_FHL_UP;
    uint8_t C_USM_FHL_DN;
    uint8_t C_USM_AM_PD_1;
    uint8_t C_USM_AM_PD_2;
    uint8_t C_USM_AM_PD_3;
    uint8_t C_AM_VEXTSEL;
} UFC23_CR_B1_TypeDef;

typedef struct {
    uint16_t C_USM_MASK_HR_WIN_UP;
    uint16_t C_USM_MASK_HR_WIN_DN;
} UFC23_CR_B2_TypeDef;

typedef struct {
    UFC23_CR_A0_TypeDef CR_A0;
    UFC23_CR_A1_TypeDef CR_A1;
    UFC23_CR_A2_TypeDef CR_A2;
    UFC23_CR_A3_TypeDef CR_A3;
    UFC23_CR_A4_TypeDef CR_A4;
    UFC23_CR_A5_TypeDef CR_A5;
    UFC23_CR_A6_TypeDef CR_A6;
    UFC23_CR_A7_TypeDef CR_A7;
    UFC23_CR_A9_TypeDef CR_A9;
    UFC23_CR_AA_TypeDef CR_AA;
    UFC23_CR_AB_TypeDef CR_AB;
    UFC23_CR_AC_TypeDef CR_AC;
    UFC23_CR_AD_TypeDef CR_AD;
    UFC23_CR_AE_TypeDef CR_AE;
    UFC23_CR_AF_TypeDef CR_AF;
    UFC23_CR_B0_TypeDef CR_B0;
    UFC23_CR_B1_TypeDef CR_B1;
    UFC23_CR_B2_TypeDef CR_B2;
} UFC23_ParamTypeDef;

typedef struct ScioSense_Ufc23
{
    ScioSense_Ufc23_IO          io;
    UFC23_REG_SIZE              CR[UFC23_AMOUNT_CONFIGURATION_REGISTERS];
    UFC23_REG_ADD_SIZE          Addresses[UFC23_AMOUNT_CONFIGURATION_REGISTERS];
    UFC23_COM_SIZE              DataBuffer[UFC23_AMOUNT_USM_BATCH_BYTES];

    UFC23_ParamTypeDef          Param;                                              // Configuration Parameter
    Ufc23_StateTypeDef          State;                                              // Status of UFC23
    Ufc23_OpModeTypeDef         OperationMode;                                      // Operation mode of UFC23
    UFC23_BATCH_AMOUNT_SIZE     amountCycles;                                       // Amount of cycles to be used in the batch mode. Zero means Single Cycle Mode
    UFC23_BATCH_AMOUNT_SIZE     cyclesInBatch;                                      // Amount of cycles received in the measurement batch of the device
    UFC23_FR_SIZE               frontendStatusFlags;                                // Measurements updated corresponding to Frontend Status Flag register
    UFC23_FR_FE_SIZE            frontendErrorFlags;                                 // Errors reported in the Frontend Error Flag Register
    UFC23_CYCLE_TIME_SIZE       measureCycleTimeUs;                                 // Measure Cycle Time in microseconds
    Ufc23_PartID                partId;                                             // Part ID of the device
    float                       correctionFactorHso;                                // Correction factor to use with TDC results
    float                       pwLsbNs;                                            // Value of the LSB of a Pulse Width measurement in nanoseconds
    float                       tofLsbNs;                                           // Value of the LSB of a Time of Flight measurement in nanoseconds
    float                       zeroCrossCalibration;                               // Last measured Zero Cross Calibration value
} ScioSense_Ufc23;

static inline Result                    Ufc23_Reset                                 (ScioSense_Ufc23* ufc23);               // Executes reset of all digital blocks 
static inline Result                    Ufc23_Init                                  (ScioSense_Ufc23* ufc23);               // Resets the device, waits and checks for the end of the bootload sequence, read initial setup measurements
static inline uint8_t                   Ufc23_IsConnected                           (ScioSense_Ufc23* ufc23);               // Checks if a successful communication was done to the device
static inline Result                    Ufc23_Update                                (ScioSense_Ufc23* ufc23);               // If no communication or frontend errors are reported, it reads the frontend data
static inline Result                    Ufc23_StartCyclingMeasurement               (ScioSense_Ufc23* ufc23);               // Start the cycling measurement timer, so the scheduled measurement tasks will be performed
static inline Result                    Ufc23_DetectEndBootLoadSequence             (ScioSense_Ufc23* ufc23);               // Checks if the bootload has been completed
static inline Result                    Ufc23_SetStandbyState                       (ScioSense_Ufc23* ufc23);               // Sets the sensor into Standby mode

static inline uint8_t                   Ufc23_IsPartIdValid                         (ScioSense_Ufc23* ufc23);               // Returns 1 if the part ID is a UFC18 or UFC23
static inline Result                    Ufc23_GetPartId                             (ScioSense_Ufc23* ufc23);               // Gets the Part ID of the sensor

static inline Result                    Ufc23_WriteRemoteCommand                    (ScioSense_Ufc23* ufc23, uint8_t remoteCommand, uint16_t extendedCommand);                                                  // Writes a remote command to the sensor
static inline Result                    Ufc23_ReadRemoteCommand                     (ScioSense_Ufc23* ufc23, uint8_t remoteCommand, uint16_t extendedCommand, uint8_t* dataToRead, uint16_t dataToReadSize);    // Writes a remote command and then reads the data response
static inline Result                    Ufc23_WriteDWordRAM                         (ScioSense_Ufc23* ufc23, uint16_t RAMAddress, UFC23_REG_SIZE registerContent);                                              // Writes one DWORD (4 bytes) register at the provided RAM address
static inline Result                    Ufc23_ReadDWordRAM                          (ScioSense_Ufc23* uf23, uint16_t RAMAddress, UFC23_COM_SIZE* registerContents, uint16_t registersToRead);                   // Reads the specified amount of  DWORD (4 bytes) register from the provided RAM address
static inline uint32_t                  Ufc23_ByteArrayToDWord                      (uint8_t* byteArray, uint16_t startIndex);                                                                                  // Creates a 4 byte register from a MSB first byte array

static inline Result                    Ufc23_GetMode                               (ScioSense_Ufc23* ufc23);                                                       // Gets the current state of the UFC23 state machine
static inline Result                    Ufc23_EnableMeasureMode                     (ScioSense_Ufc23* ufc23);                                                       // Enable the Measure Cycle Timer and set the device into Measure mode
static inline Result                    Ufc23_DisableMeasureMode                    (ScioSense_Ufc23* ufc23);                                                       // Disable the Measure Cycle Timer and set the device into Standby mode
static inline Result                    Ufc23_EnableMeasureModeTimerHalted          (ScioSense_Ufc23* ufc23);                                                       // Enable the Measure Cycle Mode with the Timer disabled (measurements must be manually triggered)
static inline Result                    Ufc23_HaltMeasureTimer                      (ScioSense_Ufc23* ufc23);                                                       // Halt the Measure Cycle Timer
static inline Result                    Ufc23_ReleaseHaltMeasureTimer               (ScioSense_Ufc23* ufc23);                                                       // Release the halt on the Measure Cycle Timer but keep the device mode

static inline UFC23_BATCH_AMOUNT_SIZE   Ufc23_GetAmountMeasurementsInBatch          (ScioSense_Ufc23* ufc23);                                                       // Returns the amount of measurements on each batch
static inline UFC23_FR_SIZE             Ufc23_GetCommunicationFlagRegister          (ScioSense_Ufc23* ufc23);                                                       // Read the Communication flag register from the device
static inline UFC23_FR_SIZE             Ufc23_GetInterruptFlagRegister              (ScioSense_Ufc23* ufc23);                                                       // Read the Interrupt flag register from the device
static inline UFC23_FR_FE_SIZE          Ufc23_GetFrontendErrorFlagRegister          (ScioSense_Ufc23* ufc23);                                                       // Read the Frontend Error flag register from the device
static inline UFC23_FR_SIZE             Ufc23_GetFrontendStatusFlagRegister         (ScioSense_Ufc23* ufc23);                                                       // Read the Frontend Status flag register from the device
static inline UFC23_FR_SIZE             Ufc23_GetSystemStatusFlagRegister           (ScioSense_Ufc23* ufc23);                                                       // Read the System Status flag register from the device
static inline Result                    Ufc23_ClearFlagRegisters                    (ScioSense_Ufc23* ufc23);                                                       // Clears all the command-clearable flags from the flag resigsters
static inline Result                    Ufc23_TriggerSingleMeasurement              (ScioSense_Ufc23* ufc23, Ufc23_CycleTaskRequest requestType);                   // Puts the sensor into Standby and starts one of the indicated cycle task
static inline Result                    Ufc23_TriggerTransducerPortOpenMeasurement  (ScioSense_Ufc23* ufc23);                                                       // Puts the sensor into Standby and starts a measurement of the spool condition
static inline UFC23_FR_FE_SIZE          Ufc23_ErrorsPresentInLastUpdate             (ScioSense_Ufc23* ufc23);                                                       // Returns the last value read of the Frontend error flag register

static inline Result                    Ufc23_GetUSMData                            (ScioSense_Ufc23* ufc23);                                                       // Read the whole USM section of the RAM
static inline void                      Ufc23_UpdateCorrectionFactorHso             (ScioSense_Ufc23* ufc23, uint32_t rmHsoCalib, float lsoNominalFrequencyHz);     // Use the High Speed Oscillator calibration value to update the Calibration Factor HSO
static inline void                      Ufc23_UpdatePulseWidthLsb                   (ScioSense_Ufc23* ufc23, float correctionFactor, float nominalFrequencyHz);     // Use the Correction Factor HSO to calculate the conversion factor of the Pulse Width into nanoseconds
static inline void                      Ufc23_UpdateTimeOfFlightLsb                 (ScioSense_Ufc23* ufc23, float correctionFactor, float nominalFrequencyHz);     // Use the Correction Factor HSO to calculate the conversion factor of the Time of Flight into nanoseconds
static inline void                      Ufc23_SetMeasureCycleTimeUs                 (ScioSense_Ufc23* ufc23);                                                       // Updates the configured MeasureCycleTime in microseconds
static inline UFC23_CYCLE_TIME_SIZE     Ufc23_GetMeasureCycleTimeUs                 (ScioSense_Ufc23* ufc23);                                                       // Returns the configured MeasureCycleTime in microseconds
static inline void                      Ufc23_UpdateAmountBundlesInBatch            (ScioSense_Ufc23* ufc23);                                                       // Set the amount of bundle on each batch from the sensor configuration

static inline uint16_t                  Ufc23_GetConfigurationRegisterAddress       (ScioSense_Ufc23* ufc23, uint8_t idx);                                          // Get the RAM address of the configuration register corresponding to the provided index
static inline uint32_t                  Ufc23_GetConfigurationRegisterSetting       (ScioSense_Ufc23* ufc23, uint8_t idx);                                          // Get the configured content of the configuration register corresponding to the provided index
static inline Result                    Ufc23_WriteConfig                           (ScioSense_Ufc23* ufc23);                                                       // Write the configured content of the configuration registers into the sensor
static inline Result                    Ufc23_ReadConfig                            (ScioSense_Ufc23* ufc23);                                                       // Read the configuration from the device, and if no error ocurred, copies it to the UFC23 object
static inline void                      Ufc23_UpdateParameters                      (ScioSense_Ufc23* ufc23);                                                       // Update the struct configuration parameters with the content of the configuration array
static inline void                      Ufc23_UpdateConfiguration                   (ScioSense_Ufc23* ufc23);                                                       // Update the content of the configuration array with the struct configuration parameters
static inline void                      Ufc23_SetConfigurationRegisters             (ScioSense_Ufc23* ufc23, uint32_t* registerConfiguration);                      // Update the configuration data using the provided array
static inline void                      Ufc23_InitializeConfiguration               (ScioSense_Ufc23* ufc23);                                                       // Set the configuration data to the default

static inline uint32_t                  Ufc23_ParseUsmDWordValue                    (ScioSense_Ufc23* ufc23, uint16_t dWordIdx);                                                                                    // Returns the DWORD corresponding to the provided index from the last USM data update
static inline uint32_t                  Ufc23_ParseUsmBatchDWordValue               (ScioSense_Ufc23* ufc23, uint8_t bundleIdx, uint8_t dWordIdxInBatch);                                                           // Returns the DWORD corresponding to the provided index relative to the provided batch from the last USM data update
static inline uint32_t                  Ufc23_ParseErrorFlags                       (ScioSense_Ufc23* ufc23, uint8_t batchIndex);                                                                                   // Returns the Frontend error flags corresponding to the provided bundle index from the last USM data update
static inline uint8_t                   Ufc23_ParseAmplitudeRaw                     (ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_AMP_Raw_TypeDef* amplitudesRawUp, UFC23_AMP_Raw_TypeDef* amplitudesRawDn);   // Returns the raw amplitude values located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParsePulseWidthRaw                    (ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_PW_Raw_TypeDef* pulseWidthsRawUp, UFC23_PW_Raw_TypeDef* pulseWidthsRawDn);   // Returns the raw pulse width values located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseTofMultiHitSumRaw                (ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint64_t* tofMultiHitUp, uint64_t* tofMultiHitDn);                                 // Returns the raw multi hit sum values located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseTofMultiHitsCount                (ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint8_t* multiHitCountUp, uint8_t* multiHitCountDn);                               // Returns the amount of ToF hits located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseVddRaw                           (ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint16_t* vdd, uint16_t* vcc);                                                     // Returns the raw supply voltage values located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseHccCalibRaw                      (ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint32_t* hccCalibration);                                                         // Returns the raw High Speed Calibration value located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseZcLvlRaw                         (ScioSense_Ufc23* ufc23, uint8_t batchIndex, uint16_t* zcLvl);                                                                  // Returns the raw Zero Cross Level value located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseSingleCycleUsTofHitsRaw          (ScioSense_Ufc23* ufc23, uint32_t* usTofHitUp, uint32_t* usTofHitDn, uint8_t* amountHitsUp, uint8_t* amountHitsDn);             // Returns the raw single measurement hits from the last USM data update if it is a valid measurement

static inline uint8_t                   Ufc23_ParseAmplitudeV                       (ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_AMP_V_TypeDef* amplitudesVUp, UFC23_AMP_V_TypeDef* amplitudesVDn);           // Returns the amplitude values in volts located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParsePulseWidthRatio                  (ScioSense_Ufc23* ufc23, uint8_t batchIndex, UFC23_PW_Ps_TypeDef* pulseWidthsRatioUp, UFC23_PW_Ps_TypeDef* pulseWidthsRatioDn); // Returns the pulse width ratio values located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseTofMultiHitNs                    (ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* tofMultiHitUp, float* tofMultiHitDn);                                       // Returns the multi hit sum values in nanoseconds located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseVddV                             (ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* vdd, float* vcc);                                                           // Returns the supply voltage values in volts located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseHsoMhz                           (ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* fHsoMhz);                                                                   // Returns the High Speed Calibration value in MHz located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseZcLvlV                           (ScioSense_Ufc23* ufc23, uint8_t batchIndex, float* zcLvl);                                                                     // Returns the Zero Cross Level value in volts located at the specified bundle index from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseSingleCycleUsTofHitsNs           (ScioSense_Ufc23* ufc23, float* usTofHitUp, float* usTofHitDn, uint8_t* amountHitsUp, uint8_t* amountHitsDn);                   // Returns the single measurement hits in nanoseconds from the last USM data update if it is a valid measurement
static inline uint8_t                   Ufc23_ParseTemperatureSeq1degC              (ScioSense_Ufc23* ufc23, float* temperature1DegC, float* temperature2DegC);                                                     // Returns the temperature values in degrees Centigrades from port 1 and 2 calculated in sequence 1 if it is a valid measurement
static inline uint8_t                   Ufc23_ParseTemperatureSeq2degC              (ScioSense_Ufc23* ufc23, float* temperature1DegC, float* temperature2DegC);                                                     // Returns the temperature values in degrees Centigrades from port 1 and 2 calculated in sequence 2 if it is a valid measurement

static inline uint8_t                   Ufc23_ParseBatchAmplitudeRaw                (ScioSense_Ufc23* ufc23, UFC23_AMP_Raw_TypeDef* amplitudesRawUp, UFC23_AMP_Raw_TypeDef* amplitudesRawDn);       // Returns the raw amplitude values from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchPulseWidthRaw               (ScioSense_Ufc23* ufc23, UFC23_PW_Raw_TypeDef* pulseWidthsRawUp, UFC23_PW_Raw_TypeDef* pulseWidthsRawDn);       // Returns the raw pulse width values from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchTofMultiHitSumRaw           (ScioSense_Ufc23* ufc23, uint64_t* tofMultiHitUp_Raw, uint64_t* tofMultiHitDn_Raw);                             // Returns the raw multi hit sum values from all the measurement bundels in the batch

static inline uint8_t                   Ufc23_ParseBatchAmplitudeV                  (ScioSense_Ufc23* ufc23, UFC23_AMP_V_TypeDef* amplitudesVUp, UFC23_AMP_V_TypeDef* amplitudesVDn);               // Returns the amplitude values in volts from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchPulseWidthRatio             (ScioSense_Ufc23* ufc23, UFC23_PW_Ps_TypeDef* pulseWidthsVUp, UFC23_PW_Ps_TypeDef* pulseWidthsVDn);             // Returns the pulse width ratio values from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchTofMultiHitNs               (ScioSense_Ufc23* ufc23, float* tofMultiHitPsUp, float* tofMultiHitPsDn);                                       // Returns the multi hit sum values in nanoseconds from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchTofMultiHitsCount           (ScioSense_Ufc23* ufc23, uint8_t* multiHitCountUp, uint8_t* multiHitCountDn);                                   // Returns the amount of hits from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchVddV                        (ScioSense_Ufc23* ufc23, float* vdd, float* vcc);                                                               // Returns the supply voltage values in volts from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchHsoMhz                      (ScioSense_Ufc23* ufc23, float* fHsoMhz);                                                                       // Returns the High Speed Oscillator frequencies in MHz from all the measurement bundels in the batch
static inline uint8_t                   Ufc23_ParseBatchZcLvlV                      (ScioSense_Ufc23* ufc23, float* zcLvl);                                                                         // Returns the Zero Cross Level values in volts from all the measurement bundels in the batch

#include "ScioSense_Ufc23.inl.h"
#endif // SCIOSENSE_UFC23_C_H
