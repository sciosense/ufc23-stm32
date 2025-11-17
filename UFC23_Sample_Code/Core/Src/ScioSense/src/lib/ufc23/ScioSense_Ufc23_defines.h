#ifndef SCIOSENSE_UFC23_DEFINES_C_H
#define SCIOSENSE_UFC23_DEFINES_C_H

#include <inttypes.h>

/************************** UFC23 Structs and buffers ************************/

//// UFC23 struct sizes
#define UFC23_REG_SIZE                                  uint32_t        // Size of each of the configuration registers
#define UFC23_COM_SIZE                                  uint8_t         // Size of the data package element received from the sensor
#define UFC23_REG_ADD_SIZE                              uint8_t         // Size of the addressing variable used to identify each register 
#define UFC23_BATCH_AMOUNT_SIZE                         uint8_t         // Size of the variable keeping track of how many batches are being measured. Zero means Single Cycle Mode 
#define UFC23_FR_SIZE                                   uint8_t         // Size of the Flag registers except the Frontend Error Flag register 
#define UFC23_FR_FE_SIZE                                uint16_t        // Size of the Frontend Error Flag register 
#define UFC23_TOF_RES_SIZE                              float           // Data type of the calculated ToF LSB  
#define UFC23_CYCLE_TIME_SIZE                           uint32_t        // Data type of the cycle time (in microseconds)
#define UFC23_ID_SIZE                                   uint16_t        // Data type of the device ID
#define UFC23_AMOUNT_CONFIGURATION_REGISTERS            (18)            // Addresses from CR[0] up to CR[17]
#define UFC23_AMOUNT_USM_BATCH_REGISTERS                (160)           // Amount of registers of size UFC23_REGISTER_SIZE to be read in batch mode
#define UFC23_AMOUNT_USM_BATCH_BYTES                    (640)           // Amount of Bytes to be read in batch mode
#define UFC23_AMOUNT_US_TOF_HITS_SINGLE_CYCLE           (60)            // Amount of ToF Start Hits reported in the Single Cycle Mode
#define UFC23_AMOUNT_FRONTEND_ERROR_FLAGS               (12)            // Amount of Error flags from the Frontend Error and Status register
#define UFC23_AMOUNT_AMP_MEAS                           (3)             // Amount of upstream and downstream amplitude measurements per batch
#define UFC23_AMOUNT_PW_MEAS                            (3)             // Amount of upstream and downstream pulse widths measurements per batch
#define UFC23_AMOUNT_TOF_HITS_MEAS                      (60)            // Amount of upstream and downstream ToF hits measurements per single measurement
#define UFC23_AMOUNT_REGISTERS_IN_BUNDLE                (12)            // Amount of uint32_t registers on each bundle
#define UFC23_AMOUNT_BYTES_IN_BUNDLE                    (48)            // Amount of uint8_t bytes on each bundle
#define UFC23_AMOUNT_BUNDLES_MAX                        (12)            // Maximum amount of bundles on a Batch Cycle mode
#define UFC23_AMOUNT_PART_ID_TYPES                      (4)             // Amount of Part ID identification, counting non initialized and unknown
#define UFC23_BUNDLE_CYCLE_LENGTH                       (12)            // Amount of UFC23_REG_SIZE registers that belong to each measurement in the data bundle
#define UFC23_RAM_USM_ADDRESS_START                     (0)             // Start of the RAM section dedicated to the Main Task Results (USM)
#define UFC23_RAM_USM_ADDRESS_END                       (159)           // End of the RAM section dedicated to the Main Task Results (USM)
#define UFC23_RAM_CONFIG_REGISTER_ADDRESS_START         (160)           // Start of the RAM section dedicated to the Configuration Registers
#define UFC23_RAM_CONFIG_REGISTER_ADDRESS_END           (191)           // End of the RAM section dedicated to the Configuration Registers

/******************************* UFC23 constants *****************************/

//// UFC23 electrical constants
#define UFC23_SAR_LSB_V                                 (0.001367)      // LSB value of the SAR used for measuring voltages. Float value in Volts
#define UFC23_SAR_LSB_NUMERATOR                         (14)            // Numerator of the LSB value of the SAR used for measuring voltages
#define UFC23_SAR_LSB_DENOMINATOR                       (10240)         // Denominator of the LSB value of the SAR used for measuring voltages
#define UFC23_LSO_NOMINAL_FREQUENCY_HZ                  (32678)         // Nominal frequency of the Low Speed Oscillator

//// UFC23 timing constants
#define UFC23_TOF_LSB_DENOMINATOR                       (65536.0)       // Value to divide the T_HSO to obtain the value of the ToF values LSB
#define UFC23_PW_LSB_DENOMINATOR                        (1024)          // Value to divide the T_HSO to obtain the value of the PW values LSB
#define UFC23_F_LFO_HZ                                  (32768)         // Frequency in Hertz of the Low Speed Oscillator
#define UFC23_HCC_THSO_CONSTANT_PS                      (8.0E12)        // Constant equivalent to 4 * T_LSO * 65536 for T_LSO in picoseconds
#define UFC23_HCC_FHSO_RATIO_MHZ                        (8.0E6)         // Ratio between the frequency of the High Speed Oscillator and the HCC register content 
#define UFC23_PAUSE_BETWEEN_COMMANDS_MS                 (1)             // Time to wait in milliseconds before issuing a new RC_FRU_CLR command ( >3 clock LSO )
#define UFC23_T_VDD_STBL_MS                             (10)            // Stable Time VDD (after power on of VCC)
#define UFC23_T_LSO_STUP_MS                             (200)           // Startup Time Low Speed Oscillator
#define UFC23_SWITCH_MEAS_MODE_MS                       (10)            // Time needed to switch measure mode in milliseconds
#define UFC23_T_RC_RLS_MS                               (232)           // Release time for remote communication
#define UFC23_T_MM_RLS_MS                               (1200)          // Release time for measure mode
#define UFC23_C_MCYCLE_TIME_LSB_US                      (977)           // LSB in microseconds of the Measure Cycle Time on C_MCYCLE_TIME bits of CR_MCT register
#define UFC23_MINIMUM_CYCLE_TIME_US                     (9766)          // Minimum Measure Cycle Time in microseconds
#define UFC23_HSO_FREQUENCY_CONVERSION_FACTOR_MHZ       (4000000)       // Conversion factor for C_FEP_4M_CLK_DIV register into HSO frequency Hz
#define UFC23_PW_LSB_PRESCALER                          (1024.0)        // Scaling factor for the Pulse Width LSB value
#define UFC23_TOF_LSB_PRESCALER                         (65536.0)       // Scaling factor for the Time of Flight LSB value
#define UFC23_NANOSECONDS_IN_A_SECOND                   (1000000000.0)  // Conversion from seconds to nanoseconds

//// UFC23 conversion from C_RMSET to Rdriver in ohms
#define UFC23_C_RMSET_RDRIVER_LSB_OHM                   (25)            // Value in ohms of the R_driver that the LSB represents according to C_RMSET_TX and C_RMSET_RX
#define UFC23_C_RMSET_RDRIVER_LSB_OHM_OFFSET            (1)             // Index offset for the conversion of C_RMSET_TX and C_RMSET_RX to driver Resistance in ohms
#define UFC23_C_RMSET_MAX_VALUE                         (31)            // Maximum value for C_RMSET_TX and C_RMSET_RX

//// UFC23 conversion from temperature ratio to degrees Centigrades
#define UFC23_PT_POLY_SQUARE_TERM                       (10.115)        // Square value of the inverted R(T) polynomial for PT temperature sensors (according to IEC 60751:2008)
#define UFC23_PT_POLY_LINEAR_TERM                       (235.57)        // Linear value of the inverted R(T) polynomial for PT temperature sensors (according to IEC 60751:2008)
#define UFC23_PT_POLY_CONST_TERM                        (245.683)       // Constant value of the inverted R(T) polynomial for PT temperature sensors (according to IEC 60751:2008)

/************************** UFC23 Commands definitions ***********************/

//// Remote command bit shifting
#define UFC23_COMMAND_BYTES                             (2)             // Amount of bytes that make up a command
#define UFC23_REMOTE_COMMAND_INDEX                      (4)             // Amount of bits to left shift the remote command to create the first command byte
#define UFC23_EXTENDED_COMMAND_INDEX_0                  (6)             // Amount of bits to right shift the extended command to create the first command byte
#define UFC23_EXTENDED_COMMAND_INDEX_1                  (2)             // Amount of bits to left shift the extended command to create the second command byte
#define UFC23_REMOTE_COMMAND_MASK                       (0xF0)          // Mask for the remote command on the first command byte
#define UFC23_REMOTE_EXTENDED_MASK_0                    (0x0F)          // Mask for the extended command on the first command byte
#define UFC23_REMOTE_EXTENDED_MASK_1                    (0xFC)          // Mask for the extended command on the second command byte
#define UFC23_RAM_ADDRESS_INDEX_0                       (4)             // Amount of bits to right shift the RAM Address to create the first command byte
#define UFC23_RAM_ADDRESS_INDEX_1                       (4)             // Amount of bits to left shift the RAM Address to create the second command byte
#define UFC23_RAM_ADDRESS_MASK_0                        (0x0F)          // Mask for the RAM address on the first command byte
#define UFC23_RAM_ADDRESS_MASK_1                        (0xF0)          // Mask for the RAM address on the second command byte

//// Remote commands of the UFC23, 4 bits long (named to match the datasheet). 
typedef uint8_t Ufc23_RemoteCommands;
#define UFC23_REMOTE_COMMAND_RC_RM_REQ                  (0x05)          // Reset Management Request
#define UFC23_REMOTE_COMMAND_RC_FRU_CLR                 (0x0B)          // Flag Register Clear
#define UFC23_REMOTE_COMMAND_RC_MM_CTRL                 (0x03)          // Measure Mode Control
#define UFC23_REMOTE_COMMAND_RC_STASK_REQ               (0x06)          // Service Task Request
#define UFC23_REMOTE_COMMAND_RC_CTASK_REQ               (0x07)          // Cycle Task Request
#define UFC23_REMOTE_COMMAND_RC_FRU_RD                  (0x09)          // Flag Register Read
#define UFC23_REMOTE_COMMAND_RC_RAA_WR                  (0x0D)          // Random Area Write
#define UFC23_REMOTE_COMMAND_RC_RAA_RD                  (0x0E)          // Random Area Read

//// Extended commands of the UFC23, 10 bits long (named to match the datasheet). 
typedef uint16_t Ufc23_ExtendedCommands;
//// Commands for EC_RM_REQ
#define UFC23_EXTENDED_COMMAND_EC_SYS_RST               (0x0AA)         // Resets all digital blocks
#define UFC23_EXTENDED_COMMAND_EC_SV_INIT               (0x0A9)         // Resets Supervisor block (except startup timer)
#define UFC23_EXTENDED_COMMAND_EC_CR_INIT               (0x0AC)         // Resets Configuration register
#define UFC23_EXTENDED_COMMAND_EC_FEP_INIT              (0x0B2)         // Resets Frontend Processing block (including batch counter)
#define UFC23_EXTENDED_COMMAND_EC_TCU_INIT              (0x0B7)         // Resets Test registers
//// Commands for EC_FRU_CLR
#define UFC23_EXTENDED_COMMAND_EC_CMF_CLR               (1<<0)          // Clears Communication Flag Register
#define UFC23_EXTENDED_COMMAND_EC_IF_CLR                (1<<1)          // Clears Interrupt Flag Register
#define UFC23_EXTENDED_COMMAND_EC_EF_CLR                (1<<2)          // Clears Frontend Error Flag Register
#define UFC23_EXTENDED_COMMAND_EC_FES_CLR               (1<<3)          // Clears Frontend Status Flag Register
//// Commands for EC_MM_CTRL
#define UFC23_EXTENDED_COMMAND_EC_MM_ENA_DISABLED       (0)             // Measure Mode Disabled
#define UFC23_EXTENDED_COMMAND_EC_MM_ENA_ENABLED        (1<<0)          // Measure Mode Enabled
#define UFC23_EXTENDED_COMMAND_EC_MCT_HALT_RELEASED     (0)             // Measure Cycle Timer Released 
#define UFC23_EXTENDED_COMMAND_EC_MCT_HALT_HALTED       (1<<1)          // Measure Cycle Timer Halt 
#define UFC23_EXTENDED_COMMAND_EC_MCT_CLR_RELEASED      (0)             // Measure Cycle Timer Released
#define UFC23_EXTENDED_COMMAND_EC_MCT_CLR_CLEAR         (1<<2)          // Measure Cycle Timer Clear
#define UFC23_EXTENDED_COMMAND_EC_MRG_CLR_RELEASED      (0)             // Measure Rate Generator Released
#define UFC23_EXTENDED_COMMAND_EC_MRG_CLR_CLEAR         (1<<3)          // Measure Rate Generator Clear
//// Commands for EC_STASK_REQ
#define UFC23_EXTENDED_COMMAND_EC_EHSP_REQ              (0xE2)          // Error Handling: Spool Piece
//// Commands for EC_CTASK_REQ
#define UFC23_EXTENDED_COMMAND_EC_USM_REQ               (1<<0)          // Ultrasonic Measurement Request
#define UFC23_EXTENDED_COMMAND_EC_ZCC_REQ               (1<<1)          // Zero Cross Calibration Request
#define UFC23_EXTENDED_COMMAND_EC_VCCM_REQ              (1<<2)          // VCC/VDD Measurement Request
#define UFC23_EXTENDED_COMMAND_EC_HCC_REQ               (1<<3)          // High Speed Clock Calibration Request
#define UFC23_EXTENDED_COMMAND_EC_FBC_REQ               (1<<4)          // Fire Burst Calibration Request
#define UFC23_EXTENDED_COMMAND_EC_TM_REQ                (1<<5)          // Temperature Measurement Request
//// Commands for EC_FRU_RD
#define UFC23_EXTENDED_COMMAND_RC_CMF_RD                (1<<0)          // Read Communication Flag Register
#define UFC23_EXTENDED_COMMAND_RC_IF_RD                 (1<<1)          // Read Interrupt Flag Register
#define UFC23_EXTENDED_COMMAND_RC_FES_RD                (1<<2)          // Read Frontend Error Flag Register
#define UFC23_EXTENDED_COMMAND_RC_EF_RD                 (1<<3)          // Read Frontend Status Flag Register
#define UFC23_EXTENDED_COMMAND_RC_SSF_RD                (1<<4)          // Read System Status Flag Register

//// Read command length
#define UFC23_COMMAND_RESPONSE_RC_CMF_RD_LENGTH         (1)             // Length in bytes of the Communication Flag Register
#define UFC23_COMMAND_RESPONSE_RC_IF_RD_LENGTH          (1)             // Length in bytes of the Interrupt Flag Register
#define UFC23_COMMAND_RESPONSE_RC_FES_RD_LENGTH         (2)             // Length in bytes of the Frontend Error Flag Register
#define UFC23_COMMAND_RESPONSE_RC_EF_RD_LENGTH          (1)             // Length in bytes of the Frontend Status Flag Register
#define UFC23_COMMAND_RESPONSE_RC_SSF_RD_LENGTH         (1)             // Length in bytes of the System Status Flag Register

/************************** UFC23 Register definitions ***********************/

//// Register addresses UFC23
#define UFC23_CR_FRU_IFH_ADDRESS                        (0xA0)          // Interrupt Flag Handling
#define UFC23_CR_FRU_EFH_ADDRESS                        (0xA1)          // Error Flag Handling
#define UFC23_CR_GP_CTRL_ADDRESS                        (0xA2)          // General Purpose Control
#define UFC23_CR_PM_ADDRESS                             (0xA3)          // Power Management
#define UFC23_CR_TSC_ADDRESS                            (0xA4)          // Task Sequence Control
#define UFC23_CR_MCT_ADDRESS                            (0xA5)          // Measure Cycle Timer
#define UFC23_CR_MRG_ADDRESS                            (0xA6)          // Measure Rate Generator
#define UFC23_CR_FEP_MCTRL_ADDRESS                      (0xA7)          // Front end control
#define UFC23_CR_FEP_TDC_TRIM_ADDRESS                   (0xA9)          // TDC Trimming
#define UFC23_CR_USM_PROC_ADDRESS                       (0xAA)          // Ultrasonic processing
#define UFC23_CR_USM_FBG_MCTRL_ADDRESS                  (0xAB)          // Fire burst generator control
#define UFC23_CR_USM_FBG_HRC_ADDRESS                    (0xAC)          // FBG high resolution control
#define UFC23_CR_USM_ANA_CTRL1_ADDRESS                  (0xAD)          // Analog control 1
#define UFC23_CR_USM_ANA_CTRL2_ADDRESS                  (0xAE)          // Analog control 2
#define UFC23_CR_USM_RCV_INIT_ADDRESS                   (0xAF)          // Ultrasonic receiver initialization
#define UFC23_CR_USM_HIT_CTRL_ADDRESS                   (0xB0)          // Ultrasonic hit control
#define UFC23_CR_USM_WVM_ADDRESS                        (0xB1)          // Ultrasonic wave monitor
#define UFC23_CR_USM_MASK_HR_WIN_ADDRESS                (0xB2)          // High resolution receiver mask in up direction
#define UFC23_CR_SR_DEVICE_ID_ADDRESS                   (0xFF)          // Identification of UFC23 or UFC18

//// Index of CR configuration registers array
#define UFC23_CR_FRU_IFH_INDEX                          (0)             // Index of the configuration for the CR_FRU_IFH register on the CR array
#define UFC23_CR_FRU_EFH_INDEX                          (1)             // Index of the configuration for the CR_FRU_EFH register on the CR array
#define UFC23_CR_GP_CTRL_INDEX                          (2)             // Index of the configuration for the CR_GP_CTRL register on the CR array
#define UFC23_CR_PM_INDEX                               (3)             // Index of the configuration for the CR_PM register on the CR array
#define UFC23_CR_TSC_INDEX                              (4)             // Index of the configuration for the CR_TSC register on the CR array
#define UFC23_CR_MCT_INDEX                              (5)             // Index of the configuration for the CR_MCT register on the CR array
#define UFC23_CR_MRG_INDEX                              (6)             // Index of the configuration for the CR_MRG register on the CR array
#define UFC23_CR_FEP_MCTRL_INDEX                        (7)             // Index of the configuration for the CR_FEP_MCTRL register on the CR array
#define UFC23_CR_FEP_TDC_TRIM_INDEX                     (8)             // Index of the configuration for the CR_FEP_TDC_TRIM register on the CR array
#define UFC23_CR_USM_PROC_INDEX                         (9)             // Index of the configuration for the CR_USM_PROC register on the CR array
#define UFC23_CR_USM_FBG_MCTRL_INDEX                    (10)            // Index of the configuration for the CR_USM_FBG_MCTRL register on the CR array
#define UFC23_CR_USM_FBG_HRC_INDEX                      (11)            // Index of the configuration for the CR_USM_FBG_HRC register on the CR array
#define UFC23_CR_FEP_ANA_CTRL1_INDEX                    (12)            // Index of the configuration for the CR_FEP_ANA_CTRL1 register on the CR array
#define UFC23_CR_FEP_ANA_CTRL2_INDEX                    (13)            // Index of the configuration for the CR_FEP_ANA_CTRL2 register on the CR array
#define UFC23_CR_USM_RCV_INIT_INDEX                     (14)            // Index of the configuration for the CR_USM_RCV_INIT register on the CR array
#define UFC23_CR_USM_HIT_CTRL_INDEX                     (15)            // Index of the configuration for the CR_USM_HIT_CTRL register on the CR array
#define UFC23_CR_USM_WVM_INDEX                          (16)            // Index of the configuration for the CR_USM_WVM register on the CR array
#define UFC23_CR_USM_MASK_HR_WIN_INDEX                  (17)            // Index of the configuration for the CR_USM_MASK_HR_WIN register on the CR array

//// Bit definition of CR_FRU_IFH register
#define UFC23_C_IRQ_EN_BL_DONE_Pos                      (0U)                                            // Interrupt Request Enable, Bootload Done
#define UFC23_C_IRQ_EN_BL_DONE_Msk                      (0x01UL << UFC23_C_IRQ_EN_BL_DONE_Pos)          // 0x00000001
#define UFC23_C_IRQ_EN_BL_DONE_ENABLED                  (1)                                             // Interrupt on Bootload Done
#define UFC23_C_IRQ_EN_BL_DONE_DISABLED                 (0)                                             // No Interrupt on Bootload Done
#define UFC23_C_IRQ_EN_MIS_DONE_Pos                     (1U)                                            // Interrupt Request Enable, Measure Init Done
#define UFC23_C_IRQ_EN_MIS_DONE_Msk                     (0x01UL << UFC23_C_IRQ_EN_MIS_DONE_Pos)         // 0x00000002
#define UFC23_C_IRQ_EN_MIS_DONE_ENABLED                 (1)                                             // Interrupt on Measure Init Done
#define UFC23_C_IRQ_EN_MIS_DONE_DISABLED                (0)                                             // No Interrupt on Measure Init Done
#define UFC23_C_IRQ_EN_MCS_DONE_Pos                     (2U)                                            // Interrupt Request Enable, Measure Cycle Done
#define UFC23_C_IRQ_EN_MCS_DONE_Msk                     (0x01UL << UFC23_C_IRQ_EN_MCS_DONE_Pos)         // 0x00000004
#define UFC23_C_IRQ_EN_MCS_DONE_ENABLED                 (1)                                             // Interrupt on Measure Cycle Done
#define UFC23_C_IRQ_EN_MCS_DONE_DISABLED                (0)                                             // No Interrupt on Measure Cycle Done
#define UFC23_C_IRQ_EN_MC_BATCH_DONE_Pos                (3U)                                            // Interrupt Request Enable, Batch of Measure Cycle Done
#define UFC23_C_IRQ_EN_MC_BATCH_DONE_Msk                (0x01UL << UFC23_C_IRQ_EN_MC_BATCH_DONE_Pos)    // 0x00000008
#define UFC23_C_IRQ_EN_MC_BATCH_DONE_ENABLED            (1)                                             // Interrupt on Batch of Measure Cycle Done
#define UFC23_C_IRQ_EN_MC_BATCH_DONE_DISABLED           (0)                                             // No Interrupt on Batch of Measure Cycle Done
#define UFC23_C_IRQ_EN_STASK_DONE_Pos                   (4U)                                            // Interrupt Request Enable, Service Task Done
#define UFC23_C_IRQ_EN_STASK_DONE_Msk                   (0x01UL << UFC23_C_IRQ_EN_STASK_DONE_Pos)       // 0x00000010
#define UFC23_C_IRQ_EN_STASK_DONE_ENABLED               (1)                                             // Interrupt on Service Task Done
#define UFC23_C_IRQ_EN_STASK_DONE_DISABLED              (0)                                             // No Interrupt on Service Task Done
#define UFC23_C_IRQ_EN_USM_PAUSE_ERR_Pos                (5U)                                            // Interrupt Request Enable, Ultrasonic Measurement Pause Error
#define UFC23_C_IRQ_EN_USM_PAUSE_ERR_Msk                (0x01UL << UFC23_C_IRQ_EN_USM_PAUSE_ERR_Pos)    // 0x00000020
#define UFC23_C_IRQ_EN_USM_PAUSE_ERR_ENABLED            (1)                                             // Interrupt on Ultrasonic Measurement Pause Error
#define UFC23_C_IRQ_EN_USM_PAUSE_ERR_DISABLED           (0)                                             // No Interrupt on Ultrasonic Measurement Pause Error
#define UFC23_C_IRQ_EN_TSC_TMO_Pos                      (6U)                                            // Interrupt Request Enable, Task Sequencer Timeout
#define UFC23_C_IRQ_EN_TSC_TMO_Msk                      (0x01UL << UFC23_C_IRQ_EN_TSC_TMO_Pos)          // 0x00000040
#define UFC23_C_IRQ_EN_TSC_TMO_ENABLED                  (1)                                             // Interrupt on Task Sequencer Timeout
#define UFC23_C_IRQ_EN_TSC_TMO_DISABLED                 (0)                                             // No Interrupt on Task Sequencer Timeout

//// Bit definition of CR_FRU_EFH register
#define UFC23_C_EF_EN_HCC_TDC_TMO_Pos                   (0U)                                            // Error Flag Enable, HCC TDC Timeout
#define UFC23_C_EF_EN_HCC_TDC_TMO_Msk                   (0x01UL << UFC23_C_EF_EN_HCC_TDC_TMO_Pos)       // 0x00000001
#define UFC23_C_EF_EN_TM_TDC_TMO_Pos                    (1U)                                            // Error Flag Enable, TM TDC Timeout
#define UFC23_C_EF_EN_TM_TDC_TMO_Msk                    (0x01UL << UFC23_C_EF_EN_TM_TDC_TMO_Pos)        // 0x00000002
#define UFC23_C_EF_EN_TM_OC_ERR_Pos                     (2U)                                            // Error Flag Enable, TM Open Circuit Error
#define UFC23_C_EF_EN_TM_OC_ERR_Msk                     (0x01UL << UFC23_C_EF_EN_TM_OC_ERR_Pos)         // 0x00000004
#define UFC23_C_EF_EN_TM_SC_ERR_Pos                     (3U)                                            // Error Flag Enable, TM Short Circuit Error
#define UFC23_C_EF_EN_TM_SC_ERR_Msk                     (0x01UL << UFC23_C_EF_EN_TM_SC_ERR_Pos)         // 0x00000008
#define UFC23_C_EF_EN_USM_HW_UP_ERR_Pos                 (4U)                                            // Error Flag Enable, USM HW Error Up
#define UFC23_C_EF_EN_USM_HW_UP_ERR_Msk                 (0x01UL << UFC23_C_EF_EN_USM_HW_UP_ERR_Pos)     // 0x00000010
#define UFC23_C_EF_EN_USM_HW_DN_ERR_Pos                 (5U)                                            // Error Flag Enable, USM HW Error Dn
#define UFC23_C_EF_EN_USM_HW_DN_ERR_Msk                 (0x01UL << UFC23_C_EF_EN_USM_HW_DN_ERR_Pos)     // 0x00000020
#define UFC23_C_EF_EN_USM_HW_UP_ERR_DISABLED            (0)                                             // Clear hardware error flags
#define UFC23_C_EF_EN_USM_HW_UP_ERR_ENABLED             (1)                                             // Set hardware error flags
#define UFC23_C_EF_EN_PW_UP_TDC_TMO_Pos                 (6U)                                            // Error Flag Enable, PW TDC  Timeout Up
#define UFC23_C_EF_EN_PW_UP_TDC_TMO_Msk                 (0x01UL << UFC23_C_EF_EN_PW_UP_TDC_TMO_Pos)     // 0x00000040
#define UFC23_C_EF_EN_PW_DN_TDC_TMO_Pos                 (7U)                                            // Error Flag Enable, PW TDC  Timeout Up
#define UFC23_C_EF_EN_PW_DN_TDC_TMO_Msk                 (0x01UL << UFC23_C_EF_EN_PW_DN_TDC_TMO_Pos)     // 0x00000080
#define UFC23_C_EF_EN_TOF_UP_TDC_TMO_Pos                (8U)                                            // Error Flag Enable, TOF TDC  Timeout Up
#define UFC23_C_EF_EN_TOF_UP_TDC_TMO_Msk                (0x01UL << UFC23_C_EF_EN_TOF_UP_TDC_TMO_Pos)    // 0x00000100
#define UFC23_C_EF_EN_TOF_DN_TDC_TMO_Pos                (9U)                                            // Error Flag Enable, TOF TDC  Timeout dn
#define UFC23_C_EF_EN_TOF_DN_TDC_TMO_Msk                (0x01UL << UFC23_C_EF_EN_TOF_DN_TDC_TMO_Pos)    // 0x00000200
#define UFC23_C_EF_EN_USM_UP_TMO_Pos                    (10U)                                           // Error Flag Enable, USM Timeout Up
#define UFC23_C_EF_EN_USM_UP_TMO_Msk                    (0x01UL << UFC23_C_EF_EN_USM_UP_TMO_Pos)        // 0x00000400
#define UFC23_C_EF_EN_USM_DN_TMO_Pos                    (11U)                                           // Error Flag Enable, USM Timeout Up
#define UFC23_C_EF_EN_USM_DN_TMO_Msk                    (0x01UL << UFC23_C_EF_EN_USM_DN_TMO_Pos)        // 0x00000800

//// Bit definition of CR_GP_CTRL register
#define UFC23_C_GPIO0_MODE_Pos                          (0U)                                            // Mode of GPIO port 0
#define UFC23_C_GPIO0_MODE_Msk                          (0x07UL << UFC23_C_GPIO0_MODE_Pos)              // 0x00000007
#define UFC23_C_GPIO1_MODE_Pos                          (3U)                                            // Mode of GPIO port 1
#define UFC23_C_GPIO1_MODE_Msk                          (0x07UL << UFC23_C_GPIO1_MODE_Pos)              // 0x00000038
#define UFC23_C_GPIO2_MODE_Pos                          (6U)                                            // Mode of GPIO port 2
#define UFC23_C_GPIO2_MODE_Msk                          (0x07UL << UFC23_C_GPIO2_MODE_Pos)              // 0x000001C0
#define UFC23_C_GPIO3_MODE_Pos                          (9U)                                            // Mode of GPIO port 3
#define UFC23_C_GPIO3_MODE_Msk                          (0x07UL << UFC23_C_GPIO3_MODE_Pos)              // 0x00000E00
#define UFC23_C_GPIO_MODE_DI                            (7)
#define UFC23_C_GPIO_MODE_DI_PD                         (3)
#define UFC23_C_GPIO_MODE_DO                            (4)
#define UFC23_C_GPIO_MODE_DO_PD                         (0)
#define UFC23_C_GPIO_MODE_AIO                           (5)
#define UFC23_C_GPIO_MODE_AIO_PD                        (1)
#define UFC23_C_PRB_SEL_Pos                             (12U)                                           // Reserved
#define UFC23_C_PRB_SEL_Msk                             (0x0FUL << UFC23_C_PRB_SEL_Pos)                 // 0x0000F000
#define UFC23_C_TST_STM_Pos                             (16U)                                           // Reserved
#define UFC23_C_TST_STM_Msk                             (0x1FUL << UFC23_C_TST_STM_Pos)                 // 0x001F0000
#define UFC23_C_MISO_HZ_DIS_Pos                         (21U)                                           // Remove HZ option for SPI MISO when not used
#define UFC23_C_MISO_HZ_DIS_Msk                         (0x01UL << UFC23_C_MISO_HZ_DIS_Pos)             // 0x00200000
#define UFC23_C_MISO_HZ_DIS_HZ                          (0)                                             // MISO is HZ when not used for read data
#define UFC23_C_MISO_HZ_DIS_LOW                         (1)                                             // MISO is low when not used for read data

//// Bit definition of CR_PM register
#define UFC23_C_LDO_RF_RATE_Pos                         (0U)                                            // 
#define UFC23_C_LDO_RF_RATE_Msk                         (0x07UL << UFC23_C_LDO_RF_RATE_Pos)             // 0x00000007
#define UFC23_C_LDO_RF_RATE_64HZ                        (0)                                             // LDO refresh rate state 64 Hz
#define UFC23_C_LDO_RF_RATE_32HZ                        (1)                                             // LDO refresh rate state 32 Hz
#define UFC23_C_LDO_RF_RATE_16HZ                        (2)                                             // LDO refresh rate state 16 Hz
#define UFC23_C_LDO_RF_RATE_8HZ                         (3)                                             // LDO refresh rate state 8 Hz
#define UFC23_C_LDO_RF_RATE_4HZ                         (4)                                             // LDO refresh rate state 4 Hz
#define UFC23_C_LDO_RF_RATE_2HZ                         (5)                                             // LDO refresh rate state 2 Hz
#define UFC23_C_LDO_RF_RATE_1HZ                         (6)                                             // LDO refresh rate state 1 Hz
#define UFC23_C_LDO_RF_RATE_05HZ                        (7)                                             // LDO refresh rate state 0.5 Hz
#define UFC23_C_VDD18_SW_MODE_Pos                       (4U)                                            // VDD18 Switch Mode
#define UFC23_C_VDD18_SW_MODE_Msk                       (0x01UL << UFC23_C_VDD18_SW_MODE_Pos)           // 0x00000010
#define UFC23_C_VDD18_SW_MODE_TASK_SEQUENCER            (0)                                             // VDD_SW controlled by Task Sequencer (default)
#define UFC23_C_VDD18_SW_MODE_ALWAYS_ENABLED            (1)                                             // VDD_SW always enabled

//// Bit definition of CR_TSC register
#define UFC23_C_USM_PAUSE_TSEL_Pos                      (0U)                                            // Pause time between 2 US measurements up-dn (multiple of LS period)
#define UFC23_C_USM_PAUSE_TSEL_Msk                      (0x07UL << UFC23_C_USM_PAUSE_TSEL_Pos)          // 0x00000007
#define UFC23_C_USM_PAUSE_TSEL_05MS                     (0)                                             // Pause time 0.5 ms
#define UFC23_C_USM_PAUSE_TSEL_1MS                      (1)                                             // Pause time 1 ms
#define UFC23_C_USM_PAUSE_TSEL_2MS                      (2)                                             // Pause time 2 ms
#define UFC23_C_USM_PAUSE_TSEL_4MS                      (3)                                             // Pause time 4 ms
#define UFC23_C_USM_PAUSE_TSEL_8MS                      (4)                                             // Pause time 8 ms
#define UFC23_C_USM_PAUSE_TSEL_10MS                     (5)                                             // Pause time 10 ms
#define UFC23_C_USM_PAUSE_TSEL_16MS                     (6)                                             // Pause time 16.66 ms
#define UFC23_C_USM_PAUSE_TSEL_20MS                     (7)                                             // Pause time 20 ms
#define UFC23_C_USM_REPEAT_Pos                          (3U)                                            // Ultrasonic Measurement Repeat Mode
#define UFC23_C_USM_REPEAT_Msk                          (0x01UL << UFC23_C_USM_REPEAT_Pos)              // 0x00000008
#define UFC23_C_USM_REPEAT_1_US_MEASUREMENT             (0)                                             // Only 1 ultrasonic measurement performed 
#define UFC23_C_USM_REPEAT_2_US_MEASUREMENT             (1)                                             // 2 US measurements performed in different directions, separated by pause time
#define UFC23_C_USM_DIR_MODE_Pos                        (4U)                                            // Ultrasonic Measurement Direction Mode
#define UFC23_C_USM_DIR_MODE_Msk                        (0x03UL << UFC23_C_USM_DIR_MODE_Pos)            // 0x00000030
#define UFC23_C_USM_DIR_MODE_START_UP                   (0)                                             // Always starting firing via Fire Buffer Up
#define UFC23_C_USM_DIR_MODE_START_DN                   (1)                                             // Always starting firing via Fire Buffer Down
#define UFC23_C_USM_DIR_MODE_TOGGLE                     (2)                                             // Toggling direction with every US measurement
#define UFC23_C_USM_EDGE_MODE_Pos                       (6U)                                            // USM TOF Measurement, Edge Mode
#define UFC23_C_USM_EDGE_MODE_Msk                       (0x03UL << UFC23_C_USM_EDGE_MODE_Pos)           // 0x000000C0
#define UFC23_C_USM_EDGE_MODE_POS_EDGE                  (0)                                             // TOF measurement on positive edge of TOF Hit
#define UFC23_C_USM_EDGE_MODE_NEG_EDGE                  (1)                                             // TOF measurement on negative edge of TOF Hit
#define UFC23_C_USM_EDGE_MODE_TOGGLE_EVERY_1            (2)                                             // Edge for TOF measurement toggling after every measurement cycle
#define UFC23_C_USM_EDGE_MODE_TOGGLE_EVERY_2            (3)                                             // Edge for TOF measurement toggling after every 2 measurement cycles
#define UFC23_C_LDO_STUP_TSEL_Pos                       (8U)                                            // LDO Startup Timing Select (multiple of LS period)
#define UFC23_C_LDO_STUP_TSEL_Msk                       (0x03UL << UFC23_C_LDO_STUP_TSEL_Pos)           // 0x00000300
#define UFC23_C_LDO_STUP_TSEL_025MS                     (0)                                             // 0.25 ms
#define UFC23_C_LDO_STUP_TSEL_05MS                      (1)                                             // 0.5 ms
#define UFC23_C_LDO_STUP_TSEL_1MS                       (2)                                             // 1 ms
#define UFC23_C_LDO_STUP_TSEL_2MS                       (3)                                             // 2 ms
#define UFC23_C_LDO_FEP_MODE_Pos                        (10U)                                           // LDO request by task sequencer during frontend processing (by signal LDO_REQ_TSC)
#define UFC23_C_LDO_FEP_MODE_Msk                        (0x01UL << UFC23_C_LDO_FEP_MODE_Pos)            // 0x00000400
#define UFC23_C_LDO_FEP_MODE_NOT_REQUESTED              (0)                                             // LDO not requested during frontend processing
#define UFC23_C_LDO_FEP_MODE_REQUESTED                  (1)                                             // LDO requested during frontend processing
#define UFC23_C_TM_SQC_MODE_Pos                         (11U)                                           // Temperature Measurement Sequence Mode
#define UFC23_C_TM_SQC_MODE_Msk                         (0x03UL << UFC23_C_TM_SQC_MODE_Pos)             // 0x00001800
#define UFC23_C_TM_SQC_MODE_DEFAULT_ORDER               (0)                                             // Always starting in default order
#define UFC23_C_TM_SQC_MODE_REVERSED_ORDER              (1)                                             // Always starting reversed order
#define UFC23_C_TM_SQC_MODE_TOGGLE_ORDER                (2)                                             // Toggling order with every TM measurement

//// Bit definition of CR_MCT register
#define UFC23_C_MCYCLE_TIME_Pos                         (0U)                                            // Measure Cycle Trigger timing
#define UFC23_C_MCYCLE_TIME_Msk                         (0xFFFUL << UFC23_C_MCYCLE_TIME_Pos)            // 0x00000FFF
#define UFC23_C_MCYCLE_TAIL_SEL_Pos                     (12U)                                           // Measure Cycle, Tail Time Select
#define UFC23_C_MCYCLE_TAIL_SEL_Msk                     (0x07UL << UFC23_C_MCYCLE_TAIL_SEL_Pos)         // 0x00007000
#define UFC23_C_MCYCLE_TAIL_SEL_488US                   (0)                                             // Measure Cycle, Tail Time 0.488 ms
#define UFC23_C_MCYCLE_TAIL_SEL_977US                   (1)                                             // Measure Cycle, Tail Time 0.977 ms
#define UFC23_C_MCYCLE_TAIL_SEL_1465US                  (2)                                             // Measure Cycle, Tail Time 1.465 ms
#define UFC23_C_MCYCLE_TAIL_SEL_1953US                  (3)                                             // Measure Cycle, Tail Time 1.953 ms
#define UFC23_C_MCYCLE_TAIL_SEL_2930US                  (4)                                             // Measure Cycle, Tail Time 2.930 ms
#define UFC23_C_MCYCLE_TAIL_SEL_3910US                  (5)                                             // Measure Cycle, Tail Time 3.910 ms
#define UFC23_C_MCYCLE_TAIL_SEL_5859US                  (6)                                             // Measure Cycle, Tail Time 5.859 ms
#define UFC23_C_MCYCLE_TAIL_SEL_7813US                  (7)                                             // Measure Cycle, Tail Time 7.813 ms
#define UFC23_C_MCT_EN_Pos                              (15U)                                           // Measure Cycle Timer Enable
#define UFC23_C_MCT_EN_Msk                              (0x01UL << UFC23_C_MCT_EN_Pos)                  // 0x00008000
#define UFC23_C_MCT_EN_DISABLED                         (0)                                             // Measure Cycle Timer Disabled
#define UFC23_C_MCT_EN_ENABLED                          (1)                                             // Measure Cycle Timer Enabled

//// Bit definition of CR_MRG register
#define UFC23_C_USM_RATE_Pos                            (0U)                                            // Ultrasonic Measurement Rate
#define UFC23_C_USM_RATE_Msk                            (0x07UL << UFC23_C_USM_RATE_Pos)                // 0x00000007
#define UFC23_C_USM_RATE_NOT_TRIGGERED                  (0)                                             // US measurement not triggered by MCT
#define UFC23_C_USM_RATE_TRIGGER_EVERY_1                (1)                                             // US measurement every measure cycle trigger
#define UFC23_C_USM_RATE_TRIGGER_EVERY_2                (2)                                             // US measurement every 2 measure cycle triggers
#define UFC23_C_USM_RATE_TRIGGER_EVERY_4                (3)                                             // US measurement every 4 measure cycle triggers
#define UFC23_C_USM_RATE_TRIGGER_EVERY_8                (4)                                             // US measurement every 8 measure cycle triggers
#define UFC23_C_ZCC_RATE_Pos                            (3U)                                            // Zero Cross Calibration Rate
#define UFC23_C_ZCC_RATE_Msk                            (0x07UL << UFC23_C_ZCC_RATE_Pos)                // 0x00000038
#define UFC23_C_ZCC_RATE_NOT_TRIGGERED                  (0)                                             // Zero cross calibration not triggered by MCT
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_1                (1)                                             // Zero cross calibration every measure cycle trigger
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_2                (2)                                             // Zero cross calibration every 2 measure cycle triggers
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_4                (3)                                             // Zero cross calibration every 4 measure cycle triggers
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_8                (4)                                             // Zero cross calibration every 8 measure cycle triggers
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_16               (5)                                             // Zero cross calibration every 16 measure cycle triggers
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_32               (6)                                             // Zero cross calibration every 32 measure cycle triggers
#define UFC23_C_ZCC_RATE_TRIGGER_EVERY_64               (7)                                             // Zero cross calibration every 64 measure cycle triggers
#define UFC23_C_VCCM_RATE_Pos                           (6U)                                            // VCC Measurement Rate
#define UFC23_C_VCCM_RATE_Msk                           (0x07UL << UFC23_C_VCCM_RATE_Pos)               // 0x000001C0
#define UFC23_C_VCCM_RATE_NOT_TRIGGERED                 (0)                                             // VCC Measurement not triggered by MCT
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_1               (1)                                             // VCC Measurement every measure cycle trigger
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_2               (2)                                             // VCC Measurement every 2 measure cycle triggers
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_4               (3)                                             // VCC Measurement every 4 measure cycle triggers
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_8               (4)                                             // VCC Measurement every 8 measure cycle triggers
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_16              (5)                                             // VCC Measurement every 16 measure cycle triggers
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_32              (6)                                             // VCC Measurement every 32 measure cycle triggers
#define UFC23_C_VCCM_RATE_TRIGGER_EVERY_64              (7)                                             // VCC Measurement every 64 measure cycle triggers
#define UFC23_C_HCC_RATE_Pos                            (9U)                                            // High-Speed Clock Calibration Rate
#define UFC23_C_HCC_RATE_Msk                            (0x07UL << UFC23_C_HCC_RATE_Pos)                // 0x00000E00
#define UFC23_C_HCC_RATE_NOT_TRIGGERED                  (0)                                             // High-Speed Clock Calibration not triggered by MCT
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_1                (1)                                             // High-Speed Clock Calibration every measure cycle trigger
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_2                (2)                                             // High-Speed Clock Calibration every 2 measure cycle triggers
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_4                (3)                                             // High-Speed Clock Calibration every 4 measure cycle triggers
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_8                (4)                                             // High-Speed Clock Calibration every 8 measure cycle triggers
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_16               (5)                                             // High-Speed Clock Calibration every 16 measure cycle triggers
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_32               (6)                                             // High-Speed Clock Calibration every 32 measure cycle triggers
#define UFC23_C_HCC_RATE_TRIGGER_EVERY_64               (7)                                             // High-Speed Clock Calibration every 64 measure cycle triggers
#define UFC23_C_FBC_RATE_Pos                            (12U)                                           // Fire Burst Calibration Rate
#define UFC23_C_FBC_RATE_Msk                            (0x07UL << UFC23_C_FBC_RATE_Pos)                // 0x00007000
#define UFC23_C_FBC_RATE_NOT_TRIGGERED                  (0)                                             // Fire Burst Calibration not triggered by MCT
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_1                (1)                                             // Fire Burst Calibration every measure cycle trigger
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_2                (2)                                             // Fire Burst Calibration every 2 measure cycle triggers
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_4                (3)                                             // Fire Burst Calibration every 4 measure cycle triggers
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_8                (4)                                             // Fire Burst Calibration every 8 measure cycle triggers
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_16               (5)                                             // Fire Burst Calibration every 16 measure cycle triggers
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_32               (6)                                             // Fire Burst Calibration every 32 measure cycle triggers
#define UFC23_C_FBC_RATE_TRIGGER_EVERY_64               (7)                                             // Fire Burst Calibration every 64 measure cycle triggers
#define UFC23_C_TM_RATE_Pos                             (16U)                                           // Temperature Measurement Rate
#define UFC23_C_TM_RATE_Msk                             (0x3FFUL << UFC23_C_TM_RATE_Pos)                // 0x03FF0000
#define UFC23_C_TM_RATE_DISABLED                        (0)                                             // Temperature measurement disabled

//// Bit definition of CR_FEP_MCTRL register
#define UFC23_C_FEP_4M_CLK_DIV_Pos                      (0U)                                            // 
#define UFC23_C_FEP_4M_CLK_DIV_Msk                      (0x07UL << UFC23_C_FEP_4M_CLK_DIV_Pos)          // 0x00000007
#define UFC23_C_FEP_4M_CLK_DIV_NOT_CALCULATED           (0)                                             // Not calculated
#define UFC23_C_FEP_4M_CLK_DIV_4MHZ                     (1)                                             // 4 MHz
#define UFC23_C_FEP_4M_CLK_DIV_8MHZ                     (2)                                             // 8 MHz
#define UFC23_C_FEP_4M_CLK_DIV_12MHZ                    (3)                                             // 12 MHz
#define UFC23_C_FEP_4M_CLK_DIV_16MHZ                    (4)                                             // 16 MHz
#define UFC23_C_FEP_4M_CLK_DIV_20MHZ                    (5)                                             // 20 MHz
#define UFC23_C_ADC_ST_Pos                              (3U)                                            // ADC Step Width
#define UFC23_C_ADC_ST_Msk                              (0x01UL << UFC23_C_ADC_ST_Pos)                  // 0x00000008
#define UFC23_C_ADC_ST_1US                              (0)                                             // ADC Step Width 1.0 us
#define UFC23_C_ADC_ST_2US                              (1)                                             // ADC Step Width 2.0 us
#define UFC23_C_FEP_STUP_TSEL_Pos                       (4U)                                            // Time until any FEP measurement gets released (HSO settling time)
#define UFC23_C_FEP_STUP_TSEL_Msk                       (0x0FUL << UFC23_C_FEP_STUP_TSEL_Pos)           // 0x000000F0
#define UFC23_C_FEP_STUP_TSEL_200US                     (0xC)                                           // HSO settling time 200 us
#define UFC23_C_FEP_STUP_TSEL_250US                     (0xD)                                           // HSO settling time 250 us
#define UFC23_C_FEP_STUP_TSEL_350US                     (0xE)                                           // HSO settling time 350 us
#define UFC23_C_FEP_STUP_TSEL_500US                     (0xF)                                           // HSO settling time 500 us
#define UFC23_C_USM_TMO_SEL_Pos                         (8U)                                            // Timeout Ultrasonic Measurement
#define UFC23_C_USM_TMO_SEL_Msk                         (0x0FUL << UFC23_C_USM_TMO_SEL_Pos)             // 0x00000F00
#define UFC23_C_TM_PORT_NO_Pos                          (12U)                                           // Temperature Measurement Port Number
#define UFC23_C_TM_PORT_NO_Msk                          (0x01UL << UFC23_C_TM_PORT_NO_Pos)              // 0x00001000
#define UFC23_C_TM_PORT_NO_1_PORT                       (0)                                             // Temperature Measurement 1 Port
#define UFC23_C_TM_PORT_NO_2_PORTS                      (1)                                             // Temperature Measurement 2 Ports
#define UFC23_C_TM_PORT_MODE_Pos                        (13U)                                           // Temperature Measurement Port Mode
#define UFC23_C_TM_PORT_MODE_Msk                        (0x01UL << UFC23_C_TM_PORT_MODE_Pos)            // 0x00002000
#define UFC23_C_TM_PORT_MODE_INACTIVE_TO_GND            (0)                                             // Inactive ports pulled to GND during measurement
#define UFC23_C_TM_PORT_MODE_INACTIVE_TO_HIGH_Z         (1)                                             // Inactive ports pulled to HighZ during measurement
#define UFC23_C_TM_CYCLE_SEL_Pos                        (14U)                                           // Temperature Measurement Cycle Select
#define UFC23_C_TM_CYCLE_SEL_Msk                        (0x03UL << UFC23_C_TM_CYCLE_SEL_Pos)            // 0x0000C000
#define UFC23_C_TM_CYCLE_SEL_CYCLE_256US                (0)                                             // Temperature Measurement Cycle 256 us (speed up option 1)
#define UFC23_C_TM_CYCLE_SEL_CYCLE_384US                (1)                                             // Temperature Measurement Cycle 384 us (speed up option 2)
#define UFC23_C_TM_CYCLE_SEL_CYCLE_512US                (2)                                             // Temperature Measurement Cycle 512 us (default, recommended for typical application)
#define UFC23_C_TM_CYCLE_SEL_CYCLE_1024US               (3)                                             // Temperature Measurement Cycle 1024 us (slow down option)
#define UFC23_C_TM_SQC_NO_Pos                           (16U)                                           // Temperature Measurement Sequence Number
#define UFC23_C_TM_SQC_NO_Msk                           (0x01UL << UFC23_C_TM_SQC_NO_Pos)               // 0x00010000
#define UFC23_C_TM_SQC_NO_1_SEQUENCE                    (0)                                             // Temperature Measurement 1 sequence
#define UFC23_C_TM_SQC_NO_2_SEQUENCES                   (1)                                             // Temperature Measurement 2 sequences in reversed order
#define UFC23_C_HF_CLB_SEL_Pos                          (17U)                                           // Period cycles for HF oscillator calibration
#define UFC23_C_HF_CLB_SEL_Msk                          (0x03UL << UFC23_C_HF_CLB_SEL_Pos)              // 0x00060000
#define UFC23_C_HF_CLB_SEL_4_CYCLES                     (0)                                             // HF oscillator calibration 4 cycles
#define UFC23_C_HF_CLB_SEL_2_CYCLES                     (1)                                             // HF oscillator calibration 2 cycles
#define UFC23_C_HF_CLB_SEL_1_CYCLE                      (2)                                             // HF oscillator calibration 1 cycle
#define UFC23_C_HF_SERIAL_Pos                           (19U)                                           // HF DCO calibration
#define UFC23_C_HF_SERIAL_Msk                           (0x01UL << UFC23_C_HF_SERIAL_Pos)               // 0x00080000
#define UFC23_C_HF_SERIAL_PARALLEL                      (0)                                             // HF DCO calibration done in parallel to FBC
#define UFC23_C_HF_SERIAL_BEFORE                        (1)                                             // HF DCO calibration done before FBC
#define UFC23_C_HF_TRIM_Pos                             (20U)                                           // DCO trimming value to be applied in case
#define UFC23_C_HF_TRIM_Msk                             (0x3FFUL << UFC23_C_HF_TRIM_Pos)                // 0x3FF00000
#define UFC23_C_HF_CALIB_MODE_Pos                       (30U)                                           // Trimming Select
#define UFC23_C_HF_CALIB_MODE_Msk                       (0x01UL << UFC23_C_HF_CALIB_MODE_Pos)           // 0x40000000
#define UFC23_C_HF_CALIB_MODE_AUTO_TRIMMING             (0)                                             // Auto trimming during calibration
#define UFC23_C_HF_CALIB_MODE_CONSTANT_VALUE            (1)                                             // Constant trimming value (C_HF_TRIM)
#define UFC23_C_HF_DCO_ENA_Pos                          (31U)                                           // Enable internal HF (external not used) 
#define UFC23_C_HF_DCO_ENA_Msk                          (0x01UL << UFC23_C_HF_DCO_ENA_Pos)              // 0x80000000
#define UFC23_C_HF_DCO_ENA_DISABLE                      (0)                                             // Enable internal HF (external not used) 
#define UFC23_C_HF_DCO_ENA_ENABLE                       (1)                                             // Disable internal HF

//// Bit definition of CR_FEP_TDC_TRIM register
#define UFC23_C_TDC_SEL_QHA1_Pos                        (0U)                                            // TDC Path Trim (SELQHA1)
#define UFC23_C_TDC_SEL_QHA1_Msk                        (0x3FUL << UFC23_C_TDC_SEL_QHA1_Pos)            // 0x0000003F
#define UFC23_C_TDC_SEL_QHA2_Pos                        (6U)                                            // TDC Path Trim (SELQHA2)
#define UFC23_C_TDC_SEL_QHA2_Msk                        (0x3FUL << UFC23_C_TDC_SEL_QHA2_Pos)            // 0x00000FC0
#define UFC23_C_TDC_PHS_MODE_Pos                        (12U)                                           // TDC Phase Shift Mode
#define UFC23_C_TDC_PHS_MODE_Msk                        (0x01UL << UFC23_C_TDC_PHS_MODE_Pos)            // 0x00001000
#define UFC23_C_TDC_PHS_MODE_DISABLED                   (0)                                             // Delay line not used
#define UFC23_C_TDC_PHS_MODE_ENABLED                    (1)                                             // Delay line connected to TDC_START
#define UFC23_C_PHS_CELLS_Pos                           (13U)                                           // Delay line number of cells 
#define UFC23_C_PHS_CELLS_Msk                           (0x03UL << UFC23_C_PHS_CELLS_Pos)               // 0x00006000
#define UFC23_C_PHS_CELLS_8                             (0)                                             // Delay line 8 cells 
#define UFC23_C_PHS_CELLS_16                            (1)                                             // Delay line 16 cells 
#define UFC23_C_PHS_CELLS_32                            (2)                                             // Delay line 32 cells 
#define UFC23_C_PHS_CELLS_64                            (3)                                             // Delay line 64 cells (only valid if C_PHS_INCR=0)
#define UFC23_C_PHS_INCR_Pos                            (15U)                                           // Delay line counter increment
#define UFC23_C_PHS_INCR_Msk                            (0x01UL << UFC23_C_PHS_INCR_Pos)                // 0x00008000
#define UFC23_C_PHS_INCR_PLUS_1                         (0)                                             // Delay line counter increment +1 at every measure cycle (up+dn)
#define UFC23_C_PHS_INCR_PLUS_2                         (1)                                             // Delay line counter increment +2 at every measure cycle (up+dn)
#define UFC23_C_TDC_HR_ADJUST_Pos                       (16U)                                           // High Resolution Adjust
#define UFC23_C_TDC_HR_ADJUST_Msk                       (0x7FFUL << UFC23_C_TDC_HR_ADJUST_Pos)          // 0x07FF0000

//// Bit definition of CR_USM_PROC register
#define UFC23_C_USM_MASK_WIN_Pos                        (0U)                                            // Mask window before ultrasonic receive burst gets released
#define UFC23_C_USM_MASK_WIN_Msk                        (0x1FFFUL << UFC23_C_USM_MASK_WIN_Pos)          // 0x00001FFF
#define UFC23_C_USM_MHIT_BATCH_Pos                      (13U)                                           // Number US Multi Hits per batch
#define UFC23_C_USM_MHIT_BATCH_Msk                      (0x07UL << UFC23_C_USM_MHIT_BATCH_Pos)          // 0x0000E000
#define UFC23_C_USM_MHIT_BATCH_1_USM_BUNDLE             (0)                                             // Single Cycle Mode: 1 USM bundle
#define UFC23_C_USM_MHIT_BATCH_2_USM_BUNDLES            (1)                                             // Batch Cycle Mode: 2 USM bundles
#define UFC23_C_USM_MHIT_BATCH_4_USM_BUNDLES            (2)                                             // Batch Cycle Mode: 4 USM bundles
#define UFC23_C_USM_MHIT_BATCH_6_USM_BUNDLES            (3)                                             // Batch Cycle Mode: 6 USM bundles
#define UFC23_C_USM_MHIT_BATCH_8_USM_BUNDLES            (4)                                             // Batch Cycle Mode: 8 USM bundles
#define UFC23_C_USM_MHIT_BATCH_10_USM_BUNDLES           (5)                                             // Batch Cycle Mode: 10 USM bundles
#define UFC23_C_USM_MHIT_BATCH_12_USM_BUNDLES           (6)                                             // Batch Cycle Mode: 12 USM bundles
#define UFC23_C_USM_SENSOR_MODE_Pos                     (16U)                                           // Ultrasonic Sensor Mode
#define UFC23_C_USM_SENSOR_MODE_Msk                     (0x01UL << UFC23_C_USM_SENSOR_MODE_Pos)         // 0x00010000
#define UFC23_C_USM_SENSOR_MODE_1_SENSOR                (0)                                             // 1 Sensor only (sending & receiving on same transducer)
#define UFC23_C_USM_SENSOR_MODE_2_SENSORS               (1)                                             // 2 Sensor (sending & receiving by 2 opposite transducer)
#define UFC23_C_USM_AM_MODE_Pos                         (17U)                                           // Amplitude Measurement Mode
#define UFC23_C_USM_AM_MODE_Msk                         (0x01UL << UFC23_C_USM_AM_MODE_Pos)             // 0x00000000
#define UFC23_C_USM_AM_MODE_DISABLED                    (0)                                             // Cyclic Amplitude Measurement disabled
#define UFC23_C_USM_AM_MODE_ENABLED                     (1)                                             // Cyclic Amplitude Measurement enabled
#define UFC23_C_USM_PWD_MODE_Pos                        (18U)                                           // Pulse width detection Mode
#define UFC23_C_USM_PWD_MODE_Msk                        (0x03UL << UFC23_C_USM_PWD_MODE_Pos)            // 0x000C0000
#define UFC23_C_USM_PWD_MODE_DISABLED                   (0)                                             // Cyclic Pulse width detection disabled
#define UFC23_C_USM_PWD_MODE_SINGLE                     (2)                                             // Cyclic Pulse width detection enabled
#define UFC23_C_USM_PWD_MODE_DOUBLE                     (3)                                             // Cyclic double Pulse width detection enabled
#define UFC23_C_ZCD_LVL_Pos                             (20U)                                           // Zero Cross Detection Level
#define UFC23_C_ZCD_LVL_Msk                             (0x3FFUL << UFC23_C_ZCD_LVL_Pos)                // 0x3FF00000
#define UFC23_C_ZCC_MODE_Pos                            (30U)                                           // Zero Cross Calibration Mode
#define UFC23_C_ZCC_MODE_Msk                            (0x01UL << UFC23_C_ZCC_MODE_Pos)                // 0x40000000
#define UFC23_C_ZCC_INIT_EN_Pos                         (31U)                                           // Zero Cross Calibration
#define UFC23_C_ZCC_INIT_EN_Msk                         (0x01UL << UFC23_C_ZCC_INIT_EN_Pos)             // 0x80000000
#define UFC23_C_ZCC_INIT_EN_DISABLED                    (0)                                             // Zero Cross Calibration not performed during Measure Init Sequence
#define UFC23_C_ZCC_INIT_EN_ENABLED                     (1)                                             // Zero Cross Calibration performed during Measure Init Sequence

//// Bit definition of CR_USM_FBG_MCTRL register
#define UFC23_C_FBG_SEL_Pos                             (0U)                                            // Fire Burst Generator Select
#define UFC23_C_FBG_SEL_Msk                             (0x01UL << UFC23_C_FBG_SEL_Pos)                 // 0x00000001
#define UFC23_C_FBG_SEL_LOW_RESOLUTION                  (0)                                             // Fire Burst Generator Low Resolution (FBG-LR)
#define UFC23_C_FBG_SEL_HIGH_RESOLUTION                 (1)                                             // Fire Burst Generator High Resolution (FBG-HR)
#define UFC23_C_FBG_LR_CLK_DIV_Pos                      (1U)                                            // 
#define UFC23_C_FBG_LR_CLK_DIV_Msk                      (0x3FUL << UFC23_C_FBG_LR_CLK_DIV_Pos)          // 0x0000007E
#define UFC23_C_FBG_FBNUM_Pos                           (7U)                                            // Number of fire burst pulse
#define UFC23_C_FBG_FBNUM_Msk                           (0x3FUL << UFC23_C_FBG_FBNUM_Pos)               // 0x00001F80
#define UFC23_C_FBG_FBNUM_MAX_PULSES                    (63)                                            // Maximum amount of fire burst pulses
#define UFC23_C_FBG_FBSP_Pos                            (13U)                                           // Selection fire burst pulse for fire burst splitting
#define UFC23_C_FBG_FBSP_Msk                            (0x3FUL << UFC23_C_FBG_FBSP_Pos)                // 0x0007E000
#define UFC23_C_FSPLITWID_Pos                           (19U)                                           // Split width
#define UFC23_C_FSPLITWID_Msk                           (0x07UL << UFC23_C_FSPLITWID_Pos)               // 0x00380000
#define UFC23_C_FBG_HR_CLK_DIV_Pos                      (22U)                                           // DCO division factor
#define UFC23_C_FBG_HR_CLK_DIV_Msk                      (0x1FUL << UFC23_C_FBG_HR_CLK_DIV_Pos)          // 0x07C00000

//// Bit definition of CR_USM_FBG_HRC register
#define UFC23_C_FBG_HR_CALIB_Pos                        (0U)                                            // Calibration expected value
#define UFC23_C_FBG_HR_CALIB_Msk                        (0xFFFUL << UFC23_C_FBG_HR_CALIB_Pos)           // 0x00000FFF
#define UFC23_C_FBG_HR_CLB_SEL_Pos                      (12U)                                           // Period cycles for HR oscillator calibration
#define UFC23_C_FBG_HR_CLB_SEL_Msk                      (0x03UL << UFC23_C_FBG_HR_CLB_SEL_Pos)          // 0x00003000
#define UFC23_C_FBG_HR_CLB_SEL_4_CYCLES                 (0)                                             // HR oscillator calibration 4 cycles
#define UFC23_C_FBG_HR_CLB_SEL_2_CYCLES                 (1)                                             // HR oscillator calibration 2 cycles
#define UFC23_C_FBG_HR_CLB_SEL_1_CYCLE                  (2)                                             // HR oscillator calibration 1 cycle
#define UFC23_C_FBG_HR_TRIM_Pos                         (14U)                                           // DCO trimming value to be applied in case
#define UFC23_C_FBG_HR_TRIM_Msk                         (0x3FFUL << UFC23_C_FBG_HR_TRIM_Pos)            // 0x00FFC000
#define UFC23_C_FBG_CALIB_MODE_Pos                      (24U)                                           // Trimming Select
#define UFC23_C_FBG_CALIB_MODE_Msk                      (0x01UL << UFC23_C_FBG_CALIB_MODE_Pos)          // 0x01000000
#define UFC23_C_FBG_CALIB_MODE_AUTO_TRIM                (0)                                             // Auto trimming during calibration
#define UFC23_C_FBG_CALIB_MODE_CONSTANT_TRIM            (1)                                             // Constant trimming value (C_FBG_HR_TRIM)

//// Bit definition of CR_FEP_ANA_CTRL1 register
#define UFC23_C_HS_OSC_TRIM_Pos                         (0U)                                            // High speed oscillator drive capability configuration bits
#define UFC23_C_HS_OSC_TRIM_Msk                         (0x0FUL << UFC23_C_HS_OSC_TRIM_Pos)             // 0x0000000F
#define UFC23_C_HS_OSC_CFG_Pos                          (4U)                                            // high speed oscillator configuration bits
#define UFC23_C_HS_OSC_CFG_Msk                          (0x07UL << UFC23_C_HS_OSC_CFG_Pos)              // 0x00000070
#define UFC23_C_LS_OSC_CFG_Pos                          (7U)                                            //  low speed oscillator configuration bits
#define UFC23_C_LS_OSC_CFG_Msk                          (0x07UL << UFC23_C_LS_OSC_CFG_Pos)              // 0x00000380
#define UFC23_C_PMU_BG_TRIM_Pos                         (10U)                                           // bandgap voltage trimming
#define UFC23_C_PMU_BG_TRIM_Msk                         (0x0FUL << UFC23_C_PMU_BG_TRIM_Pos)             // 0x00003C00
#define UFC23_C_PMU_BIAS_TRIM_Pos                       (14U)                                           //  central bias current trimming
#define UFC23_C_PMU_BIAS_TRIM_Msk                       (0x1FUL << UFC23_C_PMU_BIAS_TRIM_Pos)           // 0x0007C000
#define UFC23_C_PMU_LDO_SEL_Pos                         (19U)                                           //  LDO regulated voltage trimming
#define UFC23_C_PMU_LDO_SEL_Msk                         (0x07UL << UFC23_C_PMU_LDO_SEL_Pos)             // 0x00380000
#define UFC23_C_ZCD_IBSEL_Pos                           (22U)                                           // Zero crossing detector bias current selection
#define UFC23_C_ZCD_IBSEL_Msk                           (0x03UL << UFC23_C_ZCD_IBSEL_Pos)               // 0x00C00000
#define UFC23_C_ZCD_DAC_VREFN_SEL_Pos                   (24U)                                           // DAC10 negative reference selection
#define UFC23_C_ZCD_DAC_VREFN_SEL_Msk                   (0x01UL << UFC23_C_ZCD_DAC_VREFN_SEL_Pos)       // 0x01000000
#define UFC23_C_PGA_GPIO_SEL_Pos                        (25U)                                           // Connection mode of external PGA 
#define UFC23_C_PGA_GPIO_SEL_Msk                        (0x01UL << UFC23_C_PGA_GPIO_SEL_Pos)            // 0x02000000
#define UFC23_C_PGA_GPIO_SEL_PGA_NOT_CONNECTED          (0)                                             // Connection mode of external PGA: not connected via GPIOs
#define UFC23_C_PGA_GPIO_SEL_PGA_CONNECTED              (1)                                             // Connection mode of external PGA: connected via GPIOs
#define UFC23_C_TX_CAP_MODE_Pos                         (26U)                                           //  Transmit Capacity Mode (X1 Option)
#define UFC23_C_TX_CAP_MODE_Msk                         (0x01UL << UFC23_C_TX_CAP_MODE_Pos)             // 0x04000000
#define UFC23_C_TX_CAP_MODE_DISABLED                    (0)                                             //  Transmit Capacity Mode (X1 Option): dummy cap disabled
#define UFC23_C_TX_CAP_MODE_ENABLED                     (1)                                             //  Transmit Capacity Mode (X1 Option): dummy cap enabled
#define UFC23_C_USVREF_CAP_EN_Pos                       (27U)                                           // Enables discharging of external Vref capacity 
#define UFC23_C_USVREF_CAP_EN_Msk                       (0x01UL << UFC23_C_USVREF_CAP_EN_Pos)           // 0x08000000
#define UFC23_C_USVREF_CAP_EN_DISABLED                  (0)                                             // Discharging of external Vref cap disabled
#define UFC23_C_USVREF_CAP_EN_ENABLED                   (1)                                             // Discharging of external Vref cap enabled 

//// Bit definition of CR_USM_ANA_CTRL2 register
#define UFC23_C_PGA_ST2_GAIN_Pos                        (0U)                                            // PGA2 Gain selection
#define UFC23_C_PGA_ST2_GAIN_Msk                        (0x03UL << UFC23_C_PGA_ST2_GAIN_Pos)            // 0x00000003
#define UFC23_C_PGA_ST2_CBYP_Pos                        (2U)                                            // Enables tgate that bypass PGA stage 2
#define UFC23_C_PGA_ST2_CBYP_Msk                        (0x01UL << UFC23_C_PGA_ST2_CBYP_Pos)            // 0x00000004
#define UFC23_C_PGA_ST2_CBYP_NOT_BYPASSED               (0)                                             // PGA stage 2 not bypassed
#define UFC23_C_PGA_ST2_CBYP_BYPASSED                   (1)                                             // PGA stage 2 bypassed
#define UFC23_C_PGA_ST1_GAIN_Pos                        (3U)                                            // PGA1 Gain selection
#define UFC23_C_PGA_ST1_GAIN_Msk                        (0x1FUL << UFC23_C_PGA_ST1_GAIN_Pos)            // 0x000000F8
#define UFC23_C_PGA_ST1_CBYP_Pos                        (8U)                                            // Enables tgate that bypass PGA stage 1
#define UFC23_C_PGA_ST1_CBYP_Msk                        (0x01UL << UFC23_C_PGA_ST1_CBYP_Pos)            // 0x00000100
#define UFC23_C_PGA_G1_OPEN_Pos                         (9U)                                            // Unity gain configuration for PGA stage 1
#define UFC23_C_PGA_G1_OPEN_Msk                         (0x01UL << UFC23_C_PGA_G1_OPEN_Pos)             // 0x00000200
#define UFC23_C_PGA_G1_OPEN_UNITY_GAIN                  (0)                                             // Unity gain configuration, capacitor disconnected
#define UFC23_C_PGA_G1_OPEN_DEFAULT_GAIN                (1)                                             // Default gain configuration, capacitor connected
#define UFC23_C_PGA_ISEL_Pos                            (10U)                                           // Selection of PGA 1st stage OPAMP bias current
#define UFC23_C_PGA_ISEL_Msk                            (0x03UL << UFC23_C_PGA_ISEL_Pos)                // 0x00000C00
#define UFC23_C_PGA_ISEL_CURRENT_1I                     (0)                                             // PGA 1st stage OPAMP bias current 1I
#define UFC23_C_PGA_ISEL_CURRENT_2I                     (1)                                             // PGA 1st stage OPAMP bias current 2I
#define UFC23_C_PGA_ISEL_CURRENT_3I                     (2)                                             // PGA 1st stage OPAMP bias current 3I
#define UFC23_C_PGA_ISEL_CURRENT_4I                     (3)                                             // PGA 1st stage OPAMP bias current 4I
#define UFC23_C_PGA_ST1_OPAN_ENA_Pos                    (12U)                                           // PGA 1st stage enable (inverting input)
#define UFC23_C_PGA_ST1_OPAN_ENA_Msk                    (0x01UL << UFC23_C_PGA_ST1_OPAN_ENA_Pos)        // 0x00001000
#define UFC23_C_PGA_ST1_OPAN_ENA_DISABLE                (0)                                             // PGA 1st stage inverting input disable
#define UFC23_C_PGA_ST1_OPAN_ENA_ENABLE                 (1)                                             // PGA 1st stage inverting input enable
#define UFC23_C_PGA_ST1_OPAP_ENA_Pos                    (13U)                                           // PGA 1st stage enable (non-inverting input)
#define UFC23_C_PGA_ST1_OPAP_ENA_Msk                    (0x01UL << UFC23_C_PGA_ST1_OPAP_ENA_Pos)        // 0x00002000
#define UFC23_C_PGA_ST1_OPAP_ENA_DISABLE                (0)                                             // PGA 1st stage non-inverting input disable
#define UFC23_C_PGA_ST1_OPAP_ENA_ENABLE                 (1)                                             // PGA 1st stage non-inverting input enable
#define UFC23_C_PGA_ST2_OPA_ENA_Pos                     (14U)                                           // PGA 2nd stage enable
#define UFC23_C_PGA_ST2_OPA_ENA_Msk                     (0x01UL << UFC23_C_PGA_ST2_OPA_ENA_Pos)         // 0x00004000
#define UFC23_C_PGA_ST2_OPA_ENA_DISABLE                 (0)                                             // PGA 2nd stage disable
#define UFC23_C_PGA_ST2_OPA_ENA_ENABLE                  (1)                                             // PGA 2nd stage enable
#define UFC23_C_COMPSEL_SEL_Pos                         (15U)                                           // select source for C_C_COMPSEL[1:0] and C_R_COMPSEL[1:0] from hard wired truth table or registers
#define UFC23_C_COMPSEL_SEL_Msk                         (0x01UL << UFC23_C_COMPSEL_SEL_Pos)             // 0x00008000
#define UFC23_C_R_COMPSEL_Pos                           (16U)                                           // PGA 1st stage compensation resistance selection
#define UFC23_C_R_COMPSEL_Msk                           (0x03UL << UFC23_C_R_COMPSEL_Pos)               // 0x00030000
#define UFC23_C_C_COMPSEL_Pos                           (18U)                                           // PGA 1st stage compensation capacitance selection
#define UFC23_C_C_COMPSEL_Msk                           (0x03UL << UFC23_C_C_COMPSEL_Pos)               // 0x000C0000
#define UFC23_C_RMSET_RX_Pos                            (20U)                                           // RX Resistor Matching Setting
#define UFC23_C_RMSET_RX_Msk                            (0x1FUL << UFC23_C_RMSET_RX_Pos)                // 0x01F00000
#define UFC23_C_RMSET_TX_Pos                            (25U)                                           // TX Resistor Matching Setting
#define UFC23_C_RMSET_TX_Msk                            (0x1FUL << UFC23_C_RMSET_TX_Pos)                // 0x3E000000
#define UFC23_C_SE_ENABLE_Pos                           (30U)                                           // Enables single ended (water system)
#define UFC23_C_SE_ENABLE_Msk                           (0x01UL << UFC23_C_SE_ENABLE_Pos)               // 0x40000000

//// Bit definition of CR_USM_RCV_INIT register
#define UFC23_C_US_VR_INIT_Pos                          (0U)                                            // US Vref Init Time in us
#define UFC23_C_US_VR_INIT_Msk                          (0xFFUL << UFC23_C_US_VR_INIT_Pos)              // 0x000000FF
#define UFC23_C_PGA_INIT_Pos                            (8U)                                            // PGA Init Time in us
#define UFC23_C_PGA_INIT_Msk                            (0x3FUL << UFC23_C_PGA_INIT_Pos)                // 0x00003F00
#define UFC23_C_COMP_INIT_Pos                           (14U)                                           // RX comparator Init Time in us
#define UFC23_C_COMP_INIT_Msk                           (0x03UL << UFC23_C_COMP_INIT_Pos)               // 0x0000C000
#define UFC23_C_ADC_VR_INIT_Pos                         (16U)                                           //  ADC Vref Init Time in us
#define UFC23_C_ADC_VR_INIT_Msk                         (0x1FUL << UFC23_C_ADC_VR_INIT_Pos)             // 0x001F0000
#define UFC23_C_ADC_INIT_Pos                            (21U)                                           // ADC Init Time in us
#define UFC23_C_ADC_INIT_Msk                            (0x07UL << UFC23_C_ADC_INIT_Pos)                // 0x00E00000
#define UFC23_C_USM_INIT_MODE_Pos                       (24U)                                           // Ultrasonic Measurement Initialization Mode
#define UFC23_C_USM_INIT_MODE_Msk                       (0x03UL << UFC23_C_USM_INIT_MODE_Pos)           // 0x03000000
#define UFC23_C_DCO_INIT_Pos                            (26U)                                           // DCO Init Time in us
#define UFC23_C_DCO_INIT_Msk                            (0x03UL << UFC23_C_DCO_INIT_Pos)                // 0x0C000000
#define UFC23_C_DCO_INIT_32                             (0)                                             // DCO Init Time 32 us
#define UFC23_C_DCO_INIT_48                             (1)                                             // DCO Init Time 48 us
#define UFC23_C_DCO_INIT_64                             (2)                                             // DCO Init Time 64 us
#define UFC23_C_DCO_INIT_128                            (3)                                             // DCO Init Time 128 us

//// Bit definition of CR_USM_HIT_CTRL register
#define UFC23_C_TOF_HIT_NO_Pos                          (0U)                                            // Number of TOF hits stored in RAM (counting from START hit)
#define UFC23_C_TOF_HIT_NO_Msk                          (0x3FUL << UFC23_C_TOF_HIT_NO_Pos)              // 0x0000003F
#define UFC23_C_TOF_HIT_RLS_MODE_Pos                    (6U)                                            // TOF Hit Release Mode
#define UFC23_C_TOF_HIT_RLS_MODE_Msk                    (0x01UL << UFC23_C_TOF_HIT_RLS_MODE_Pos)        // 0x00000040
#define UFC23_C_TOF_HIT_RLS_MODE_FIRST_HIT              (0)                                             // Hit release condition derived from First Hit Level Detection only
#define UFC23_C_TOF_HIT_RLS_MODE_USM_HIT                (1)                                             // Hit release condition derived from USM_HIT_RLS_DLY_U/D
#define UFC23_C_TOF_HIT_IGN_MODE_Pos                    (7U)                                            // TOF Hit Ignore Mode
#define UFC23_C_TOF_HIT_IGN_MODE_Msk                    (0x01UL << UFC23_C_TOF_HIT_IGN_MODE_Pos)        // 0x00000080
#define UFC23_C_TOF_HIT_IGN_MODE_IGNORES_NONE           (0)                                             // No TOF Hits ignored
#define UFC23_C_TOF_HIT_IGN_MODE_IGNORES_2              (1)                                             // Ignores every 2. TOF Hit
#define UFC23_C_TOF_MULTIHIT_START_Pos                  (8U)                                            // Number of TOF hits from START HIT before starting multi-hit summation
#define UFC23_C_TOF_MULTIHIT_START_Msk                  (0x1FUL << UFC23_C_TOF_MULTIHIT_START_Pos)      // 0x00001F00
#define UFC23_C_TOF_MULTIHIT_NO_Pos                     (13U)                                           // Number of TOF hits taken for multihit summation
#define UFC23_C_TOF_MULTIHIT_NO_Msk                     (0x1FUL << UFC23_C_TOF_MULTIHIT_NO_Pos)         // 0x0003E000

//// Bit definition of CR_USM_WVM register
#define UFC23_C_USM_FHL_UP_Pos                          (0U)                                            // US Measurement, First Hit Level Up
#define UFC23_C_USM_FHL_UP_Msk                          (0xFFUL << UFC23_C_USM_FHL_UP_Pos)              // 0x000000FF
#define UFC23_C_USM_FHL_DN_Pos                          (8U)                                            // US Measurement, First Hit Level Down
#define UFC23_C_USM_FHL_DN_Msk                          (0xFFUL << UFC23_C_USM_FHL_DN_Pos)              // 0x0000FF00
#define UFC23_C_USM_AM_PD_1_Pos                         (16U)                                           // Amplitude Monitoring, Measure peak 1. Number of hits until peak detection ends
#define UFC23_C_USM_AM_PD_1_Msk                         (0x1FUL << UFC23_C_USM_AM_PD_1_Pos)             // 0x001F0000
#define UFC23_C_USM_AM_PD_2_Pos                         (21U)                                           // Amplitude Monitoring, Measure peak 2. Number of hits until peak detection ends
#define UFC23_C_USM_AM_PD_2_Msk                         (0x1FUL << UFC23_C_USM_AM_PD_2_Pos)             // 0x03E00000
#define UFC23_C_USM_AM_PD_3_Pos                         (26U)                                           // Amplitude Monitoring, Measure peak 3Number of hits until peak detection end
#define UFC23_C_USM_AM_PD_3_Msk                         (0x1FUL << UFC23_C_USM_AM_PD_3_Pos)             // 0x7C000000
#define UFC23_C_AM_VEXTSEL_Pos                          (31U)                                           // C_AM_VEXTSEL
#define UFC23_C_AM_VEXTSEL_Msk                          (0x01UL << UFC23_C_AM_VEXTSEL_Pos)              // 0x80000000
#define UFC23_C_AM_VEXTSEL_PGA_OUTPUT                   (0)                                             // PGA output connected to peakdetector input
#define UFC23_C_AM_VEXTSEL_EXTERNAL_PIN                 (1)                                             // External pin connected to peakdetector input (ext PGA mode)

//// Bit definition of CR_USM_MASK_HR_WIN register
#define UFC23_C_USM_MASK_HR_WIN_UP_Pos                  (0U)                                            // High resolution mask window in up direction
#define UFC23_C_USM_MASK_HR_WIN_UP_Msk                  (0xFFFFUL << UFC23_C_USM_MASK_HR_WIN_UP_Pos)    // 0x0000FFFF
#define UFC23_C_USM_MASK_HR_WIN_DN_Pos                  (16U)                                           // High resolution mask window in dn direction
#define UFC23_C_USM_MASK_HR_WIN_DN_Msk                  (0xFFFFUL << UFC23_C_USM_MASK_HR_WIN_DN_Pos)    // 0xFFFF0000

//// Bit definition of CR_SR_DEVICE_ID register
#define UFC23_C_SR_DEVICE_ID_Pos                        (8U)                                            // Device identification
#define UFC23_C_SR_DEVICE_ID_Msk                        (0xFFUL << UFC23_C_SR_DEVICE_ID_Pos)            // 0x0000FF00
#define UFC23_C_SR_DEVICE_ID_UFC18                      (0x18)                                          // Device identification for UFC18
#define UFC23_C_SR_DEVICE_ID_UFC23                      (0x23)                                          // Device identification for UFC23

//// Byte definitions for the USM Bundle data. For batch mode these definitions are relative to the start of each batch
#define UFC23_USM_BUNDLE_AMPL_UPX3_ADDRESS              (0)             // Byte of the amplitude of the 3 selected waves in up direction
#define UFC23_USM_BUNDLE_PW_UP_FHL_ADDRESS              (1)             // PW of the 2nd and 1st hit in up direction calculated with first hit level threshold
#define UFC23_USM_BUNDLE_PW_UP_ZCL_ADDRESS              (2)             // PW of the 3rd or 4th hit in up direction + number hits received
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_UP_LSB_ADDRESS    (3)             // Multi hit summation in up direction
#define UFC23_USM_BUNDLE_AMPL_DNX3_ADDRESS              (4)             // Byte of the amplitude of the 3 selected waves in down direction
#define UFC23_USM_BUNDLE_PW_DN_FHL_ADDRESS              (5)             // PW of the 2nd and 1st hit in down direction calculated with first hit level threshold
#define UFC23_USM_BUNDLE_PW_DN_ZCL_ADDRESS              (6)             // PW of the 3rd or 4th hit in down direction + number hits received
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_DN_LSB_ADDRESS    (7)             // Multi hit summation in down direction
#define UFC23_USM_BUNDLE_VCC_ADDRESS                    (8)             // Supply measurement value
#define UFC23_USM_BUNDLE_HCC_CALIB_ADDRESS              (9)             // High speed oscillator calibration value
#define UFC23_USM_BUNDLE_ZC_LVL_ADDRESS                 (10)            // Zero Cross threshold level (either fixed or by calibration)
#define UFC23_USM_BUNDLE_STATUS_ADDRESS                 (11)            // Batch error and status flags

#define UFC23_USM_SINGLE_CYCLE_US_TOF_HIT_UP_1          (16)            // Index of the ToF of the START HIT in up direction. Only available in single cycle mode. 60 more hits follow
#define UFC23_USM_SINGLE_CYCLE_US_TOF_HIT_DN_1          (80)            // Index of the ToF of the START HIT in down direction. Only available in single cycle mode. 60 more hits follow

#define UFC23_USM_BUNDLE_GAIN_COMPENSATION_SEQ1         (144)           // Relative position of the Gain compensation seq1 byte
#define UFC23_USM_BUNDLE_RDSON_COMPENSATION_SEQ1        (145)           // Relative position of the RDSON compensation seq1 byte
#define UFC23_USM_BUNDLE_REFERENCE_PORT_SEQ1            (146)           // Relative position of the Reference port seq1 byte
#define UFC23_USM_BUNDLE_MEASURE_PORT_1_SEQ1            (147)           // Relative position of the Measure port 1 seq1 byte
#define UFC23_USM_BUNDLE_MEASURE_PORT_2_SEQ1            (148)           // Relative position of the Measure port 2 seq1 byte
#define UFC23_USM_BUNDLE_GAIN_COMPENSATION_SEQ2         (149)           // Relative position of the Gain compensation seq2 byte
#define UFC23_USM_BUNDLE_RDSON_COMPENSATION_SEQ2        (150)           // Relative position of the RDSON compensation seq2 byte
#define UFC23_USM_BUNDLE_REFERENCE_PORT_SEQ2            (151)           // Relative position of the Reference port seq2 byte
#define UFC23_USM_BUNDLE_MEASURE_PORT_1_SEQ2            (152)           // Relative position of the Measure port 1 seq2 byte
#define UFC23_USM_BUNDLE_MEASURE_PORT_2_SEQ2            (153)           // Relative position of the Measure port 2 seq2 byte

//// Bit definitions for the USM Bundle data
#define UFC23_USM_BUNDLE_AMPL_1_Pos                     (0U)                                                    // Start of amplitude data of the 1st selected wave
#define UFC23_USM_BUNDLE_AMPL_1_Msk                     (0x3FFUL << UFC23_USM_BUNDLE_AMPL_1_Pos)                // 0x000003FF
#define UFC23_USM_BUNDLE_AMPL_2_Pos                     (10U)                                                   // Start of amplitude data of the 2nd selected wave
#define UFC23_USM_BUNDLE_AMPL_2_Msk                     (0x3FFUL << UFC23_USM_BUNDLE_AMPL_2_Pos)                // 0x000FFC00
#define UFC23_USM_BUNDLE_AMPL_3_Pos                     (20U)                                                   // Start of amplitude data of the 3rd selected wave
#define UFC23_USM_BUNDLE_AMPL_3_Msk                     (0x3FFUL << UFC23_USM_BUNDLE_AMPL_3_Pos)                // 0x3FF00000
#define UFC23_USM_BUNDLE_PW1_FHL_Pos                    (0U)                                                    // PW of the 1st hit calculated with first hit level threshold
#define UFC23_USM_BUNDLE_PW1_FHL_Msk                    (0xFFFFUL << UFC23_USM_BUNDLE_PW1_FHL_Pos)              // 0xFFFF0000
#define UFC23_USM_BUNDLE_PW2_FHL_Pos                    (16U)                                                   // PW of the 2nd hit calculated with first hit level threshold (optional)
#define UFC23_USM_BUNDLE_PW2_FHL_Msk                    (0xFFFFUL << UFC23_USM_BUNDLE_PW2_FHL_Pos)              // 0x0000FFFF
#define UFC23_USM_BUNDLE_PW_ZCL_Pos                     (16U)                                                   // PW of the 3rd or 4th hit calculated with zero cross threshold (optional)
#define UFC23_USM_BUNDLE_PW_ZCL_Msk                     (0xFFUL << UFC23_USM_BUNDLE_PW_ZCL_Pos)                 // 0x0000FFFF
#define UFC23_USM_BUNDLE_TOF_HIT_NUM_Pos                (8U)                                                   // Number of hits received while transmitting
#define UFC23_USM_BUNDLE_TOF_HIT_NUM_Msk                (0xFFUL << UFC23_USM_BUNDLE_TOF_HIT_NUM_Pos)            // 0x00FF0000
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_Pos               (0U)                                                    // Multi hit summation in up direction MSB
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_Msk               (0xFFUL << UFC23_USM_BUNDLE_TOF_MULTIHIT_Pos)           // 0xFF000000
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Pos           (0U)                                                    // Multi hit summation in up direction LSB
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Msk           (0xFFFFFFFFUL << UFC23_USM_BUNDLE_TOF_MULTIHIT_LSB_Pos) // 0xFFFFFFFF
#define UFC23_USM_BUNDLE_VCC_Pos                        (0U)                                                    // Measured value of VCC
#define UFC23_USM_BUNDLE_VCC_Msk                        (0xFFFFUL << UFC23_USM_BUNDLE_VCC_Pos)                  // 0x0000FFFF
#define UFC23_USM_BUNDLE_VDD_Pos                        (16U)                                                   // Measured value of VDD
#define UFC23_USM_BUNDLE_VDD_Msk                        (0xFFFFUL << UFC23_USM_BUNDLE_VDD_Pos)                  // 0xFFFF0000
#define UFC23_USM_BUNDLE_HCC_CALIB_Pos                  (0U)                                                    // High speed oscillator calibration value
#define UFC23_USM_BUNDLE_HCC_CALIB_Msk                  (0xFFFFFFFFUL << UFC23_USM_BUNDLE_HCC_CALIB_Pos)        // 0xFFFFFFFF
#define UFC23_USM_BUNDLE_ZC_LVL_Pos                     (0U)                                                    // High speed oscillator calibration value
#define UFC23_USM_BUNDLE_ZC_LVL_Msk                     (0x3FFUL << UFC23_USM_BUNDLE_HCC_CALIB_Pos)             // 0x000003FF
#define UFC23_USM_BUNDLE_BATCH_STATUS_Pos               (0U)                                                    // Frontend Error and Status flags
#define UFC23_USM_BUNDLE_BATCH_STATUS_Msk               (0x3FFFFFFUL << UFC23_USM_BUNDLE_HCC_CALIB_Pos)         // 0x03FFFFFF

#define UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Pos        (32U)                                                       // Index of the MSB data of the ToF multihit summation
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Msk        (0xFFLL << UFC23_USM_BUNDLE_TOF_MULTIHIT_64_MSB_Pos)        // Mask of the MSB data of the ToF multihit summation
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Pos        (0U)                                                        // Index of the LSB data of the ToF multihit summation
#define UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Msk        (0xFFFFFFFFLL << UFC23_USM_BUNDLE_TOF_MULTIHIT_64_LSB_Pos)  // Mask of the LSB data of the ToF multihit summation

//// Communication Flag register bits
#define UFC23_CMF_NO_ERROR                              (0)             // No errors or collisions reported
#define UFC23_CMF_SYSTEM_BUS_MASTER_STATE               (1L<<0)         // System Bus Master State bit of Communication Flag Register. 0: SPI, 1:FEP
#define UFC23_CMF_SYSTEM_BUS_COLLISION                  (1L<<1)         // System Bus Collision bit of Communication Flag Register. 1: Collision was detected
#define UFC23_CMF_TASK_EXECUTION                        (1L<<2)         // Task execution bit of Communication Flag Register. 0: IDLE, 1:BUSY
#define UFC23_CMF_MEASURE_CYCLE_TIMER_TAIL_TIME         (1L<<3)         // Measure Cycle Timer Tail Time bit of Communication Flag Register
#define UFC23_CMF_CR_UPDATE_ERROR                       (1L<<6)         // CR update error bit of Communication Flag Register. 1: configuration register was written when not allowed
#define UFC23_CMF_ERROR_FLAG                            (1L<<7)         // Error Flag bit of Communication Flag Register. OR of error flag register

//// Interrupt Flag register bits
#define UFC23_IF_BOOTLOAD_SEQUENCE_DONE                 (1L<<0)         // Bootload Sequence Done bit of Interrupt Flag Register
#define UFC23_IF_MEASURE_INIT_SEQUENCE_DONE             (1L<<1)         // Measure Init Sequence Done bit of Interrupt Flag Register
#define UFC23_IF_MEASURE_CYCLE_SEQUENCE_DONE            (1L<<2)         // Measure Cycle Sequence Done bit of Interrupt Flag Register
#define UFC23_IF_MEASURE_CYCLE_BATCH_DONE               (1L<<3)         // Measure Cycle Batch Done bit of Interrupt Flag Register
#define UFC23_IF_SERVICE_TASK_REQUEST_DONE              (1L<<4)         // Service Task Request Done bit of Interrupt Flag Register
#define UFC23_IF_USM_PAUSE_ERR                          (1L<<5)         // USM_PAUSE_ERR bit of Interrupt Flag Register. Ultrasonic measure sequence not finished within configured pause time (C_USM_PAUSE_TSEL)
#define UFC23_IF_TASK_TIMEOUT                           (1L<<6)         // Task Timeout bit of Interrupt Flag Register. New task requested before the previous was completed
#define UFC23_IF_ERROR_DETECTED                         (1L<<7)         // Error Detected bit of Interrupt Flag Register. Doesn't generate an interrupt on the INTN pin

//// Frontend Error Flag Register register bits
#define UFC23_FES_TDC_TO_HCC                            (1L<<0)         // TDC TO HCC bit of Frontend Error Flag Register. TDC counter Timeout detected during HCC measurement
#define UFC23_FES_TDC_TO_TM                             (1L<<1)         // TDC TO TM bit of Frontend Error Flag Register. TDC counter Timeout detected during temperature measurement
#define UFC23_FES_TM_OPEN                               (1L<<2)         // TM OPEN bit of Frontend Error Flag Register. Temperature Open Circuit Error
#define UFC23_FES_TM_SHORT                              (1L<<3)         // TM SHORT bit of Frontend Error Flag Register. Temperature Short Circuit Error
#define UFC23_FES_USM_HW_ERR_UP                         (1L<<4)         // USM HW ERR UP bit of Frontend Error Flag Register. USM HW error detected in up direction
#define UFC23_FES_USM_HW_ERR_DN                         (1L<<5)         // USM HW ERR DN bit of Frontend Error Flag Register. USM HW error detected in down direction
#define UFC23_FES_TDC_TO_PW_UP                          (1L<<6)         // TDC TO PW UP bit of Frontend Error Flag Register. TDC counter Timeout detected during Pulse Width meas in up direction
#define UFC23_FES_TDC_TO_PW_DN                          (1L<<7)         // TDC TO PW DN bit of Frontend Error Flag Register. TDC counter Timeout detected during Pulse Width meas in dn direction
#define UFC23_FES_TDC_TO_TOF_UP                         (1L<<8)         // TDC TO TOF UP bit of Frontend Error Flag Register. TDC counter timeout detected during TOF meas in up direction
#define UFC23_FES_TDC_TO_TOF_DN                         (1L<<9)         // TDC TO TOF DN bit of Frontend Error Flag Register. TDC counter timeout detected during TOF meas in dn direction
#define UFC23_FES_USM_TO_TOF_UP                         (1L<<10)        // USM TO TOF UP bit of Frontend Error Flag Register. Number of expected TOF hits (single or multi) not received before the programmed timeout (defined by C_USM_TMO_SEL) in up direction
#define UFC23_FES_USM_TO_TOF_DN                         (1L<<11)        // USM TO TOF DN bit of Frontend Error Flag Register. Number of expected TOF hits (single or multi) not received before the programmed timeout (defined by C_USM_TMO_SEL) in down direction
#define UFC23_FES_USM_HCC_UPDATED                       (1L<<16)        // HCC result updated
#define UFC23_FES_USM_ZCC_UPDATED                       (1L<<17)        // ZCC result updated
#define UFC23_FES_USM_VCC_UPDATED                       (1L<<18)        // VCC / VDD results updated
#define UFC23_FES_USM_TEMPERATURE_UPDATED               (1L<<19)        // Temperature results updated
#define UFC23_FES_USM_UP_UPDATED                        (1L<<20)        // USM-UP results updated (up sequence done)
#define UFC23_FES_USM_DN_UPDATED                        (1L<<21)        // USM-DN results updated (dn sequence done)
#define UFC23_FES_USM_PWD_UPDATED                       (1L<<22)        // USM-PWD results updated
#define UFC23_FES_USM_TOF_MULTI_UPDATED                 (1L<<23)        // USM-TOF multi hit results updated
#define UFC23_FES_USM_TOF_SINGLE_UPDATED                (1L<<24)        // USM-TOF single hit results updated
#define UFC23_FES_USM_AM_UPDATED                        (1L<<25)        // USM-AM results updated
#define UFC23_FES_USM_BUNDLE_NUMBER_Pos                 (12U)                                           // Start of USM Bundle number data
#define UFC23_FES_USM_BUNDLE_NUMBER_Msk                 (0x0FUL << UFC23_FES_USM_BUNDLE_NUMBER_Pos)     // 0x0000F000

//// Frontend Status Flag Register register bits
typedef uint8_t Ufc23_TypeMeasurementCompleted;
#define UFC23_EF_HCC_RESULT_UPDATED                     (1<<0)          // HCC result updated bit of Frontend Status Flag Register
#define UFC23_EF_ZCC_RESULT_UPDATED                     (1<<1)          // ZCC result updated bit of Frontend Status Flag Register
#define UFC23_EF_VCC_RESULT_UPDATED                     (1<<2)          // VCC/VDD result updated bit of Frontend Status Flag Register
#define UFC23_EF_TEMPERATURE_RESULTS_UPDATED            (1<<3)          // Temperature results updated bit of Frontend Status Flag Register
#define UFC23_EF_USM_UP_RESULTS_UPDATED                 (1<<4)          // USM-UP results updated bit of Frontend Status Flag Register
#define UFC23_EF_USM_DN_RESULTS_UPDATED                 (1<<5)          // USM-DN results updated bit of Frontend Status Flag Register
#define UFC23_EF_US_ERROR_HANDLING_UPDATED              (1<<6)          // Ultrasonic Error Handling updated bit of Frontend Status Flag Register
#define UFC23_EF_FRONTEND_ERROR_FLAG                    (1<<7)          // Frontend Error flag bit of Frontend Status Flag Register. OR of error flag register bits 11-0

//// System Status Flag Register register bits
#define UFC23_SSF_LSO_SETTLED                           (1<<0)          // LSO Settled (LS_SETUP_TMO) bit of System Status Flag Register. LSO startup timer timeout
#define UFC23_SSF_MAIN_STATE                            (1<<1)          // Main State bit of System Status Flag Register. Main State is Standby or INIT or MEAS_IDLE or MEAS
#define UFC23_SSF_BL_REQ                                (1<<2)          // Bootload main state bit of System Status Flag Register
#define UFC23_SSF_MIS_REQ                               (1<<3)          // Init main state bit of System Status Flag Register
#define UFC23_SSF_MCYCLE_REQ                            (1<<4)          // MCYCLE_REQ bit of System Status Flag Register. MEAS_IDLE or MEAS state
#define UFC23_SSF_MCT_HALT_STATE                        (1<<5)          // Timer halt requested via SPI bit of System Status Flag Register
#define UFC23_SSF_HSO_TMO                               (1<<6)          // HSO TMO bit of System Status Flag Register. HSO not connected
#define UFC23_SSF_ERROR_FLAG                            (1<<7)          // Error flag bit of System Status Flag Register. OR of error flag register

//// Random Area addresses
#define UFC23_RAM_USM_RESULTS_ADDRESS                   (0)             // Address of the start of the USM Main Task Results
#define UFC23_RAM_CONFIG_REGISTER_ADDRESS               (160)           // Address of the start of the Configuration registers

#define UFC23_RANDOM_AREA_RC_RAA_RD_BIT_ADDRESS         (4)             // Bit address of the RC_RAA_RD on the Command Address byte
#define UFC23_RANDOM_AREA_RAA_ADR_MSB_BIT_ADDRESS       (0)             // Bit address of the MSB of the RAA_ADR on the Command Address byte
#define UFC23_RANDOM_AREA_RAA_ADR_LSB_BIT_ADDRESS       (4)             // Bit address of the LSB of the RAA_ADR on the Address byte
#define UFC23_RANDOM_AREA_RC_RAA_RD_BIT_MASK            (0xF0)          // Bit mas for the RC_RAA_RD on the Command Address byte
#define UFC23_RANDOM_AREA_RAA_ADR_MSB_BIT_MASK          (0x0F)          // Bit mas for the MSB of the RAA_ADR on the Command Address byte
#define UFC23_RANDOM_AREA_RAA_ADR_LSB_BIT_MASK          (0xF0)          // Bit mas for the LSB of the RAA_ADR on the Address byte
#define UFC23_RANDOM_AREA_DATA_SIZE                     (4)             // Size in bytes of each of the Random Area data elements

//// Result and Errors
#ifndef SCIOSENSE_RESULT_CODES
#define SCIOSENSE_RESULT_CODES
typedef int8_t Result;
#define RESULT_TIMEOUT                                  (5)             // A timeout was triggered before the result was produced
#define RESULT_NOT_ALLOWED                              (4)             // The requested command is not allowed.
#define RESULT_CHECKSUM_ERROR                           (3)             // The value was read, but the checksum over the payload (valid and data) does not match.
#define RESULT_INVALID                                  (2)             // The value was read, but the data is invalid.
#define RESULT_IO_ERROR                                 (1)             // There was an IO communication error, read/write the stream failed.
#define RESULT_OK                                       (0)             // All OK; The value was read, the checksum matches, and data is valid.
#endif

#endif // SCIOSENSE_UFC23_DEFINES_C_H
