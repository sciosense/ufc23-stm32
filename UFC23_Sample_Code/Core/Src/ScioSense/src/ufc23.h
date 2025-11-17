#ifndef SCIOSENSE_UFC23_H
#define SCIOSENSE_UFC23_H

#include <stdint.h>

#include "lib/ufc23/ScioSense_Ufc23.h"
#include "lib/io/ScioSense_IOInterface_STM32.c"

#define ERROR_STRING_LENGTH     60

class UFC23 : public ScioSense_Ufc23
{
    public:
        UFC23();
        ~UFC23();

    public:
        void            begin                               (SPI_HandleTypeDef* spi, uint16_t cs_pin, GPIO_TypeDef* port);                      // Connnects to UFC23 through SPI
        bool            isConnected                         ();                                                                                 // Checks if a communication to the device was established; returns true if so.
        bool            init                                ();                                                                                 // Resets the device and checks if the device responds and has completed the bootup

    public:
        Result          update                              ();                                                                                 // Reads the Frontend Data Buffer and Status
        Result          reset                               ();                                                                                 // Software reset the device
        Result          stopMeasurement                     ();                                                                                 // Sets the device into idle state

    public:
        void            updateConfiguration                 ();                                                                                 // Update the configuration settings of the UFC23. To be used after modifying the configuration structs manually
        Result          readConfig                          ();                                                                                 // Reads the current configuration on the device and updates the internal registers
        Result          writeConfig                         ();                                                                                 // Write and check the configuration into the UFC23
        void            setConfigurationRegisters           (uint32_t* configurationRegisters);                                                 // Set the configuration from an array containing the values for registers 0xA0-0xA7 and 0xA9-0xB2
        Result          startMeasurement                    ();                                                                                 // Perform soft reset and turn the Measure Cycle Timer on

    public:
        bool            hasError                            ();                                                                                 // Checks if any error flag has been activated in the last data update
        uint16_t        getErrors                           ();                                                                                 // Get the Frondend Error flags values from the last data update
        bool            isMeasuring                         ();                                                                                 // Returns true if the device has been configured and is measuring data
        uint8_t         errorToStrings                      (uint32_t errorRegisterContent, char errorStrings[UFC23_AMOUNT_FRONTEND_ERROR_FLAGS][ERROR_STRING_LENGTH]); // Fills the string array with descriptions for the provided errors
        const char*     partIdToString                      (Ufc23_PartID partId);                                                              // Returns a string that corresponds to the Part ID of the device

    public:
        uint8_t         readCommunicationFlags              ();                                                                                 // Reads the Communication flag register from the device
        uint8_t         readInterruptFlags                  ();                                                                                 // Reads the Interrupt flag register from the device
        uint16_t        readFrontendErrorFlags              ();                                                                                 // Reads the Frontend Error flag register from the device
        uint16_t        readFrontendStatusFlags             ();                                                                                 // Reads the Frontend Status flag register from the device
        uint8_t         readSystemStatusFlags               ();                                                                                 // Reads the System Status flag register from the device
        uint16_t        clearFlags                          ();                                                                                 // Clear all the flags from the registers on the device
        Result          triggerSingleMeasurement            (Ufc23_CycleTaskRequest measurementsToTrigger);                                     // Trigger the single measurements provided as argument (different measurement types can be freely combined)
        Result          sendRemoteCommand                   (uint8_t remoteCommand, uint16_t extendedCommand);                                  // Send a remote command to the device
        uint32_t        readRamAddress                      (uint16_t ramAddress);                                                              // Read one 4 byte register from the device RAM at the address provided
        uint32_t        writeRamAddress                     (uint16_t ramAddress, uint32_t registerContent);                                    // Write one 4 byte register to the device RAM at the address provided

    public:
        uint8_t         getAmplitudeMeasurementResultsRaw   (UFC23_AMP_Raw_TypeDef* amplitudesRawUp, UFC23_AMP_Raw_TypeDef* amplitudesRawDn);   // Parse the amplitude results from the last update as a raw value. Returns the amount of valid amplitude measurements
        uint8_t         getAmplitudeMeasurementResultsV     (UFC23_AMP_V_TypeDef* amplitudesVUp, UFC23_AMP_V_TypeDef* amplitudesVDn);           // Parse the amplitude results from the last update in volts. Returns the amount of valid amplitude measurements
        uint8_t         getPulseWidthMeasurementResultsRaw  (UFC23_PW_Raw_TypeDef* pulseWidthsRawUp, UFC23_PW_Raw_TypeDef* pulseWidthsRawDn);   // Parse the Pulse Width results from the last update as a raw value. Returns the amount of valid pulse width measurements
        uint8_t         getPulseWidthMeasurementResultsRatio(UFC23_PW_Ps_TypeDef* pulseWidthsRatioUp, UFC23_PW_Ps_TypeDef* pulseWidthsRatioDn); // Parse the Pulse Width results from the last update as a ratio to the 3rd or 4th hit. Returns the amount of valid pulse width measurements
        uint8_t         getMultiHitSumNs                    (float* tofMultiHitNsUp, float* tofMultiHitNsDn);                                   // Parse the ToF multi-hits sum from the last update in nanoseconds. Returns the amount of valid ToF multi-hits sum measurements
        uint8_t         getMultiHitCount                    (uint8_t* multiHitCountUp, uint8_t* multiHitCountDn);                               // Parse the amount of hits were used for the ToF multi-hits sum from the last update. Returns the amount of valid ToF multi-hits sum measurements
        uint8_t         getVddVcc                           (float* vdd, float* vcc);                                                           // Parse the VCC and VDD measurements from the last update in volts. Returns the amount of valid supply voltage measurements
        uint8_t         getHighSpeedOscillatorFrequencyMhz  (float* hsoMhz);                                                                    // Parse the High-Speed Oscillator measurement from the last update in MHz. Returns the amount of valid frequency measurements
        uint8_t         getZeroCrossLevelV                  (float* zeroCrossLevel);                                                            // Parse the Zero Cross Level measurements from the last update in volts. Returns the amount of valid zero cross level calibrations
        uint8_t         getTemperaturesSeq1DegC             (float* temperature1DegC, float* temperature2DegC);                                 // Parse the temperature measurements of the first sequence from the last update in degrees Centigrade. Returns the amount of valid temperature measurements
        uint8_t         getTemperaturesSeq2DegC             (float* temperature1DegC, float* temperature2DegC);                                 // Parse the temperature measurements of the second sequence from the last update in degrees Centigrade. Returns the amount of valid temperature measurements
        Result          getIndividualTofHitsNs              (float* hitsUpNs, float* hitsDnNs, uint8_t* amountHitsUp, uint8_t* amountHitsDn);   // Parse the individual ToF hits from a single measurement from the last update in nanoseconds and the amount of valid hits
        uint8_t         getAverageHitNs                     (float* tofAvgHitNsUp, float* tofAvgHitNsDn);                                       // Parse the average ToF in nanoseconds by dividing the ToF sum by the number of hits used to calculate it
        float           getSupplyVoltageV                   ();                                                                                 // Parse an individual supply voltage from the last update in volts
        float           getHighSpeedClockFrequencyHz        ();                                                                                 // Parse an individual high-speed oscillator frequency from the last update in MHz
        
    public:
        Result          triggerSpoolPortOpenCheck           ();                                                                                 // Trigger a check of the correct working of the ultrasound transducer. Will set the sensor into Stand-by state
        bool            isSpoolWorkingWell                  ();                                                                                 // Verifies if the ultrasound transducer check gave positive results
        

    public:
        ScioSense_Ufc23                 ufc23Config;

    protected:
        ScioSense_Stm32_Spi_Config      spiConfig;
};

#endif // SCIOSENSE_UFC23_H
