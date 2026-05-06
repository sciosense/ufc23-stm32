#ifndef SCIOSENSE_UFC23_FILTER
#define SCIOSENSE_UFC23_FILTER

#include <stdint.h>
#include <stdlib.h>

// Filter Parameters (Tune these to your specific sensor dynamics)
#define UFC23_ALPHA_SLOW                0.1     // Extra smoothing
#define UFC23_ALPHA_MED                 0.3     // Balanced response
#define UFC23_ALPHA_FAST                0.5     // Faster response
#define UFC23_SLEW_LIMIT_NS             40      // Max allowed change per sample (in ns)
#define UFC23_NOISE_THRESHOLD_LOW_NS    0.5     // Value below which the slow filter is used
#define UFC23_NOISE_THRESHOLD_HIGH_NS   3.0     // Value above which the fast filter is used

#define UFC23_MAX_STORED_VALUES         128     // Maximum amount of values to store to calculate the noise level
#define UFC23_AMOUNT_STORED_VALUES      4       // Amount of values to store to calculate the noise level 

class Ufc23Filter
{
    private:
        uint8_t filter_initialized;
        // Filter constants
        float alphaSlow;
        float alphaMedium;
        float alphaFast;
        float currentAlpha;
        float noiseThrsLowNs;
        float noiseThrsHighNs;
        float maxSlewRateNs;
        // Stored previous values for noise evaluation
        uint16_t previousValuesIdx;
        uint16_t amountStoredValues;
        float previousValues[UFC23_MAX_STORED_VALUES];
        // Filter elements
        float previousValuesMedian0;
        float previousValuesMedian1;
        float previousValuesMedian2;
        float lastEmaOutput;
    
    public:
        Ufc23Filter()
        {
            filter_initialized      = 0;
            alphaSlow               = UFC23_ALPHA_SLOW;
            alphaMedium             = UFC23_ALPHA_MED;
            alphaFast               = UFC23_ALPHA_FAST;
            currentAlpha            = UFC23_ALPHA_SLOW;
            noiseThrsLowNs          = UFC23_NOISE_THRESHOLD_LOW_NS;
            noiseThrsHighNs         = UFC23_NOISE_THRESHOLD_HIGH_NS;
            maxSlewRateNs           = UFC23_SLEW_LIMIT_NS;
            previousValuesIdx       = 0;
            previousValuesMedian0   = 0;
            previousValuesMedian1   = 0;
            previousValuesMedian2   = 0;
            lastEmaOutput           = 0;
            amountStoredValues      = UFC23_AMOUNT_STORED_VALUES;
        }

    private:
        void initializeFilter(float initialValue)
        {
            previousValuesMedian0 = initialValue;
            previousValuesMedian1 = initialValue;
            previousValuesMedian2 = initialValue;
            
            lastEmaOutput = initialValue;

            for( uint16_t i = 0; i < amountStoredValues; i++ )
            {
                previousValues[i] = initialValue;
            }
        }
        float median3(float a, float b, float c)
        {
            if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
            if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
            return c;
        }

        float getMaxDeviation(float newValue)
        {
            previousValues[previousValuesIdx] = newValue;
            previousValuesIdx = (previousValuesIdx + 1) % amountStoredValues;
            float averagedValue = 0;
            for( uint8_t i=0; i< amountStoredValues; i++ )
            {
                averagedValue += previousValues[i];
            }
            averagedValue = averagedValue / amountStoredValues;
            float maxDeviation = 0;
            for( uint8_t i=0; i< amountStoredValues; i++ )
            {
                float absDiff = abs(averagedValue - previousValues[i]);
                maxDeviation = ( absDiff > maxDeviation ? absDiff : maxDeviation );
            }
            return maxDeviation;
        }

        float calculateAlpha(float maxDeviation)
        {
            float alpha = alphaMedium;
            if( maxDeviation > noiseThrsHighNs )
            {
                alpha = alphaFast;
            }
            else
            {
                if( maxDeviation < noiseThrsLowNs )
                {
                    alpha = alphaSlow;
                }
            }

            return alpha;
        }

    public:
        void FilterUpdateAlpha(float newSlowAlpha, float newMediumAlpha, float newFastAlpha)
        {
            alphaSlow   = newSlowAlpha;
            alphaMedium = newMediumAlpha;
            alphaFast   = newFastAlpha;
        }

        void FilterUpdateAmountPreviousPoints(float amountPreviousPoints)
        {
            previousValuesIdx   = 0;
            amountStoredValues  = amountPreviousPoints;
        }

        void FilterMaxSlewRate(float newmaxSlewRateNs)
        {
            maxSlewRateNs = newmaxSlewRateNs;
        }

        void FilterUpdateThresHolds(float lowThresholdNs, float highThresholdNs)
        {
            noiseThrsLowNs  = lowThresholdNs;
            noiseThrsHighNs = highThresholdNs;
        }

        float GetCurrentAlpha()
        {
            return currentAlpha;
        }

        float ApplyFilter(float newValueNs)
        {
            if( !filter_initialized )
            {
                initializeFilter(newValueNs);
                filter_initialized = 1;
            }

            // 1. Update Median Buffer (Moving Window)
            previousValuesMedian2 = previousValuesMedian1;
            previousValuesMedian1 = previousValuesMedian0;
            previousValuesMedian0 = newValueNs;

            // 2. Stage 1: Median of 3
            float med_val = median3(previousValuesMedian0, previousValuesMedian1, previousValuesMedian2);

            // 3. Stage 2: Slew Rate Limiter
            float diff = med_val - lastEmaOutput;
            if (diff > maxSlewRateNs)
            {
                diff = maxSlewRateNs;
            }
            if (diff < -maxSlewRateNs)
            {
                diff = -maxSlewRateNs;
            }
            float slewed_val = lastEmaOutput + diff;

            // 4. Calculate the adaptative alpha
            currentAlpha = calculateAlpha( getMaxDeviation(newValueNs) );

            // 5. Stage 3: EMA (Exponential Moving Average)
            lastEmaOutput = (currentAlpha * slewed_val) + ((1 - currentAlpha) * lastEmaOutput);

            return lastEmaOutput;
        }
};

#endif //SCIOSENSE_UFC23_FILTER