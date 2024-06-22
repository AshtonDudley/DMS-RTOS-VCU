#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h" 
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"
#include "math.h"

#include "sensor_control.h"
#include "app_main.h"

#define ADC_RESOLUTION_MAX 4096
#define ADC_RESOLUTION_MIN 0
#define ADC_BUFFER_LEN 6 // Should be equal to the number of ADC channels

#define OFFSET_THRESHOLD 10 // Percent 

#define CUT_MOTOR_SIGNAL 0


extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2; 
extern DAC_HandleTypeDef hdac;

volatile uint16_t adc_buf[ADC_BUFFER_LEN];
uint8_t dataReadyFlag = 0;



typedef struct pedalStatus_s {
    PDP_StatusTypeDef offsetStatus;
    PDP_StatusTypeDef latchStatus;
    PDP_StatusTypeDef sensorStatus;
    bool throttleOutputEnabled;
    bool processADC;
} pedalStatus_t;
 

/// @brief  Singular instance of the pedal object
pedalStatus_t g_pedal = {
    .latchStatus = PDP_OKAY,
    .offsetStatus = PDP_OKAY,
    .sensorStatus = PDP_OKAY,
    .throttleOutputEnabled = false,
    .processADC = true
};


/**
 * @brief  Normalization
 * @return normalized value
 */
float normalize(uint16_t value, uint16_t min, uint16_t max){
    return (float)(value - min) / (max - min); // TODO: Profile performance
}
/**
 * @brief  Normalization
 * @return Inverse of normalized value
 */
uint32_t denormalize(float normalizedValue , uint16_t min, uint16_t max){
    return (uint32_t)(normalizedValue * (max - min) + min);
}

float percentDifference(float a, float b) {
    if (a == b) {
        return 0.0f; // If both numbers are equal, percent difference is 0%
    } else {
        float avg = (a + b) / 2.0f;
        float diff = fabs(a - b); // Absolute difference
        return (diff / avg);
    }
}

/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
float linear_interpolation(float adc_input, float xarray[11], float yarray[11]) {
	float x0 = 0.0f, x1 = 0.0f, y0 = 0.0f, y1 = 0.0f;
	int i = 0;
	while (xarray[i] < adc_input && i < 11) {
		i++;
	}
	x0 = xarray[i - 1];
	x1 = xarray[i];
	y0 = yarray[i - 1];
	y1 = yarray[i];

	float outputValue = (y1 + (adc_input - x1) * ((y1 - y0) / (x1 - x0))); 
	return outputValue;
} 

/**
  * @brief  APPS Agreement Check. Checks if both APPS sensors are within
  * %error threshold of each other.
  * @retval 0 no fault
  * @retval 1 AAC_fault, difference between pedal sensors > %threshold
  */
PDP_StatusTypeDef apps_offset_check(float apps1, float apps2, float thresh) {
    float diff = percentDifference(apps1, apps2);
    if (diff >= thresh) {
		return PDP_ERROR;
    }
    return PDP_OKAY;
}

PDP_StatusTypeDef pedal_plasability_check(pedalStatus_t *pedal, float apps, float bps, float appsLatchThresh, float bpsLatchThresh, float appsRestThreshold) {
    
    if (apps > appsLatchThresh && bps > bpsLatchThresh) {
		return PDP_ERROR;
	} else if (pedal->latchStatus != PDP_OKAY && apps < appsRestThreshold){     // Check if latch can be reset
        return PDP_OKAY;                                                        // Disable latch
    } else if (pedal->latchStatus != PDP_OKAY && apps > appsLatchThresh){       // Waiting for latch to reset fault
        return PDP_ERROR;
    } else {
        return pedal->latchStatus;
    }
}

bool check_faults(pedalStatus_t *pedal){
    if (pedal->latchStatus != PDP_OKAY){
        return false;
    }
    else if (pedal->offsetStatus != PDP_OKAY){
        return false;
    }
    else if (pedal->sensorStatus != PDP_OKAY){
        return false;
    }
    return true;
}

float calculate_temp(void){
    float temp = (float)(adc_buf[4] * 0.322265625 / ADC_BUFFER_LEN); // TODO: This needs to be re-evaluated 
    dataReadyFlag = 0;
    return temp;
}

void set_throttle(bool enable){
    g_pedal.throttleOutputEnabled = enable;
    if (!enable){
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, CUT_MOTOR_SIGNAL);
    }
    return;
}

void sensor_init() {
    // init object 
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
    HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
    return;
}


/// @brief Store Raw ADC values to sensor struct
/// @param sensors 
void set_sensor_adc_values(SensorInfo_t sensors[]) {
    for (int i = 0; i < NUM_SENSORS; i++){
        sensors[i].currentAdcValue = adc_buf[i];
    }
}
/**
 * @brief Convert ADC value to normalized value based on min and max voltage.
 *
 * @param adcValue Raw ADC value to convert.
 * @param minVoltage Minimum voltage corresponding to 0 ADC value.
 * @param maxVoltage Maximum voltage corresponding to maximum ADC value.
 * @return Normalized value in the range [0.0, 1.0].
 */
float adc_to_normalized(int adcValue, float minVoltage, float voltageMax, int adcMax) {
    float adcRefVoltage = 3.3f;    
    // Convert ADC value to voltage
    float adc_voltage = (adcValue / (float)adcMax) * adcRefVoltage;

    // Normalize voltage to a range of 0 to 1
    float normalized = (adc_voltage - minVoltage) / (voltageMax - minVoltage);

    // Ensure normalized value is between 0 and 1
    // if (normalized < 0.0f) normalized = 0.0f;
    // if (normalized > 1.0f) normalized = 1.0f;
    return normalized;
}

void process_adc(SensorInfo_t *sensors){
    
    sensors[APPS1].currentAdcValue = adc_buf[0];
    sensors[APPS2].currentAdcValue = adc_buf[1];
    sensors[FBPS].currentAdcValue  = adc_buf[2];
    sensors[RBPS].currentAdcValue  = adc_buf[3];

    for (int i = 0; i < NUM_SENSORS; ++i){
        sensors[i].normalizedValue = adc_to_normalized(sensors[i].currentAdcValue, sensors[i].voltageMin, sensors[i].voltageMax, ADC_RESOLUTION_MAX);
    }

    // set_sensor_adc_values(sensors);

    // Do scaling and linear approximations as necessary 
    // fbpsNorm = apps2Norm; // FOR TESTING !! 
    
 }


void sensorInputTask(void *argument) {
    (void)argument;
    sensor_init();

    
    float appsLatchThresh = 0.4f, // As percent of throttle
          bpsLatchThresh  = 0.1f, 
          appsResetThresh = 0.3f;
    
    SensorInfo_t sensors[] = {
        [APPS1] = {"APPS1", 1.0f, 2.0f, 0, 0.0f},
        [APPS2] = {"APPS2", 1.0f, 2.0f, 0, 0.0f},
        [FBPS]  = {"FBPS" , 0.0f, 3.3f, 0, 0.0f},
        [RBPS]  = {"RBPS" , 0.0f, 3.3f, 0, 0.0f},       
    };

    for(;;) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);    // LEDs are used for time profiling 

        
        // ADC Processing 
        if (g_pedal.processADC){
            process_adc(sensors);
            g_pedal.throttleOutputEnabled =  check_faults(&g_pedal);
        }

        // g_pedal.offsetStatus = apps_offset_check(adcChannel.adcAPPS1, adcChannel.adcAPPS2, 0.2);
        // g_pedal.latchStatus = pedal_plasability_check(&g_pedal, adcChannel.adcAPPS1, adcChannel.adcFBPS, appsLatchThresh, bpsLatchThresh, appsResetThresh);

        // Check if brake light should be enabled
        // TODO: Add brake light as pedal object paramater?
        if (true){          
            // check_brake_light(adcChannel); 
        }
        
        // throttle Output
        if (g_pedal.throttleOutputEnabled == true){
            // uint32_t dacOut = denormalize(adcChannel.adcAPPS1, ADC_RESOLUTION_MIN, ADC_RESOLUTION_MAX);
            // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacOut);  
        }

        // Cleanup         
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        osDelay(10);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	(void)hadc;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc;
    
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    dataReadyFlag = 1;
}

