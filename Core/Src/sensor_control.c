#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h" 
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"

#include "sensor_control.h"
#include "app_main.h"

#define ADC_RESOLUTION 4096
#define ADC_BUFFER_LEN 6 // Should be equal to the number of ADC channels

#define OFFSET_THRESHOLD 10 // Percent 

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2; 
extern DAC_HandleTypeDef hdac;

volatile uint16_t adc_buf[ADC_BUFFER_LEN];
uint8_t dataReadyFlag = 0;

typedef struct adcChannel_s {
	float adcAPPS1;
	float adcAPPS2;
	float adcFBPS;
	float adcRBPS;
} adcChannel_t;

typedef struct pedalStatus_s {
    PDP_StatusTypeDef offsetStatus;
    PDP_StatusTypeDef latchStatus;
    PDP_StatusTypeDef sensorStatus;
} pedalStatus_t;
 






/**
  * @brief  APPS Agreement Check. Checks if both APPS sensors are within
  * %error threshold of each other.
  * @retval 0 no fault
  * @retval 1 AAC_fault, difference between pedal sensors > %threshold
  */
PDP_StatusTypeDef apps_offset_check(uint32_t apps1, uint32_t apps2) {
    float absDif = abs((int) apps1 - (int) apps2);                      // Calculating percent Difference
	float percentDifference = (absDif / ((apps1 + apps2) / 2)) * 100;   // 

    if (percentDifference >= OFFSET_THRESHOLD) {
		return PDP_ERROR;
    }
    return PDP_OKAY;
}

// PDP_StatusTypeDef pedal_plasability_check(uint32_t apps, uint32_t bps, pedalStatus_t *pedal) {
// 	if (apps > APPS_PAG_THRESHOLD && fbps > FBPS_PAG_THRESHOLD) {
// 		PAG_fault = PDP_ERROR;
// 		return PAG_fault;
// 	}
// }

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
float denormalize(uint16_t normalizedValue , uint16_t min, uint16_t max){
    return (int)(normalizedValue * (max - min) + min);
}





/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
float linear_interpolation(float adc_input, float xarray[11], float yarray[11]) {
	float x0 = 0.0f, x1 = 0.0f, y0 = 0.0f, y1 = 0.0f;
	int i = 0;
	while (xarray[i] < adc_input && i < 11) { // TODO: Improve the safety of this function
		i++;
	}
	x0 = xarray[i - 1];
	x1 = xarray[i];
	y0 = yarray[i - 1];
	y1 = yarray[i];

	float outputValue = (y1 + (adc_input - x1) * ((y1 - y0) / (x1 - x0))); 
	return outputValue;
} 

float calculate_temp(void){
    float temp = (float)(adc_buf[4] * 0.322265625 / ADC_BUFFER_LEN); // TODO: This needs to be re-evaluated 
    dataReadyFlag = 0;
    return temp;
}

void sensor_init() {
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
    HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
    return;
}

void sensorInputTask(void *argument) {
    (void)argument;
    sensor_init();

    adcChannel_t adcChannel;
    pedalStatus_t pedals;


    float xarray[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
    float yarray[] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
    for(;;) {
        float apps1Norm = normalize(adc_buf[0], 0, 4096);
        float apps2Norm = normalize(adc_buf[1], 0, 4096);

        adcChannel.adcAPPS1 = linear_interpolation(apps1Norm, xarray, yarray);
        adcChannel.adcAPPS2 = linear_interpolation(apps2Norm, xarray, yarray);
    

        pedals.offsetStatus = apps_offset_check(adcChannel.adcAPPS1, adcChannel.adcAPPS2);


        uint32_t dacOut = (float)denormalize(adcChannel.adcAPPS1, 0, 4096);
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacOut);      
        
        
        osDelay(10);
    }
}

// TODO: Pedal Plausibility Checks

// TODO: 



void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	(void)hadc;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc;
    
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    dataReadyFlag = 1;
}

