#include "sensor_control.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h" 

#include "main.h"
#include "app_main.h"
#include "stdio.h"

#define ADC_RESOLUTION 4096
#define ADC_BUFFER_LEN 6 // Should be equal to the number of ADC channels

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2; 
extern DAC_HandleTypeDef hdac;

volatile uint16_t adc_buf[ADC_BUFFER_LEN];
uint8_t dataReadyFlag = 0;

float map[11]= {0.0f, 409.6f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 2867.2f, 3276.8f, 3686.4f, 4096.0f};


/**
 * @brief Throttle Input Module
 * @return Throttle value scaled to desired map
 */
uint32_t adc_scale_linear_approx(uint32_t adc_input, float yarray[11]) {
    float xarray[] = {0.0f, 409.6f, 819.2f, 1228.8f, 1638.4f, 2048.0f, 2457.6f, 2867.2f, 3276.8f, 3686.4f, 4096.0f};
	float x0 = 0.0f, x1 = 0.0f, y0 = 0.0f, y1 = 0.0f;

	int i = 0;
	while (xarray[i] < adc_input && i < 11) { // TODO: Improve the safety of this function
		i++;
	}
	x0 = xarray[i - 1];
	x1 = xarray[i];
	y0 = yarray[i - 1];
	y1 = yarray[i];

	uint16_t outputValue = (y1 + (adc_input - x1) * ((y1 - y0) / (x1 - x0))); // Linear Approximation, On a scale of 1-100
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

    for(;;) {
   
        // for (int i = 0; i < 4096; i++) {
        //     HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i);
        //     osDelay(10);
        // }
        
        uint32_t adcOut = adc_scale_linear_approx(adc_buf[0], map);
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adcOut);
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

