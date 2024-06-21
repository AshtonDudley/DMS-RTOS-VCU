#include "sensor_input.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h" 

#include "main.h"
#include "app_main.h"
#include "stdio.h"


#define ADC_BUFFER_LEN 6



extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2; 
extern DAC_HandleTypeDef hdac;

volatile uint16_t adc_buf[ADC_BUFFER_LEN];
uint8_t dataReadyFlag = 0;

 
void sensor_init(){
    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
    HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
    return;
}

float calculateTemp(void){
    float temp = (float)(adc_buf[4] * 0.322265625 / ADC_BUFFER_LEN); // TODO: This needs to be re-evaluated 
    dataReadyFlag = 0;
    return temp;
}


void status_leds_entry(void *argument)
{
    /* USER CODE BEGIN status_leds_entry */
    /* Infinite loop */
    for(;;) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        osDelay(100);
    }
    /* USER CODE END status_leds_entry */
}

void sensor_input_entry(void *argument)
{

    sensor_init();

    for(;;) {
        
        for (int i = 0; i < 4096; i++) {
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i);
            osDelay(10);
        }


        osDelay(1);
    }
}





void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	dataReadyFlag = 1;
    // process_adc_buffer(&adc_buffer[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    dataReadyFlag = 1;
	// process_adc_buffer(&adc_buffer[ADC_SAMPLES * 2]);
}

