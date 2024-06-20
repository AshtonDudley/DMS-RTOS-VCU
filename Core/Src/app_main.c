#include "app_main.h"
#include "stm32f4xx_hal.h"
#include "sensor_input.h"


// Handles 
extern DAC_HandleTypeDef hdac;
extern DAC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

void app_config(){
    sensor_init(&hadc1);
	HAL_TIM_Base_Start(&htim2);
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
}

void app_main(){

}