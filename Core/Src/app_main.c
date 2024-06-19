#include "app_main.h"
#include "stm32f4xx_hal.h"

// Handles 
extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

void app_config(){
	HAL_TIM_Base_Start(&htim2);
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
}

void app_main(){

}