#include "sensor_input.h"
#include "stm32f4xx_hal.h"

#define ADC_BUFFER_LEN 128



extern DAC_HandleTypeDef hdac;

volatile uint16_t adc_buf[ADC_BUFFER_LEN];
volatile uint16_t dac_buf[ADC_BUFFER_LEN];

static volatile uint16_t *inBufPtr;								// TODO: These are currently unused
static volatile uint16_t *outBufPtr = &adc_buf[0];				// https://www.youtube.com/watch?v=zlGSxZGwj-E for how to use them


void sensor_int(){
    return;
}

void status_leds_entry(void *argument)
{
  /* USER CODE BEGIN status_leds_entry */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    osDelay(100);
  }
  /* USER CODE END status_leds_entry */
}

void sensor_input_entry(void *argument)
{
  /* USER CODE BEGIN sensor_input_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END sensor_input_entry */
}

void sensor_init(ADC_HandleTypeDef *sensor_hadc1){
    HAL_ADC_Start_DMA(sensor_hadc1, (uint32_t*)adc_buf, ADC_BUFFER_LEN);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1){

	//inBufPtr  = &adc_buf[0];
	//outBufPtr = &dac_buf[0];
	//dataReadyFlag = 1;



	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);	// Flashing this LED lets us monitor the state
}															// of the buffer using the oscilloscope

/**
  * @brief  This function is executed when  TIM buffer is completely full
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	inBufPtr  = &adc_buf[ADC_BUFFER_LEN / 2];
	outBufPtr = &dac_buf[ADC_BUFFER_LEN / 2];
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}


