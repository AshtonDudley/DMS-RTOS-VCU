#include "app_main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#define EXIT_STATE end
#define ENTRY_STATE entry

/* int (*state[])(void) and enum below must be in sync! */
typedef enum { entry, idle, forward, reverse, end }state_codes_t;
int (*state[])(
		void) = {entry_state, idle_state, forward_state, reverse_state, end_state
};


struct transition {
	state_codes_t src_state;
	ret_codes_t ret_code;
	state_codes_t dst_state;
};

// Defines the State, Input, and Next state
struct transition state_transitions[] = {
    {entry,       SM_OKAY,              idle},
	{entry,       SM_FAIL,              entry},
	{idle,        SM_DIR_FORWARD,  	    forward},
	{idle,        SM_DIR_REVERSE,  	    reverse},
	{idle,        SM_OKAY,              idle},
	{forward,     SM_OKAY,              forward},
	{forward,     SM_FAIL,              forward },
	{forward,     SM_CHANGE_MAP,        forward},
	{forward,     SM_VEHCILE_STOPPED,	idle},
	{forward,     SM_ADC_DATA_READY,    forward},
	{reverse,     SM_OKAY,              reverse},
	{reverse,     SM_FAIL,              forward },
	{reverse,     SM_VEHCILE_STOPPED,   idle},
};


void app_config(){
    
	// HAL_TIM_Base_Start(&htim2);
	//HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
}




int entry_state(void){
    return SM_OKAY;
}

int idle_state(void){
    return SM_OKAY;
}

int forward_state(void){
    return 0;
}

int reverse_state(void){
    return 0;
}

int end_state(void){
    return 0;
}

state_codes_t lookup_transitions(state_codes_t cur_state, ret_codes_t rc){
	for (int i = 0; i < sizeof(state_transitions) / sizeof(state_transitions[0]); i++) {
		if (state_transitions[i].src_state == cur_state && state_transitions[i].ret_code == rc) {
            return state_transitions[i].dst_state; // Return the next state
		}
    }
    // Return an error code indicating that no matching transition was found
	return SM_ERROR;
}

void stateMachineTask(void *argument){
    (void)argument;

    state_codes_t cur_state = ENTRY_STATE;
	ret_codes_t rc;
	int (*state_fun)(void);

    for(;;) {

	    state_fun = state[cur_state];
	    rc = state_fun(); // runs the corresponding state function 

        if (rc == SM_ERROR){
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        }
        else {
	        cur_state = lookup_transitions(cur_state, rc);
        }

        osDelay(1);
    }
}