#include "app_main.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#define EXIT_STATE end
#define ENTRY_STATE entry

typedef enum { entry, idle, forward, reverse, end }state_codes_t;

/// @brief      Maps a state to it's state transition function, which should be called
///             when the state transitions into this state.
/// @warning    This has to stay in sync with the state_codes_t enum!
int (*state[])(void) = {
    entry_state, 
    idle_state, 
    forward_state, 
    reverse_state, 
    end_state
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

/**
 * @brief Look up the next state based on the current state and return code.
 * @param cur_state The current state of the state machine.
 * @param rc The return code indicating the result of the previous state operation.
 * @return The next state of the state machine if a valid transition is found,
 *         otherwise SM_ERROR.
 */
state_codes_t lookupTransitions(state_codes_t cur_state, ret_codes_t rc){
	for (uint16_t i = 0; i < sizeof(state_transitions) / sizeof(state_transitions[0]); i++) {
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
	    cur_state = lookupTransitions(cur_state, rc);


        osDelay(1);
    }
}