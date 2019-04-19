/*!
 	 \file motor_control.c

 	 \brief This is the source file for a 70:1 37Dx70L[mm] motor with 64 CPR Encoder.
 	 	 	 All the initialization and control functions are found in this source file.

 	 \author HEMI team
 	 	 	 Arpio Fernandez, Leon 				ie702086@iteso.mx
 	 	 	 Barragan Alvarez, Daniel 			ie702554@iteso.mx
 	 	 	 Delsordo Bustillo, Jose Ricardo	ie702570@iteso.mx

 	 \date 	19/04/2019
 */

#include "motor_control.h"
#include "flexTimer1.h"
#include "flexTimer2.h"
#include "flexTimer3.h"
#include "pin_mux.h"

/** Defines the initial value for the variables*/
#define INIT_VAL					(0x00)

/** Defines the maximum allowed RPM*/
#define MAX_RPM						(150)
/** Defines the minimum allowed RPM*/
#define MIN_RPM						(13)
/** Defines the maximum frequency given by the encoder*/
#define MAX_FREQ					(2600)

/** RPM divider for the frequency*/
#define RPM_DIVIDER					(2)

/** Variable to store the current motor speed*/
static volatile motor_speed_t current_speed = {INIT_VAL, motor_forward};
/** Variable to store the clock frequency*/
static uint32_t frequency = INIT_VAL;

/** This function updates the PWM ducy cycle according to the speed given*/
void MC_update_duty_cycle(motor_speed_t new_speed)
{
	/** Variable to calculate the new duty cycle*/
	uint16_t new_duty_cycle = INIT_VAL;
	/** Variable to calculate the duty cycle (Inverse slope)*/
	uint16_t inv_duty_cycle = INIT_VAL;

	/** If the RPM received is greater than the maximum RPM*/
	if(MAX_RPM < new_speed.RPM)
	{
		/** Sets the new RPM as the maximum*/
		new_speed.RPM = MAX_RPM;
	}

	/** If the RPM received is lower than the minimum RPM*/
	else if(MIN_RPM > new_speed.RPM)
	{
		/** Stops the motor*/
		new_speed.RPM = INIT_VAL;
	}

	/** Stores the received variable*/
	current_speed.RPM = new_speed.RPM;
	current_speed.direction = new_speed.direction;

	/** Calculates the duty cycle (Inverse slope)*/
	inv_duty_cycle = (uint16_t)((new_speed.RPM * DUTY_CYCLE_INV) / MAX_RPM);

	/** Calculates the new duty cycle*/
	new_duty_cycle = (uint16_t)(DUTY_CYCLE_INV - inv_duty_cycle);

	/** If the motor has stopped*/
	if(INIT_VAL == new_speed.RPM)
	{
        /** Stops both PWM*/
        FTM_DRV_DeinitPwm(INST_FLEXTIMER1);
        FTM_DRV_DeinitPwm(INST_FLEXTIMER2);
	}

	/** If the direction set is reverse*/
	else if(motor_reverse == new_speed.direction)
	{
        /** Stops both PWM*/
        FTM_DRV_DeinitPwm(INST_FLEXTIMER1);
        FTM_DRV_DeinitPwm(INST_FLEXTIMER2);

        /** Restarts PWM 2 (Reverse)*/
        FTM_DRV_InitPwm(INST_FLEXTIMER2, &flexTimer2_PwmConfig);
        /** Updates the PWM duty cycle*/
	    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER2, PWM_CHANNEL, FTM_PWM_UPDATE_IN_DUTY_CYCLE, new_duty_cycle, PWM_EDGE, true);
	}

	/** If the direction is set forward*/
	else
	{
        /** Stops both PWM*/
		FTM_DRV_DeinitPwm(INST_FLEXTIMER2);
        FTM_DRV_DeinitPwm(INST_FLEXTIMER1);

        /** Restarts PWM 1 (Forward)*/
        FTM_DRV_InitPwm(INST_FLEXTIMER1, &flexTimer2_PwmConfig);
        /** Updates the PWM duty cycle*/
	    FTM_DRV_UpdatePwmChannel(INST_FLEXTIMER1, PWM_CHANNEL, FTM_PWM_UPDATE_IN_DUTY_CYCLE, new_duty_cycle, PWM_EDGE, true);
	}
}

/** This function returns the RPM and direction of the motor*/
void MC_get_RPM(motor_speed_t* speed)
{
	/** Variable to calculate the RPM*/
	uint8_t RPM = INIT_VAL;
	/** Variable to read the input capture*/
	uint16_t inputCaptureMeas = INIT_VAL;

    /* Get the FTM1 frequency to calculate
     * the frequency of the measured signal.
     */
    frequency = FTM_DRV_GetFrequency(INST_FLEXTIMER3);

    /** Sets the direction of the motor
     	 (In this case, the direction cannot be known by
     	 reading the IC)*/
	speed->direction = current_speed.direction;

    /** Gets the IC frequency*/
    inputCaptureMeas = FTM_DRV_GetInputCaptureMeasurement(INST_FLEXTIMER3, IC_CHANNEL);
    /** Calculate the signal frequency using recorded data*/
    inputCaptureMeas = frequency / (inputCaptureMeas);

    /** Calculates the RPM according to the frequency*/
    RPM = (uint8_t)((inputCaptureMeas * MAX_RPM) / (MAX_FREQ * RPM_DIVIDER));

    /** Sets the RPM*/
    speed->RPM = RPM;
}
