/*!
 	 \file motor_control.h

 	 \brief This is the header file for a 70:1 37Dx70L[mm] motor with 64 CPR Encoder.
 	 	 	 All the initialization and control functions are found in this source file.

 	 \author HEMI team
 	 	 	 Arpio Fernandez, Leon 				ie702086@iteso.mx
 	 	 	 Barragan Alvarez, Daniel 			ie702554@iteso.mx
 	 	 	 Delsordo Bustillo, Jose Ricardo	ie702570@iteso.mx

 	 \date 	19/04/2019
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "stdint.h"
#include "flexTimer1.h"
#include "flexTimer2.h"
#include "flexTimer3.h"

/** Defines the value for a duty cycle of 0%*/
#define DUTY_CYCLE_INV				(0x8000)
/** Defines the channel used for the PWM*/
#define PWM_CHANNEL					(0U)
/** Defines the edge value used for the PWM*/
#define PWM_EDGE					(0U)
/** Defines the channel used for the input capture*/
#define IC_CHANNEL					(0U)

/*!
 	 \brief Enumerator to define the direction of the motor.
 */
typedef enum
{
	motor_reverse,	/*!< Defines the motor in reverse*/
	motor_forward	/*!< Defines the motor forward*/
}motor_direction_t;

/*!
 	 \brief Structure to define the speed of the motor.
 */
typedef struct
{
	uint8_t RPM;					/*!< RPM of the motor*/
	motor_direction_t direction;	/*!< Direction of the motor*/
}motor_speed_t;

/*!
 	 \brief This function updates the duty cycle of the motor according
 	 	 	 to the desired RPM.

 	 \param[in] new_speed Speed in RPM and direction of the motor.

 	 \return void.
 */
void MC_update_duty_cycle(motor_speed_t new_speed);

/*!
 	 \brief This function returns the speed of the motor according to
 	 	 	 the value received from the IC.

 	 \param[out] speed Speed in RPM and direction of the motor.

 	 \return void.
 */
void MC_get_RPM(motor_speed_t* speed);

#endif /* MOTOR_CONTROL_H_ */
