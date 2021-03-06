/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : flexTimer2.c
**     Project     : S32K144_FreeRTOS
**     Processor   : S32K144_100
**     Component   : ftm
**     Version     : Component SDK_S32K144_04, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K144_04
**     Compiler    : GNU C Compiler
**     Date/Time   : 2019-04-17, 21:13, # CodeGen: 14
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc.
**     Copyright 2016-2017 NXP
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file flexTimer2.c
** @version 01.00
*/
/*!
**  @addtogroup flexTimer2_module flexTimer2 module documentation
**  @{
*/

/* Module flexTimer2.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 8.4, external symbol defined without a prior
 * declaration.
 * The symbols are declared in the driver header as external; the header is not included
 * by this file.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variable will be used in other source files in application code.
 */

#include "flexTimer2.h"


/* Fault configuration structure for FTM1 */
ftm_pwm_fault_param_t flexTimer2_FaultConfig =
{
    false, /*!< Output pin state on fault */
    false, /*!< PWM fault interrupt state */
    0U, /* Fault filter value */
    FTM_FAULT_CONTROL_DISABLED,  /*!< Fault mode */
    {
        {
            false, /* Fault Channel state (Enabled/Disabled) */
            false, /* Fault channel filter state (Enabled/Disabled) */
            FTM_POLARITY_LOW  /* Fault Pin Polarity */
        },
        {
            false, /* Fault Channel state (Enabled/Disabled) */
            false, /* Fault channel filter state (Enabled/Disabled) */
            FTM_POLARITY_LOW  /* Fault Pin Polarity */
        },
        {
            false, /* Fault Channel state (Enabled/Disabled) */
            false, /* Fault channel filter state (Enabled/Disabled) */
            FTM_POLARITY_LOW  /* Fault Pin Polarity */
        },
        {
            false, /* Fault Channel state (Enabled/Disabled) */
            false, /* Fault channel filter state (Enabled/Disabled) */
            FTM_POLARITY_LOW  /* Fault Pin Polarity */
        }
   }
};
/* Independent channels configuration structure for flexTimer2 */
ftm_independent_ch_param_t flexTimer2_IndependentChannelsConfig[1] =
{
    {
        0U, /* hwChannelId */
        FTM_POLARITY_LOW, /* Edge mode */
        32768U, /* Duty cycle percent 0-0x8000 */
        false, /* External Trigger */
    }
};
/*PWM configuration for flexTimer2 */
 ftm_pwm_param_t flexTimer2_PwmConfig =
{
     1U, /* Number of independent PWM channels */
     0U, /* Number of combined PWM channels */
     FTM_MODE_CEN_ALIGNED_PWM, /* PWM mode */
     0U, /* Dead time value */
     FTM_DEADTIME_DIVID_BY_1, /* Dead time clock divider */
     1000U, /* PWM frequency */
     flexTimer2_IndependentChannelsConfig, /* Independent PWM channels config structure */
     NULL, /* No PWM channels config structure */
     &flexTimer2_FaultConfig /* PWM fault config structure */
};
/* Timer mode configuration for flexTimer2 */
/* Global configuration of flexTimer2 */
ftm_user_config_t  flexTimer2_InitConfig =
{
    {
        true,   /* Software trigger state */
        false,  /* Hardware trigger 1 state */
        false,  /* Hardware trigger 2 state */
        false,  /* Hardware trigger 3 state */
        false, /* Max loading point state */
        false, /* Min loading point state */
        FTM_SYSTEM_CLOCK, /* Update mode for INVCTRL register */
        FTM_SYSTEM_CLOCK, /* Update mode for SWOCTRL register */
        FTM_SYSTEM_CLOCK, /* Update mode for OUTMASK register */
        FTM_SYSTEM_CLOCK, /* Update mode for CNTIN register */
        false, /* Automatic clear of the trigger*/
        FTM_UPDATE_NOW, /* Synchronization point */
    },
     FTM_MODE_CEN_ALIGNED_PWM, /*!< Mode of operation for FTM */
     FTM_CLOCK_DIVID_BY_4, /* FTM clock prescaler */
     FTM_CLOCK_SOURCE_EXTERNALCLK,   /* FTM clock source */
     FTM_BDM_MODE_00, /* FTM debug mode */
     false, /* Interrupt state */
     false /* Initialization trigger */
};

/* END flexTimer2. */

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/

