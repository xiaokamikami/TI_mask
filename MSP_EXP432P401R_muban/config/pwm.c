/*
 * pwm.c
 *
 *  Created on: 2021Äê7ÔÂ6ÈÕ
 *      Author: 13173
 */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "math.h"
#define TIMER_PERIOD 319
    Timer_A_PWMConfig pwmConfig1 =
    {

            TIMER_A_CLOCKSOURCE_ACLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_1,
            TIMER_PERIOD,
            TIMER_A_CAPTURECOMPARE_REGISTER_1,
            TIMER_A_OUTPUTMODE_RESET_SET,
            300
    };

    Timer_A_PWMConfig pwmConfig2 =
    {
            TIMER_A_CLOCKSOURCE_ACLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_1,
            TIMER_PERIOD,
            TIMER_A_CAPTURECOMPARE_REGISTER_2,
            TIMER_A_OUTPUTMODE_RESET_SET,
            100
    };

void PWM_Init(void){

    //![Simple Timer_A Example]
    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    //MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

    // Configuring GPIO2.4 and GPIO2.5 as peripheral output for PWM
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4|GPIO_PIN5,
            GPIO_PRIMARY_MODULE_FUNCTION);


    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
}


// Duty = 100%-0%
void PWM_Duty(int Duty1,int Duty2){
    int Dy1,Dy2;
    int ls = TIMER_PERIOD/100;
    Dy1 = Duty1 *ls;
    Dy2 = Duty2 *ls;
    pwmConfig1.dutyCycle =  Dy1;
    pwmConfig2.dutyCycle = Dy2;
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig2);
}


