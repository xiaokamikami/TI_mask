/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 GPIO - Toggle Output High/Low
 *
 * Description: In this very simple example, the LED on P1.0 is configured as
 * an output using DriverLib's GPIO APIs. An infinite loop is then started
 * which will continuously toggle the GPIO and effectively blink the LED.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P1.0  |---> P1.0 LED
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/*Inculde User driverlib */
#include "pwmled.h"
#include "Key.h"
#include "gpio.h"
#include "pwm.h"
#include "usart.h"
static uint8_t key_mode = 0;
static uint8_t ducty1 = 0,ducty2 = 0;
//![Simple GPIO Config]
int main(void)
{
    volatile uint32_t ii;


    MAP_WDT_A_holdTimer();
   // SystemInit();
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    //MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /*   初始化        */
    //PWMLED_Init();
    Key_Init();
    GPIO_Init();
    PWM_Init();
    USART1_Init();
    USART2_Init();
    //使能中断
    Interrupt_enableInterrupt(INT_PORT1);

    //Interrupt_enableSleepOnIsrExit(); //休眠CPU

    /* Enabling SRAM Bank Retention */
    //MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);

    MAP_Interrupt_enableMaster();

    while (1)
    {

        if(key_mode == 1){

        }
        else if(key_mode ==2){

        }
        else if(key_mode ==3){

            key_mode = 0;
        }
        if(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN6)==1){
            GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
        }
        else if(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7)==1){
            GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
        }
        else{
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
        }
        //MAP_PCM_gotoLPM0();
    }
}

//![GPIO IRQHandlerReBack]
void PORT1_IRQHandler(void)
{
    //中断服务
    uint32_t status =0;
    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1); //获取中断状态
    GPIO_clearInterruptFlag(GPIO_PORT_P1,status);   //清除标志位

    if(status & GPIO_PIN1)
    {
        key_mode ++;


    }
    else if( status & GPIO_PIN4 )
    {
        ducty1 +=20;
        ducty2 +=10;
        if(ducty1 > 100)
        {
            ducty1 =0;
        }
        if(ducty2 > 100)
        {
            ducty2 =0;
        }

        PWM_Duty(ducty1,ducty2);
    }
    else
    {

    }
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void TA0_N_IRQHandler(void)
{
    MAP_Timer_A_clearInterruptFlag(TIMER_A0_BASE);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

/* EUSCI A0 UART ISR - Echos data back to PC host */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {

        //MAP_UART_transmitData(EUSCI_A2_BASE, MAP_UART_receiveData(EUSCI_A2_BASE));    //原地返回
        MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A2_BASE));      //返回上位机
    }

}
/* EUSCI A0 UART ISR - Echoes data back to PC host */
//void EUSCIA0_IRQHandler(void)
//{
//    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

//    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
//    {
//        MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
//    }

//}
