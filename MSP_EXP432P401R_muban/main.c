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

#include"oled.h"
#include"bmp.h"
static uint8_t key_mode = 0;
static uint8_t Pepole_temp = 0;
static uint8_t OLED_Clear_Flag = 0;
//![Simple GPIO Config]
int main(void)
{
    volatile uint32_t ii;


    MAP_WDT_A_holdTimer();
    SystemInit();
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /*   初始化        */
    //PWMLED_Init();
    Key_Init();
    GPIO_Init();
    //PWM_Init();
    OLED_Pin_Init();
    OLED_Init();
    //USART1_Init();
    //USART2_Init();
    MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);//Enabling SRAM Bank Retention
    //使能中断
    Interrupt_enableInterrupt(INT_PORT1);

    //Interrupt_enableSleepOnIsrExit(); //休眠CPU

    /* Enabling SRAM Bank Retention */
    //MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);

    MAP_Interrupt_enableMaster();
    OLED_Display_On();
    OLED_Clear();
    OLED_ShowCHinese(0,0,0);      //全
    OLED_ShowCHinese(18,0,1);     //国
    OLED_ShowCHinese(36,0,5);     //电
    OLED_ShowCHinese(54,0,6);    //子
    OLED_ShowCHinese(72,0,9);     //竞
    OLED_ShowCHinese(90,0,10);    //赛

    while (1)
    {

                // delay_ms(5);

                // delay_ms(500);

                // OLED_Clear();
                // delay_ms(5);

                // OLED_DrawBMP(0,0,128,8,BMP1);  //图片显示
                // delay_ms(500);

                // OLED_Clear();
                // delay_ms(5);
        if(key_mode == 1){
            if(OLED_Clear_Flag!=1){
                OLED_Clear_Line(2);
                OLED_Clear_Line(3);
                OLED_Clear_Flag=1;
                OLED_ShowCHinese(0,2,14);    //体
                OLED_ShowCHinese(18,2,15);   //温

                OLED_ShowString(36,2,":");
            }
            OLED_ShowString(54,2,"37.2");
            if(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN6)==1){
                GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);
                GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
                OLED_Clear_Line(4);
                OLED_Clear_Line(5);
                OLED_ShowCHinese(0,4,16);    //口
                OLED_ShowCHinese(18,4,17);   //罩
                OLED_ShowString(36,2,":");
                OLED_ShowCHinese(54,4,19);   //
                OLED_ShowCHinese(72,4,20);   //
                OLED_ShowCHinese(90,4,21);   //
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
        }
        else if(key_mode ==2){
            if(OLED_Clear_Flag!=1){
                OLED_Clear_Line(2);
                OLED_Clear_Line(3);
                OLED_Clear_Line(4);
                OLED_Clear_Line(5);
                delay_ms(5);
                OLED_Clear_Flag=1;
                OLED_ShowCHinese(0,2,22);    //
                OLED_ShowCHinese(18,2,23);   //
                OLED_ShowCHinese(36,2,24);   //
                OLED_ShowCHinese(54,2,25);   //
                OLED_ShowCHinese(72,2,26);   //
                OLED_ShowCHinese(90,2,27);   //
                OLED_ShowCHinese(108,2,28);   //
            }
        }
        else if(key_mode ==3){

            key_mode = 1;
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
        key_mode --;
    }
    else
    {

    }
    OLED_Clear_Flag=0;
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
