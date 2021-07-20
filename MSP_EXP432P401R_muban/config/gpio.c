/*
 * gpio.c
 *
 *  Created on: 2021Äê7ÔÂ6ÈÕ
 *      Author: 13173
 */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void GPIO_Init(){

    //Output
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);    //off
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);    //off
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);    //off

    //Input
    GPIO_setAsInputPin(GPIO_PORT_P3,GPIO_PIN6);
    GPIO_setAsInputPin(GPIO_PORT_P3,GPIO_PIN7);
}


