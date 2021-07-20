#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void Key_Init(){
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1); //P1.1 button1
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN4); //P1.4 button2

    GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN1);  //清空中断标识位
    GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN4);  //清空中断标识位

    GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);   //edge
    GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN4,GPIO_HIGH_TO_LOW_TRANSITION);   //edge

    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN4);



}
