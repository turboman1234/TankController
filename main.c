#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "definitions.h"
#include "rcc.h"
#include "initPeripheral.h"
#include "userLibrary.h"
#include "VTimer.h"
#include "usart.h"
#include "mytim.h"
#include "serial.h"
#include "mbslave.h"
#include "rs232.h"
#include "LCD.h"
#include "tankController.h"


int main()
{
    InitRCC();
    InitVTimers();
    InitController();
    InitLCD();
    
    while(1)
    {
        ControllerTask();
        LCDTask();
    }
    
    return 0;
}
