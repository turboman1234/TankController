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
    InitControllerPeripheral();
    SetInitialConditions();
    InitLCD();
    
    while(1)
    {
        ControllerDisplayDataTask();
    }
    
    return 0;
}

//
//#include "stdio.h"
//#define SIM_OUT_1               DAC_1
//#define SIM_IN_1                ADC_2
//
//#define SIM_OUT_2               DAC_2
//#define SIM_IN_2                ADC_1
//
//int output1Val = 0, output2Val = 0;
//int adc2 = 0, adc1 = 0, trimmer;
//
////---------------------------

//int main()
//{
//    char buff[50];
//    
//    InitADC(SIM_IN_1);
//    InitADC(SIM_IN_2);
//    
//    InitDAC(SIM_OUT_1, output1Val);
//    InitDAC(SIM_OUT_2, output2Val);
//    
//    InitButton(BUTTON_1);
//    InitButton(BUTTON_2);
//    InitButton(BUTTON_3);
//    InitButton(BUTTON_4);
//    
//    InitTrimmer(TRIMMER_1);
//    
////    InitLCD();
//    
//    while(1)
//    {
//        if(GetButtonState(BUTTON_1) == PRESSED)
//        {
//            output1Val = 2048;
//        }
//        if(GetButtonState(BUTTON_2) == PRESSED)
//        {
//            output1Val = 0;
//        }
//        
//        if(GetButtonState(BUTTON_3) == PRESSED)
//        {
//            output2Val = 2048;
//        }
//        if(GetButtonState(BUTTON_4) == PRESSED)
//        {
//            output2Val = 0;
//        }
//        
//        trimmer = GetTrimmerValue(TRIMMER_1);
//        
//        SetAnalogOutput(SIM_OUT_1, trimmer);
//        SetAnalogOutput(SIM_OUT_2, trimmer);
//        
//        adc2 = GetAnalogInput(SIM_IN_1);
//        adc1 = GetAnalogInput(SIM_IN_2);
////        
////        LCDhome();
////        
////        sprintf(buff, "ADC1: %4dADC2: %4dDAC1: %4dDAC: %4d", adc1, adc2, output1Val, output2Val);
////       
////        LCDprint(buff);
//    }
//    return 0;
//}

