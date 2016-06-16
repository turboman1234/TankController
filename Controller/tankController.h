#ifndef __TANKCONTROLLER_H
#define __TANKCONTROLLER_H

#include "definitions.h"

// hardware constants
#define FLUID_LEVEL_INPUT                                       ADC_2
#define OUTPUT_FLOW_INPUT                                       ADC_1
#define FLUID_LEVEL_SETPOINT_INPUT                              TRIMMER_1
#define MANUAL_CONTROL_VOLTAGE_INPUT                            TRIMMER_2
#define PUMP_CONTROL_VOLTAGE_OUTPUT                             DAC_2
#define SAMPLE_TIMER                                            TIMER_1
#define LCD_REFRESH_TIMER                                       TIMER_2
#define AUTO_MANUAL_SWITCH                                      SWITCH_1
#define FLUID_LEVEL_SETPOINT_0_CM_BUTTON                        BUTTON_1
#define FLUID_LEVEL_SETPOINT_1_CM_BUTTON                        BUTTON_2
#define FLUID_LEVEL_SETPOINT_2p5_CM_BUTTON                      BUTTON_3
#define FLUID_LEVEL_SETPOINT_5_CM_BUTTON                        BUTTON_4
#define FLUID_LEVEL_SETPOINT_7p5_CM_BUTTON                      BUTTON_5
#define FLUID_LEVEL_SETPOINT_10_CM_BUTTON                       BUTTON_6
#define LED_0_CM                                                LED_1
#define LED_1_CM                                                LED_2
#define LED_2p5_CM                                              LED_3
#define LED_5_CM                                                LED_4
#define LED_7p5_CM                                              LED_5
#define LED_10_CM                                               LED_6
#define LED_MANUAL_MODE                                         LED_7
#define LED_AUTO_MODE                                           LED_8

// simulator's parameters
#define U_MAX                                           3.0                                                     // V
#define H_MAX                                           0.1                                                     // m
#define PUMP_COEF                                       0.000005556                                             // m^3/s.V
#define F_IN_MAX                                        PUMP_COEF * U_MAX                                       // m^3/s
#define F_OUT_MAX                                       F_IN_MAX / 2.0                                          // m^3/s

// controller's constants
#define MIN_DAC_VALUE                                   95
#define MAX_DAC_VALUE                                   4055
#define MIN_ADC_VALUE                                   95
#define MAX_ADC_VALUE                                   4055
#define ADC_CODE_TO_FLUID_LEVEL_CONSTANT                (H_MAX + 0.0001) / (MAX_ADC_VALUE - MIN_ADC_VALUE)                               // (H_MAX + 0.0001) ----> max fluid level transmited by DAC
#define ADC_CODE_TO_OUTPUT_FLOW_CONSTANT                (F_OUT_MAX * 1.0005) / (MAX_ADC_VALUE - MIN_ADC_VALUE)                           // (F_OUT_MAX * 1.0005) ----> ~max output flow rate transmited by DAC
#define ADC_CODE_TO_SETPOINT_CONSTANT                   H_MAX / (MAX_ADC_VALUE - MIN_ADC_VALUE)                                          // trimmer resolution is 12 bits, the first 95 values of which will be ignored
#define ADC_CODE_TO_MANUAL_CONTROL_VOLTAGE              U_MAX / (MAX_ADC_VALUE - MIN_ADC_VALUE)
#define VOLTAGE_TO_DAC_CODE_CONSTANT                    (float)(MAX_DAC_VALUE - MIN_DAC_VALUE) / U_MAX                                          // 
#define SAMPLE_TIME                                     T_100_MS

typedef enum{
    eAutoMode = 0xAA,
    eManualMode
}tControllerWorkMode;

//PID parameters
typedef struct pidParameters{
    float Kp;
    float Ti;
    float Td;
    float N;
    float Tf;
    float b;
    float c;
    float T0;
    float Up;
    float Ui;
    float oldUi;
    float Ud;
    float oldUd;
    float Ci;
    float Cd;
    BOOL ManualToAutoTransitionFlag;
    int startStopController;
    tControllerWorkMode workMode;
}UniversalDPID;


typedef struct signalStructure{
    float currentSetpoint;
    float oldSetpoint;
    float pidControlVoltage;
    float manualControlVoltage;
    float currentFluidLevel;
    float oldFluidLevel;
    float outputFlowRate;
}ControllerSignals;

void InitController(void);
//void SetInitialConditions(void);
//void ReadFluidLevelValue(void);
//void ReadOutputFlowRateValue(void);
//void ReadControllerModeCommand(void);
//void ReadSetpointValue(void);
//void ReadManualControlSignal(void);
//void CalcPIDOutput(void);
//void SetControlVoltageOutput(float controlSignal);
void ControllerTask(void);



#endif
