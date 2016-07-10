#ifndef __TANKCONTROLLER_H
#define __TANKCONTROLLER_H

#include "definitions.h"

// hardware constants
#define FLUID_LEVEL_INPUT                                       ADC_2
#define OUTPUT_FLOW_INPUT                                       ADC_1
#define FLUID_LEVEL_SETPOINT_INPUT                              TRIMMER_1
#define MANUAL_CONTROL_VOLTAGE_INPUT                            TRIMMER_2
#define PUMP_CONTROL_VOLTAGE_OUTPUT                             DAC_2
#define LCD_REFRESH_TIMER                                       TIMER_1
#define AUTO_MANUAL_SWITCH                                      SWITCH_1
#define LED_2_CM                                                LED_1
#define LED_4_CM                                                LED_2
#define LED_6_CM                                                LED_3
#define LED_8_CM                                                LED_4
#define LED_10_CM                                               LED_5
#define LED_MANUAL_MODE                                         LED_7
#define LED_AUTO_MODE                                           LED_8

// simulator's parameters
#define U_MAX                                                   10.0                                                    // V
#define U_MIN                                                   3.0                                                     // V
#define H_MAX                                                   0.1                                                     // m
#define TANK_VOLUME                                             0.001                                                   // m3
#define T_FILLING                                               25.0                                                    // s
#define PUMP_COEF                                               (TANK_VOLUME / (U_MAX * T_FILLING))                     // m^3/s.V
#define F_IN_MAX                                                (PUMP_COEF * U_MAX)                                       // m^3/s
#define F_OUT_MAX                                               (F_IN_MAX / 2.0)                                          // m^3/s
#define FLUID_LEVEL_LOW_BORDER                                  0.0001                                                  //m
#define FLUID_LEVEL_HIGH_BORDER                                 (H_MAX + FLUID_LEVEL_LOW_BORDER)                          //m


// controller's constants
#define MIN_DAC_VALUE                                           95
#define MAX_DAC_VALUE                                           4055
#define MIN_ADC_VALUE                                           95
#define MAX_ADC_VALUE                                           4055
#define ADC_CODE_TO_FLUID_LEVEL_CONSTANT                        (FLUID_LEVEL_HIGH_BORDER / (MAX_ADC_VALUE - MIN_ADC_VALUE))                      // (H_MAX + 0.0001) ----> max fluid level transmited by DAC
#define ADC_CODE_TO_OUTPUT_FLOW_CONSTANT                        ((F_OUT_MAX * 1.005) / (MAX_ADC_VALUE - MIN_ADC_VALUE))                          // (F_OUT_MAX * 1.005) ----> ~max output flow rate transmited by DAC
#define ADC_CODE_TO_SETPOINT_CONSTANT                           (H_MAX / (MAX_ADC_VALUE - MIN_ADC_VALUE))                                        // trimmer resolution is 12 bits, the first 95 values of which will be ignored
#define ADC_CODE_TO_MANUAL_CONTROL_VOLTAGE                      (U_MAX / (MAX_ADC_VALUE - MIN_ADC_VALUE))
#define VOLTAGE_TO_DAC_CODE_CONSTANT                            ((float)(MAX_DAC_VALUE - MIN_DAC_VALUE) / U_MAX)                                 // 
#define SAMPLE_TIME                                             T_100_MS

typedef enum{
    eAutoMode = 0xAA,
    eManualMode
}tControllerWorkMode;

//PID parameters
typedef struct pidParameters{
    float Kp;
    float Ti;
    float Td;
    float N;                            // filter constant for D component
    float Tf;                           // anti wind-up constant
    float b;
    float c;
    float T0;
    float Up;                           // Up(k)
    float Ui;                           // Ui(k)
    float oldUi;                        // Ui(k-1)
    float Ud;                           // Ud(k)
    float oldUd;                        // Ud(k-1)
    float Ci;
    float Cd;
    BOOL SaturationFlag;                // Upid > U_MAX = TRUE; Upid < U_MAX = FALSE
    BOOL ManualToAutoTransitionFlag;
    tControllerWorkMode workMode;
}UniversalDPID;


typedef struct signalStructure{
    float currentSetpoint;              // r(k)
    float oldSetpoint;                  // r(k-1)
    float pidControlVoltage;            // Upid(k)
    float oldPidControlVoltage;         // Upid(k-1)
    float manualControlVoltage;         // U(k)
    float currentFluidLevel;            // h(k)     -> y(k)
    float oldFluidLevel;                // h(k-1)   -> y(k-1)
    float outputFlowRate;               // Fout(k)
}ControllerSignals;

void InitControllerPeripheral(void);
void SetInitialConditions(void);
//void ReadFluidLevelValue(void);
//void ReadOutputFlowRateValue(void);
//void ReadControllerModeCommand(void);
//void ReadSetpointValue(void);
//void ReadManualControlSignal(void);
//void CalcPIDOutput(void);
//void SetControlVoltageOutput(float controlSignal);
void ControllerTask(void);
void TIM5_IRQHandler(void);
void ControllerDisplayDataTask(void);

#endif
