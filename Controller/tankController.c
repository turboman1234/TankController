#include "stm32f4xx.h"
#include <math.h>
#include "initPeripheral.h"
#include "VTimer.h"
#include "userLibrary.h"
#include "tankController.h"

ControllerSignals Signals;
UniversalDPID PID;

void SetInitialConditions(void)
{
    PID.b = 1;
    PID.c = 0;
    PID.Kp = 30;
    PID.Ti = 6;
    PID.Td = 0.1;
    PID.N = 20;
    PID.Tf = PID.Td / PID.N;
    PID.T0 = 0.1; // seconds
    PID.Up = 0.0;
    PID.Ui = 0.0;
    PID.Ud = 0.0;
    
    PID.Ci = PID.T0 / PID.Ti;
    PID.Cd = PID.N / (PID.Td + PID.N * PID.T0);
    
    PID.workMode = eManualMode;
    PID.startStopController = OFF;
    PID.ManualToAutoTransitionFlag = FALSE;
    
    Signals.pidControlVoltage = 0.0;
    Signals.oldFluidLevel = 0.0;
    Signals.currentFluidLevel = 0.0;
    Signals.currentSetpoint = 0.0;
    Signals.oldSetpoint = 0.0;
    Signals.outputFlowRate = 0.0;
}

void InitController(void)
{
    // Init AIs and AOs
    InitADC(FLUID_LEVEL_INPUT);
    InitADC(OUTPUT_FLOW_INPUT);
    InitTrimmer(FLUID_LEVEL_SETPOINT_INPUT);
    InitTrimmer(MANUAL_CONTROL_VOLTAGE_INPUT);
    InitDAC(PUMP_CONTROL_VOLTAGE_OUTPUT, 0);
    
    // Init DIs
    InitSwitch(AUTO_MANUAL_SWITCH);
    InitButton(FLUID_LEVEL_SETPOINT_0_CM_BUTTON);
    InitButton(FLUID_LEVEL_SETPOINT_1_CM_BUTTON);
    InitButton(FLUID_LEVEL_SETPOINT_2p5_CM_BUTTON);
    InitButton(FLUID_LEVEL_SETPOINT_5_CM_BUTTON);
    InitButton(FLUID_LEVEL_SETPOINT_7p5_CM_BUTTON);
    InitButton(FLUID_LEVEL_SETPOINT_10_CM_BUTTON);
    
    // Init LEDs
    InitLED(LED_0_CM);
    InitLED(LED_1_CM);
    InitLED(LED_2p5_CM);
    InitLED(LED_5_CM);
    InitLED(LED_7p5_CM);
    InitLED(LED_10_CM);
    InitLED(LED_MANUAL_MODE);
    InitLED(LED_AUTO_MODE);
    
    SetInitialConditions();
}

void ReadFluidLevelValue(void)
{
    int adcCode;
    
    
    // read fluid level
    
    adcCode = GetAnalogInput(FLUID_LEVEL_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        adcCode = 0;
    }
    else
    {
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    Signals.oldFluidLevel = Signals.currentFluidLevel;
    
    Signals.currentFluidLevel = adcCode * ADC_CODE_TO_FLUID_LEVEL_CONSTANT;
}

void ReadOutputFlowRateValue(void)
{
    // read output flow rate
    int adcCode;
    
    adcCode = GetAnalogInput(OUTPUT_FLOW_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        adcCode = 0;
    }
    else
    {
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    Signals.outputFlowRate = adcCode * ADC_CODE_TO_OUTPUT_FLOW_CONSTANT * 1000000; // cm3/s
}

void ReadControllerModeCommand(void)
{
    // read switch state for define controller's work mode
    int workMode;
    
    workMode = GetSwitchState(AUTO_MANUAL_SWITCH);
    
    if(workMode == OFF)
    {
        // manual mode
        
        PID.workMode = eManualMode;
    }
    else
    {
        // auto mode
        
        PID.workMode = eAutoMode;
    }
}

void ReadSetpointValue(void)
{
    // read setpoint value
    int adcCode;
    adcCode = GetTrimmerValue(FLUID_LEVEL_SETPOINT_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        adcCode = 0;
    }
    else
    {
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    Signals.oldSetpoint = Signals.currentSetpoint;
    
    Signals.currentSetpoint = adcCode * ADC_CODE_TO_SETPOINT_CONSTANT;
}

void ReadManualControlSignal(void)
{
    int adcCode;
    adcCode = GetTrimmerValue(MANUAL_CONTROL_VOLTAGE_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        adcCode = 0;
    }
    else
    {
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    Signals.manualControlVoltage = adcCode * ADC_CODE_TO_MANUAL_CONTROL_VOLTAGE;
}

void CalcPIDOutput(void)
{
    if(PID.ManualToAutoTransitionFlag == TRUE)
    {
        PID.oldUi = Signals.manualControlVoltage - ((PID.Kp + PID.Ci) * (Signals.currentSetpoint - Signals.currentFluidLevel));
        
        PID.ManualToAutoTransitionFlag = FALSE;
    }
    else
    {
        PID.oldUd = PID.Ud;
        PID.oldUi = PID.Ui;
    }
    
    PID.Up = PID.Kp * (PID.b * Signals.currentSetpoint - Signals.currentFluidLevel);
    PID.Ui = PID.oldUi + (PID.T0 / PID.Ti) * (Signals.currentSetpoint - Signals.currentFluidLevel);
    PID.Ud = PID.Cd * PID.T0 * PID.oldUd + PID.Td * PID.Cd * (PID.c * (Signals.currentSetpoint - Signals.oldSetpoint) + (Signals.oldFluidLevel - Signals.currentFluidLevel));
        
    Signals.pidControlVoltage = PID.Up + PID.Ui + PID.Ud;
}

void SetControlVoltageOutput(float controlSignal)
{
    int dacCode;
    
    dacCode = (int)(controlSignal * VOLTAGE_TO_DAC_CODE_CONSTANT) + MIN_DAC_VALUE;
    
    if(dacCode > MAX_DAC_VALUE)
    {
        dacCode = MAX_DAC_VALUE;
    }
    
    SetAnalogOutput(PUMP_CONTROL_VOLTAGE_OUTPUT, dacCode);
}

void ControllerTask(void)
{        
    switch(PID.workMode)
    {
    case eAutoMode:
        {
            ReadFluidLevelValue();
            ReadOutputFlowRateValue();
            ReadSetpointValue();
            
            CalcPIDOutput();
            
            // write calculated control signal to the DAC
            SetControlVoltageOutput(Signals.pidControlVoltage);
            
             // read controller mode - auto or manual
            ReadControllerModeCommand();
            
            break;
        }
    case eManualMode:
        {
            ReadFluidLevelValue();
            ReadOutputFlowRateValue();
            ReadManualControlSignal();
            
            // write calculated control signal to the DAC
            SetControlVoltageOutput(Signals.manualControlVoltage);
            
            // read controller mode - auto or manual
            ReadControllerModeCommand();
            
            if(PID.workMode == eAutoMode)
            {
                PID.ManualToAutoTransitionFlag = TRUE;
            }
            
            break;
        }
    }
}

