#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "initPeripheral.h"
#include "mytim.h"
#include "VTimer.h"
#include "userLibrary.h"
#include "LCD.h"
#include "tankController.h"

ControllerSignals Signals;
UniversalDPID PID;

//LCD Data buffer
ControllerSignals LCDBuffer;
//Row arrays for more friendly writting on LCD
char Row1[ROW_LENGHT], Row2[ROW_LENGHT], Row3[ROW_LENGHT], Row4[ROW_LENGHT];

// Flags for handshaking between tank controller and LCD
BOOL ControllerAcknowledge;
BOOL ControllerReady;
BOOL LCDAcknowledge;
BOOL LCDRequest;

void InitControllerPeripheral(void)
{
    // Init AIs and AOs
    InitADC(FLUID_LEVEL_INPUT);
    InitADC(OUTPUT_FLOW_INPUT);
    InitTrimmer(FLUID_LEVEL_SETPOINT_INPUT);
    InitTrimmer(MANUAL_CONTROL_VOLTAGE_INPUT);
    InitDAC(PUMP_CONTROL_VOLTAGE_OUTPUT, 0);
    
    // Init DIs
    InitSwitch(AUTO_MANUAL_SWITCH);
    
    // Init LEDs
    InitLED(LED_2_CM);
    InitLED(LED_4_CM);
    InitLED(LED_6_CM);
    InitLED(LED_8_CM);
    InitLED(LED_10_CM);
    InitLED(LED_MANUAL_MODE);
    InitLED(LED_AUTO_MODE);
    
    InitTIM5(SAMPLE_TIME);
}

void SetInitialConditions(void)
{
    PID.b = 1;
    PID.c = 0;
    PID.Kp = 100;
    PID.Ti = 0.65;
    PID.Td = 0.1;
    PID.N = 20.0;
    PID.Tf = 20.0;
    PID.T0 = ((float)SAMPLE_TIME / 1000.0);              // 0.1 seconds
    PID.Up = 0.0;
    PID.Ui = 0.0;
    PID.Ud = 0.0;
    
    PID.Ci = PID.T0 / PID.Ti;
    PID.Cd = PID.N / (PID.Td + PID.N * PID.T0);
    
    PID.workMode = eManualMode;
    PID.ManualToAutoTransitionFlag = FALSE;
    
    Signals.pidControlVoltage = 0.0;
    Signals.oldPidControlVoltage = 0.0;
    Signals.oldFluidLevel = 0.0;
    Signals.currentFluidLevel = 0.0;
    Signals.currentSetpoint = 0.0;
    Signals.oldSetpoint = 0.0;
    Signals.outputFlowRate = 0.0;
    
    ControllerAcknowledge = FALSE;
    ControllerReady = FALSE;
    LCDAcknowledge = FALSE;
    LCDRequest = FALSE;
    
    // Start Tank controller
    TIM_Cmd(TIM5, ENABLE);
}

// Read h(k)
void ReadFluidLevelValue(void)
{
    int adcCode;
    
    adcCode = GetAnalogInput(FLUID_LEVEL_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        // adcCode is [0 - 95)
        adcCode = 0;
    }
    else if(adcCode > MAX_ADC_VALUE)
    {
        // adcCode is (4055 - 4095]
        adcCode = MAX_ADC_VALUE - MIN_ADC_VALUE;
    }
    else
    {
        // adcCode is [95 - 4055]
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    // h(k-1) = h(k)
    Signals.oldFluidLevel = Signals.currentFluidLevel;
    
    Signals.currentFluidLevel = (float)adcCode * ADC_CODE_TO_FLUID_LEVEL_CONSTANT;
    
    // simulate sensor's sensibility and eliminate calculation noise
    if(Signals.currentFluidLevel < FLUID_LEVEL_LOW_BORDER)
    {
        Signals.currentFluidLevel = 0.0;
    }
}

// Read Fout(k)
void ReadOutputFlowRateValue(void)
{
    // read output flow rate
    int adcCode;
    
    adcCode = GetAnalogInput(OUTPUT_FLOW_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        // adcCode is [0 - 95)
        adcCode = 0;
    }
    else if(adcCode > MAX_ADC_VALUE)
    {
        // adcCode is (4055 - 4095]
        adcCode = MAX_ADC_VALUE - MIN_ADC_VALUE;
    }
    else
    {
        // adcCode is [95 - 4055]
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    Signals.outputFlowRate = (float)adcCode * ADC_CODE_TO_OUTPUT_FLOW_CONSTANT * 1000000.0; // cm3/s
    
    // simulate sensor's sensibility and eliminate calculation noise
    if(Signals.outputFlowRate < 0.001)
    {
        Signals.outputFlowRate = 0.0;
    }
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

// Read r(k)
void ReadSetpointValue(void)
{
    // read setpoint value
    int adcCode;
    adcCode = GetTrimmerValue(FLUID_LEVEL_SETPOINT_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        // adcCode is [0 - 95)
        adcCode = 0;
    }
    else if(adcCode > MAX_ADC_VALUE)
    {
        // adcCode is (4055 - 4055]
        adcCode = MAX_ADC_VALUE - MIN_ADC_VALUE;
    }
    else
    {
        // adcCode is [95 - 4055]
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    // r(k-1) = r(k)
    Signals.oldSetpoint = Signals.currentSetpoint;
    
    Signals.currentSetpoint = adcCode * ADC_CODE_TO_SETPOINT_CONSTANT;
    
     // simulate sensor's sensibility and eliminate calculation noise
    if(Signals.currentSetpoint < FLUID_LEVEL_LOW_BORDER)
    {
        Signals.currentSetpoint = 0.0;
    }
}

// Read Umanual(k)
void ReadManualControlSignal(void)
{
    int adcCode;
    adcCode = GetTrimmerValue(MANUAL_CONTROL_VOLTAGE_INPUT);
    
    if(adcCode < MIN_ADC_VALUE)
    {
        // adcCode is [0 - 95)
        adcCode = 0;
    }
    else if(adcCode > MAX_ADC_VALUE)
    {
        // adcCode is (4055 - 4095]
        adcCode = MAX_ADC_VALUE - MIN_ADC_VALUE;
    }
    else
    {
        // adcCode is [95 - 4055]
        adcCode = adcCode - MIN_ADC_VALUE;
    }
    
    Signals.manualControlVoltage = adcCode * ADC_CODE_TO_MANUAL_CONTROL_VOLTAGE;
    
    // simulate voltage source sensibility and eliminate calculation noise
    if(Signals.manualControlVoltage < 0.001)
    {
        // Minimum control voltage which can be read is 0.0027777. Smaller values are calculation noise.
        Signals.manualControlVoltage = 0.0;
    }
}

void CalcPIDOutput(void)
{
    // If there is transition from manual to auto mode, recalc Ui(k-1) for shockless transition.
    if(PID.ManualToAutoTransitionFlag == TRUE)
    {
        PID.oldUi = Signals.manualControlVoltage - ((PID.Kp + PID.Ci) * (Signals.currentSetpoint - Signals.currentFluidLevel));

        // Ud(k-1) = Ud(k) = 0.0 - it is accepted that the closed system is in steady state
        PID.oldUd = PID.Ud = 0.0;
        
        PID.ManualToAutoTransitionFlag = FALSE;
    }
    else
    {
        // Ud(k-1) = Ud(k)
        PID.oldUd = PID.Ud;
        // Ui(k-1) = Ui(k)
        PID.oldUi = PID.Ui;
    }
    
    // Calc P component
    // Up(k) = Kp * (b * r(k) - h(k))
    PID.Up = PID.Kp * (PID.b * Signals.currentSetpoint - Signals.currentFluidLevel);
    
    // Calc I component
    if(PID.SaturationFlag == TRUE)
    {
        // Ui(k) = Ui(k-1) + (T0/Ti) * ( e(k-1) - (1/Tf) * (Upid(k-1) - U_MAX)) 
        PID.Ui = PID.oldUi + (PID.T0 / PID.Ti) * ((Signals.oldSetpoint - Signals.oldFluidLevel) - (1 / PID.Tf) * (Signals.oldPidControlVoltage - U_MAX));
    }
    else
    {
        // Ui(k) = Ui(k-1) + (T0/Ti) * e(k-1) 
        PID.Ui = PID.oldUi + (PID.T0 / PID.Ti) * (Signals.oldSetpoint - Signals.oldFluidLevel);
    }
    
    // Calc D component
    // Ud(k) = Cd * T0 * Ud(k-1) + Cd * Td * (c * (r(k) - r(k-1)) + h(k-1) - h(k))
    PID.Ud = PID.Cd * PID.T0 * PID.oldUd + PID.Td * PID.Cd * (PID.c * (Signals.currentSetpoint - Signals.oldSetpoint) + (Signals.oldFluidLevel - Signals.currentFluidLevel));    
    
    // Upid(k-1) = Upid(k)
    Signals.oldPidControlVoltage = Signals.pidControlVoltage;
    
    // Upid(k) = Up(k) + Ui(k) + Ud(k) + 3.0 -> 3.0 is a constant which is added to the control signal for work outside from the pump's deadzone
    Signals.pidControlVoltage = PID.Up + PID.Ui + PID.Ud + 3.0;
    
    // check for saturation
    if(Signals.pidControlVoltage > U_MAX)
    {
        PID.SaturationFlag = TRUE;
    }
    else
    {
        PID.SaturationFlag = FALSE;
    }
}

/* 
    Write Ucontrol(k)
    float controlSignal - Upid(k) or Umanual(k)
*/
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
            ReadFluidLevelValue();              // read h(k)
            ReadOutputFlowRateValue();          // read Fout(k)
            ReadSetpointValue();                // read r(k)
            
            CalcPIDOutput();                    // calc Upid(k)
            
            // write Upid(k) control signal to the DAC
            SetControlVoltageOutput(Signals.pidControlVoltage); 
            
             // read controller mode - auto or manual
            ReadControllerModeCommand();
            
            break;
        }
    case eManualMode:
        {
            ReadFluidLevelValue();              // read h(k)
            ReadOutputFlowRateValue();          // read Fout(k)
            ReadManualControlSignal();          // read Umanual(k)
            
            // write Umanual(k) control signal to the DAC
            SetControlVoltageOutput(Signals.manualControlVoltage);
            
            // read controller mode - auto or manual
            ReadControllerModeCommand();
            
            // set flag to indicate that there is transition from manual to auto mode
            if(PID.workMode == eAutoMode)
            {
                PID.ManualToAutoTransitionFlag = TRUE;
            }
            
            break;
        }
    }
}

// This fuction realize discrete execution for Tank controller
void TIM5_IRQHandler(void)
{
    ControllerTask();
    
    if(LCDRequest == TRUE)
    {
        ControllerAcknowledge = TRUE;
    }
    else if(ControllerAcknowledge == TRUE)
    {
        // send process data to LCD
        LCDBuffer.currentFluidLevel = Signals.currentFluidLevel * 100.0;       // cm
        LCDBuffer.manualControlVoltage = Signals.manualControlVoltage;       // V
        LCDBuffer.currentSetpoint = Signals.currentSetpoint * 100.0;           // cm
        LCDBuffer.outputFlowRate = Signals.outputFlowRate;                   // cm3/s
        
        ControllerAcknowledge = FALSE;
        ControllerReady = TRUE;
    }
    else if(LCDAcknowledge == TRUE)
    {
        ControllerReady = FALSE;
    }
    else
    {
        // do nothing - idle
    }
    
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}

void ControllerDisplayDataTask(void)
{
    char Buffer[81]; // (4 * row lenght) + 1 (for '\0')
    
    if(IsVTimerElapsed(LCD_REFRESH_TIMER) == ELAPSED)
    {
        if(ControllerAcknowledge == TRUE)
        {
            LCDRequest = FALSE;
        }
        else if(ControllerReady == TRUE)
        {
            if(PID.workMode == eAutoMode)
            {
                sprintf(Row1, "Mode:           %4s", "Auto");
                if(LCDBuffer.currentSetpoint < H_MAX*100.0)
                {
                    sprintf(Row4, "Setpoint:   %2.2f, cm", LCDBuffer.currentSetpoint);
                }
                else
                {
                    sprintf(Row4, "Setpoint:  %2.2f, cm", LCDBuffer.currentSetpoint);
                }
            }
            else
            {
                sprintf(Row1, "Mode:         %6s", "Manual");
                if(LCDBuffer.manualControlVoltage == U_MAX)
                {
                    sprintf(Row4, "Pump volt.: %2.2f, V", LCDBuffer.manualControlVoltage);
                }
                else
                {
                    sprintf(Row4, "Pump volt.:  %1.2f, V", LCDBuffer.manualControlVoltage);
                }
            }
            
            if(LCDBuffer.currentFluidLevel < H_MAX*100.0)
            {
                sprintf(Row2, "Fluid level: %2.2f,cm", LCDBuffer.currentFluidLevel);    
            }
            else
            {
                sprintf(Row2, "Fluid level:%2.2f,cm", LCDBuffer.currentFluidLevel);
            }
            
            if(LCDBuffer.outputFlowRate < 10.0)
            {
                sprintf(Row3, "Fout:    %1.2f, cm3/s", LCDBuffer.outputFlowRate);
            }
            else
            {
                sprintf(Row3, "Fout:   %2.2f, cm3/s", LCDBuffer.outputFlowRate);
            }
            
            LCDhome();
            
            strcpy(Buffer, Row1);
            strcat(Buffer, Row3);
            strcat(Buffer, Row2);
            strcat(Buffer, Row4);
            Buffer[80] = '\0';
            
            LCDprint(Buffer);
            
            SetVTimerValue(LCD_REFRESH_TIMER, T_500_MS); // 10 times slower than controller task
            
            LCDAcknowledge = TRUE;
        }
        else
        {
            LCDAcknowledge = FALSE;
            LCDRequest = TRUE;
        }    
    }
}

