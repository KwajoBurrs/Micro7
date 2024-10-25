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
//******************************************************************************
// Template File for LAB #7
//******************************************************************************

// Include Files
#include "LcdDrivermsp430/Crystalfontz128x128_ST7735.h"
#include "LcdDrivermsp430/HAL_MSP_EXP430FR5994_Crystalfontz128x128_ST7735.h"
#include "grlib.h"
#include "driverlib.h"
#include <stdint.h>
#include <stdio.h>

// Defines
#define PWMTimerPeriod 50000
//#define PWMTimerPeriod .00125
#define PWMClkDivider TIMER_B_CLOCKSOURCE_DIVIDER_20

#define PORT3 GPIO_PORT_P3
#define PORT2 GPIO_PORT_P2
#define PORT4 GPIO_PORT_P4
#define PIN6 GPIO_PIN6
#define PIN7 GPIO_PIN7
#define PIN5 GPIO_PIN5
#define PIN4 GPIO_PIN4
#define PIN3 GPIO_PIN3
#define PIN2 GPIO_PIN2
#define YHIGH 2575.5
#define YLOW 1552

// Global Variables
Graphics_Context g_sContext;
uint16_t JoyStickX, JoyStickY;
Timer_B_outputPWMParam MyTimerB;
Timer_A_initUpModeParam MyTimerA;
typedef enum {RED,GREEN,BLUE,CYAN,YELLOW,MAGENTA,WHITE,OFF} LedColors;
typedef enum OnOffState {on,off} OnOffState;


//Function Headers
void LCD_init(void);
void ADC_init(void);
void joyStick_init();
void configTimerA(uint16_t,uint16_t);
void myTimerADelay(uint16_t,uint16_t);
void config_PWM(uint16_t,uint16_t);

void config_mkII(void);
void rgbDriver(uint16_t);
LedColors ledState(uint16_t);


void main (void)
{
     uint8_t PBS1, PBS2;
     LedColors ledColor;
     OnOffState onOffState;

     char buffer[100];

    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    // Initialize Joystick
    joyStick_init();

    // Configure mkII
    config_mkII();

    // Activate Configuration
    PMM_unlockLPM5();

    // Initialize ADC
    ADC_init();

    // Initialize LCD
    LCD_init();


    sprintf(buffer,"K. B");
    Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,30,OPAQUE_TEXT);


    // Start Conversion
     ADC12_B_startConversion(ADC12_B_BASE,ADC12_B_START_AT_ADC12MEM0,ADC12_B_REPEATED_SEQOFCHANNELS);


 // Remove this comment statement for Part 2 of the lab

     //  Connect TB0.6 to P3.7 on FR5994
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);

     // Configure PWM and Start Counter
     //config_PWM(PWMTimerPeriod,PWMClkDivider);
     config_PWM(12.5,20);

    // Start PWM timer
     Timer_B_startCounter(TIMER_B0_BASE,TIMER_B_UP_MODE);

 // Remove this comment statement for Part 2 of the lab

     onOffState = on;


     while(1){


         PBS1 = GPIO_getInterruptStatus(PORT4,PIN3);
         //PBS2 = GPIO_getInterruptStatus(GPIO_PORT_P4,GPIO_PIN2);

         if(PBS1 == PIN3)
         {
             if(onOffState == off)
             {
                 onOffState = on;
                 // Determine current state of LED
                 //ledColor = ledState(JoyStickY);
             }
             else if(onOffState == on)
             {
                 onOffState = off;
                 //ledState(0xFFFF);
             }

             GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN3);
         }
         /*
         if(PBS2 == PIN2)
         {

         }
         */


         if(onOffState == on)
         {



        ADC12_B_startConversion(ADC12_B_BASE,ADC12_B_START_AT_ADC12MEM0,ADC12_B_SEQOFCHANNELS);
        while(ADC12_B_getInterruptStatus(ADC12_B_BASE,0,ADC12_B_IFG1) != ADC12_B_IFG1);   // Wait for conversion
        // Get JoyStickX results IFG is cleared
        JoyStickX = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);
        // Get JoyStickY results IFG is cleared
        JoyStickY = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1);
        // Clear IFG1 explicitly
        ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);

        //  Write to the LCD screen
         sprintf(buffer,"JoyStick X 0x%.3x",JoyStickX);
         Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);

         sprintf(buffer,"JoyStick Y 0x%.3x",JoyStickY);
          Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
          // Determine current state of LED
          ledColor = ledState(JoyStickY);
          sprintf(buffer,"                  ");
          Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,20,OPAQUE_TEXT);

         }
         else if(onOffState == off)
         {
             JoyStickY = 0xFFFF;
             //ledState(0xFFFF);
             // Determine current state of LED
             ledColor = ledState(JoyStickY);


             //  Write to the LCD screen
             sprintf(buffer,"JoySticks are OFF");
             Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,20,OPAQUE_TEXT);

             /*
              sprintf(buffer,"K. B");
              Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,30,OPAQUE_TEXT);
              sprintf(buffer,"JoyStick X 0x%.3x",JoyStickX);
              Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);

              sprintf(buffer,"JoyStick Y 0x%.3x",JoyStickY);
              Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
              */
         }

        //  Write to the LCD screen
         /*
        sprintf(buffer,"K. B");
        Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,30,OPAQUE_TEXT);
        sprintf(buffer,"JoyStick X 0x%.3x",JoyStickX);
        Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer, AUTO_STRING_LENGTH,64,40,OPAQUE_TEXT);

        sprintf(buffer,"JoyStick Y 0x%.3x",JoyStickY);
         Graphics_drawStringCentered(&g_sContext,(int8_t*)buffer,AUTO_STRING_LENGTH,64,50,OPAQUE_TEXT);
         */

         // Determine current state of LED
         //ledColor = ledState(JoyStickY);
         // Turn ON RGB LED based on current color
         rgbDriver(ledColor);

         // 1 second delay
         myTimerADelay(50000,TIMER_A_CLOCKSOURCE_DIVIDER_20);



     }

}

// ledState
// Converts joyStick value into a LED color based on lab specs
// Inputs: joyStick Y-axis value
// Returns: RGB LED color or OFF

LedColors ledState(uint16_t joyStickValue){

    if(joyStickValue == 0xFFFF)
    {
        return(BLUE);
    }

    else if(joyStickValue > YHIGH)
    {
        return(RED);
    }

    else if(joyStickValue < YLOW)
    {
        return(GREEN);
    }
    else
    {
        return(YELLOW);
    }



    // Replace with your function
    //return(OFF);

}

// rgbDriver
// Driver for mkII RGB LED
// Inputs: ledColor to display on RGB
// Returns: none

void rgbDriver(LedColors ledColor){

    switch(ledColor)
    {
    case RED:
        GPIO_setOutputHighOnPin(PORT3, PIN6); //RED
        GPIO_setOutputLowOnPin(PORT3, PIN5);  //GREEN
        GPIO_setOutputLowOnPin(PORT3, PIN4);  //BLUE
        break;

    case GREEN:
        GPIO_setOutputLowOnPin(PORT3, PIN6);
        GPIO_setOutputHighOnPin(PORT3, PIN5);
        GPIO_setOutputLowOnPin(PORT3, PIN4);
        break;

    case BLUE:
        GPIO_setOutputLowOnPin(PORT3, PIN6);
        GPIO_setOutputLowOnPin(PORT3, PIN5);
        GPIO_setOutputHighOnPin(PORT3, PIN4);
        break;

    case CYAN:
        GPIO_setOutputLowOnPin(PORT3, PIN6);
        GPIO_setOutputHighOnPin(PORT3, PIN5);
        GPIO_setOutputHighOnPin(PORT3, PIN4);
        break;

    case YELLOW:
        GPIO_setOutputHighOnPin(PORT3, PIN6);
        GPIO_setOutputHighOnPin(PORT3, PIN5);
        GPIO_setOutputLowOnPin(PORT3, PIN4);
        break;

    case MAGENTA:
        GPIO_setOutputHighOnPin(PORT3, PIN6);
        GPIO_setOutputLowOnPin(PORT3, PIN5);
        GPIO_setOutputHighOnPin(PORT3, PIN4);
        break;

    case WHITE:
        GPIO_setOutputHighOnPin(PORT3, PIN6);
        GPIO_setOutputHighOnPin(PORT3, PIN5);
        GPIO_setOutputHighOnPin(PORT3, PIN4);
        break;

    case OFF:
        GPIO_setOutputLowOnPin(PORT3, PIN6);
        GPIO_setOutputLowOnPin(PORT3, PIN5);
        GPIO_setOutputLowOnPin(PORT3, PIN4);
        break;

    default:
        // ALL OFF
        GPIO_setOutputLowOnPin(PORT3, PIN6);
        GPIO_setOutputLowOnPin(PORT3, PIN5);
        GPIO_setOutputLowOnPin(PORT3, PIN4);
        break;

    }

    // Replace with your function

}

// config_mkII
// Configures mkII RGB LED and PB S1 and S2
// Inputs: none
// Returns: none

void config_mkII(){

    GPIO_setAsOutputPin(PORT3, PIN7); // BUZZER

    GPIO_setAsOutputPin(PORT3, PIN6); // RED LED
    GPIO_setAsOutputPin(PORT3, PIN5); // GREEN LED
    GPIO_setAsOutputPin(PORT3, PIN4); // BLUE LED

    //GPIO_setAsInputPinWithPullUpResistor(PORT3, PIN3); // S1 PUSHBUTTON
    //GPIO_setAsInputPinWithPullUpResistor(PORT3, PIN2); // S2 PUSHBUTTON

    GPIO_setAsInputPinWithPullUpResistor(PORT4, PIN3); // S1 PUSHBUTTON
    GPIO_setAsInputPinWithPullUpResistor(PORT4, PIN2); // S2 PUSHBUTTON

    //GPIO_setAsOutputPin(PORT4, PIN3); // JOYSTICK X DIRECTION
    //GPIO_setAsOutputPin(PORT4, PIN2); // JOYSTICK Y DIRECTION

    // Set All Outputs to LOW
    GPIO_setOutputLowOnPin(PORT3, PIN7);
    GPIO_setOutputLowOnPin(PORT3, PIN6);
    GPIO_setOutputLowOnPin(PORT3, PIN5);
    GPIO_setOutputLowOnPin(PORT3, PIN4);

    // Unlock PMM-LPM5 to Activate Configuration
    PMM_unlockLPM5();

    // Replace with your function
}

// joyStick_init
// Configures mkII joysticks to analog inputs on FR599a
// Inputs: none
// Returns: none

void joyStick_init(){

    // JoyStick X
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P3, GPIO_PIN3, GPIO_TERNARY_MODULE_FUNCTION);

    // JoyStick Y
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1, GPIO_PIN2, GPIO_TERNARY_MODULE_FUNCTION);

}

// LCD_Init
// Configures mkII LCD display
// Inputs: none
// Returns: none

void LCD_init(){

/* Initializes display */
Crystalfontz128x128_Init();

/* Set default screen orientation */
Crystalfontz128x128_SetOrientation(0);


/* Initializes graphics context */
Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
Graphics_clearDisplay(&g_sContext);
}

// ADC_init
// Configures ADC to use joystick inputs
// Inputs: none
// Returns: none

void ADC_init(){

    //Initialize the ADC12B Module
    /*
    * Base address of ADC12B Module
    * Use internal ADC12B bit as sample/hold signal to start conversion
    * USE MODOSC 5MHZ Digital Oscillator as clock source
    * Use default clock divider/pre-divider of 1
    * Not use internal channel
    */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);

    /*
    * Base address of ADC12B Module
    * For memory buffers 0-7 sample/hold for 64 clock cycles
    * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
    * Enable Multiple Sampling
    */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
      ADC12_B_CYCLEHOLD_16_CYCLES,
      ADC12_B_CYCLEHOLD_4_CYCLES,
      ADC12_B_MULTIPLESAMPLESENABLE);

    //Configure Memory Buffer
    /*
    * Base address of the ADC12B Module
    * Configure memory buffer 0
    * Map input A1 to memory buffer 0
    * Vref+ = AVcc
    * Vref- = AVss
    * Memory buffer 0 is not the end of a sequence
    */
    //  JoyStickXParam Structure
    ADC12_B_configureMemoryParam joyStickXParam = {0};
    joyStickXParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    joyStickXParam.inputSourceSelect = ADC12_B_INPUT_A2;
    joyStickXParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    joyStickXParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    joyStickXParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    joyStickXParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &joyStickXParam);

    //  JoyStickYParam Structure
    ADC12_B_configureMemoryParam joyStickYParam = {0};
    joyStickYParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    joyStickYParam.inputSourceSelect = ADC12_B_INPUT_A15;
    joyStickYParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    joyStickYParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    joyStickYParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    joyStickYParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &joyStickYParam);


    // Clear Interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);

    //Enable memory buffer 1 interrupt
//    ADC12_B_enableInterrupt(ADC12_B_BASE,ADC12_B_IE1,0,0);

}

// configTimerA
// Configuration Parameters for TimerA
// Inputs: delayValue -- number of count cycles
//         clockDividerValue -- clock divider
// Returns: None

void configTimerA(uint16_t delayValue, uint16_t clockDividerValue)
{
    MyTimerA.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    MyTimerA.clockSourceDivider = clockDividerValue;
    MyTimerA.timerPeriod = delayValue;
    MyTimerA.timerClear = TIMER_A_DO_CLEAR;
    MyTimerA.startTimer = false;
}

// myTimerADelay
// Hardware Timer Delay function using polling with Timer A
// Inputs: delayValue -- number of count cycles
//         clockDividerValue -- clock divider
// Returns: none

void myTimerADelay(uint16_t delayValue, uint16_t clockDividerValue)
{

   configTimerA(delayValue,clockDividerValue);  // Configure the timer parameters
   Timer_A_initUpMode(TIMER_A0_BASE,&MyTimerA); // Initialize the timer
   Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);  // Start Timer
   while((TA0CTL&TAIFG) == 0);                   // Wait for TAIFG to become Set
   Timer_A_stop(TIMER_A0_BASE);                  // Stop timer
   Timer_A_clearTimerInterrupt(TIMER_A0_BASE);   // Reset TAIFG to Zero
}

// config_PWM
// Configures PWM Channel TB0.6
// Inputs: timerPeriod -- number of count cycles
//         timerDivider -- clock divider
// Returns: none


void config_PWM(uint16_t timerPeriod, uint16_t timerDivider){

    MyTimerB.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    MyTimerB.clockSourceDivider = timerDivider;
    MyTimerB.timerPeriod = timerPeriod;
    MyTimerB.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_6;
    MyTimerB.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    MyTimerB.dutyCycle = timerPeriod / 2;
    Timer_B_outputPWM(TIMER_B0_BASE,&MyTimerB);     // Initialize Timer
}

