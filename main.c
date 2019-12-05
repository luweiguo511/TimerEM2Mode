/**************************************************************************//**
 * @main_series0.c
 * @brief This project demonstrates frequency generation using the
 * TIMER module. The pin specified in readme.txt is configured for output
 * compare and toggles on each overflow event at a set frequency.
 * @version 0.0.1
 ******************************************************************************
 * @section License
 * <b>Copyright 2018 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "bsp.h"
#include "em_prs.h"
// Desired frequency in Hz
// Min: 107 Hz, Max: 7.5 MHz with default settings
#define OUT_FREQ 1000

// Default prescale value
#define TIMER1_PRESCALE timerPrescale1

/**************************************************************************//**
 * @brief
 *    GPIO initialization
 *****************************************************************************/
void initGpio(void)
{
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PD6 as output
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief
 *    TIMER initialization
 *****************************************************************************/
void initTimer(void)
{
  // Enable clock for TIMER1 module
  CMU_ClockEnable(cmuClock_TIMER1, true);

  // Configure TIMER1 Compare/Capture for output compare
  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.mode = timerCCModeCompare;
  timerCCInit.cmoa = timerOutputActionToggle;
  TIMER_InitCC(TIMER1, 0, &timerCCInit);

  // Route TIMER1 CC0 to location 4 and enable CC0 route pin
  // TIM1_CC0 #4 is GPIO Pin PD6
  TIMER1->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC4)|(TIMER_ROUTE_CC1PEN );

  // Set Top value
  // Note each overflow event constitutes 1/2 the signal period
  //uint32_t topValue = CMU_ClockFreqGet(cmuClock_HFPER) / (2*OUT_FREQ * (1 << TIMER1_PRESCALE))-1;
  TIMER_TopSet(TIMER1, 20);

  // Initialize and start timer with defined prescale
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.enable = false;
  timerInit.prescale = TIMER1_PRESCALE;
  TIMER_Init(TIMER1, &timerInit);
}

void initGpio1(void)
{
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure button 0 as input
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPullFilter, 1);
  // Configure button 0 as input
  GPIO_PinModeSet(gpioPortD, 7, gpioModeInputPullFilter, 1);

  // Select button 0 as the interrupt source (configure as disabled since using PRS)
  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, false, false, false);

  // Configure PD6 as output
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief
 *    PRS initialization
 *
 * @details
 *    prsEdgeOff is chosen because GPIO produces a level signal and the timer
 *    CC0 input can accept either a pulse or level. Thus, we do not need the PRS
 *    module to generate a pulse from the GPIO level signal (we can just leave
 *    it as it is). See the PRS chapter in the reference manual for further
 *    details on Producers and Consumers.
 *****************************************************************************/
void initPrs(void)
{
  // Enable PRS clock
  CMU_ClockEnable(cmuClock_PRS, true);

  // Select GPIO as source and button 0 GPIO pin as signal for PRS channel 0
  // Note that the PRS Channel Source Select splits LOWER (0-7) and HIGHER (8-15) GPIO pins
  if (BSP_GPIO_PB0_PIN < 8) {
    PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOL, BSP_GPIO_PB0_PIN, prsEdgeOff);
  } else {
    PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOH, (uint32_t)(BSP_GPIO_PB0_PIN - 8), prsEdgeOff);
  }
}

void initClockSource(void)
{

  TIMER1->CC[1].CTRL = 0x02000000;
  //CLKSEL
  TIMER1->CTRL = (TIMER0->CTRL & 0xFFFCFFFF) | 0x010000;
  TIMER_Enable(TIMER1, true);

}

/**************************************************************************//**
 * @brief
 *    Main function
 *****************************************************************************/

int main(void)
{
  // Chip errata
  CHIP_Init();

  // Initialization
  initGpio1();
  initPrs();

  initTimer();
  initClockSource();

  while (1) {
    EMU_EnterEM1(); // Enter EM1 (won't exit)
  }
}

