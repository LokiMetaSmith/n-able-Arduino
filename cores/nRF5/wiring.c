/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"
#include <nrf.h>
#include <nrfx_clock.h>
#include <stdio.h>
#include "nrf_nvic.h"

nrf_nvic_state_t nrf_nvic_state;
#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_WDT_TIMEOUT_SECONDS
#define CONFIG_WDT_TIMEOUT_SECONDS 5
#endif

#define DFU_MAGIC_SERIAL_ONLY_RESET   0x4e
#define DFU_MAGIC_UF2_RESET           0x57
#define DFU_MAGIC_OTA_RESET           0xA8

#ifdef NRF52_SERIES
    #define DEFAULT_IRQ_PRIO      (2U)

    #if defined(NRF52805_XXAA)
        #define NUM_IRQS          (26U)
    #elif defined(NRF52810_XXAA)
        #define NUM_IRQS          (30U)
    #elif defined(NRF52811_XXAA)
        #define NUM_IRQS          (30U)
    #elif defined(NRF52820_XXAA)
        #define NUM_IRQS          (40U)
    #elif defined(NRF52832_XXAA)
        #define NUM_IRQS          (39U)
    #elif defined(NRF52833_XXAA)
        #define NUM_IRQS          (48U)
    #elif defined(NRF52840_XXAA)
        #define NUM_IRQS          (48U)
    #endif
#elif defined NRF51
    #define DEFAULT_IRQ_PRIO      (1U)
    #define NUM_IRQS              (26U)
#endif

#if !defined(USE_LFXO) && !defined(USE_LFRC) && !defined(USE_LFSYNT)
#warning Low frequency clock source not defined - using RC
#endif

// Must match temp register in bootloader
#define BOOTLOADER_VERSION_REGISTER     NRF_TIMER2->CC[0]
uint32_t bootloaderVersion = 0;
static uint32_t _reset_reason = 0;
static uint8_t hw_clock_hfxo_refcnt;

static void hw_clock_evt_handler(nrfx_clock_evt_type_t event) {
    switch(event){
#if defined(USE_LFRC)
        case NRFX_CLOCK_EVT_CTTO:
            hw_clock_hfxo_request();
            if (nrfx_clock_calibration_start() != NRFX_SUCCESS) {
                nrfx_clock_calibration_timer_start(16);
            }
            break;
        case NRFX_CLOCK_EVT_CAL_DONE:
            hw_clock_hfxo_release();
            // Calibrate every 4 seconds
            nrfx_clock_calibration_timer_start(16);
            break;
#endif
        default:
            break;
    }
}

void init( void )
{
  _reset_reason = NRF_POWER->RESETREAS;

  // clear reset reason: can save it for application usage if needed.
    NRF_POWER->RESETREAS |= NRF_POWER->RESETREAS;


    nrfx_clock_init(hw_clock_evt_handler);
    nrfx_clock_enable();
#if defined(USE_LFSYNT)
    // LFSYNT requires the HF XTAL to always be running.
    hw_clock_hfxo_request();
#endif
    nrfx_clock_start(NRF_CLOCK_DOMAIN_LFCLK);
    while(!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_LFCLK, NULL)){}

  // Retrieve bootloader version
  bootloaderVersion = BOOTLOADER_VERSION_REGISTER;

  // Select Clock Source : XTAL or RC
#if defined( USE_LFXO )
  // 32Khz XTAL
  NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
#elif defined(USE_LFRC)
  // Internal OSC
  NRF_CLOCK->LFCLKSRC = (uint32_t)((CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);
 nrfx_clock_calibration_timer_start(0);
#else
  #error Clock Source is not configured, define USE_LFXO or USE_LFRC according to your board
#endif

#if defined(RESET_PIN)
    if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
        ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos))){
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_UICR->PSELRESET[0] = RESET_PIN;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_UICR->PSELRESET[1] = RESET_PIN;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        NVIC_SystemReset();
    }
#endif
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  // RTC1 could be enabled by bootloader. Disable it
  NVIC_DisableIRQ(RTC1_IRQn);
    // Set vendor IRQ's to default priority level
    for (unsigned i = 0; i < NUM_IRQS; i++) {
        NVIC_SetPriority((IRQn_Type) i, DEFAULT_IRQ_PRIO);
    }
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // Make sure all pin is set HIGH when pinmode() is called
  NRF_P0->OUTSET = UINT32_MAX;

#if CONFIG_WDT_TIMEOUT_SECONDS
    //Configure Watchdog. Pause watchdog while the CPU is halted by the debugger or sleeping.
    NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos);
	NRF_WDT->CRV = CONFIG_WDT_TIMEOUT_SECONDS*32768;
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
	NRF_WDT->TASKS_START = 1;  //Start the Watchdog timer
#endif
#ifdef NRF_P1
  NRF_P1->OUTSET = UINT32_MAX;
#endif
}



uint32_t readResetReason(void)
{
  return _reset_reason;
}

uint32_t getResetReason(void) {
  return readResetReason();
}

void systemPowerOff(void) {
    nrf_power_system_off(NRF_POWER);
}

void systemRestart(void) {
    NVIC_SystemReset();
}

void enterUf2Dfu(void)
{
  NRF_POWER->GPREGRET = DFU_MAGIC_UF2_RESET;
  NVIC_SystemReset();
}

__attribute__ ((__weak__))
void enterSerialDfu(void) {
  NRF_POWER->GPREGRET = DFU_MAGIC_SERIAL_ONLY_RESET;
    NVIC_SystemReset();
}

int hw_clock_hfxo_request(void) {
    int started = 0;
    nrf_clock_hfclk_t clk_src;

    portENTER_CRITICAL();
    assert(hw_clock_hfxo_refcnt < 0xff);
    if (hw_clock_hfxo_refcnt == 0) {
        if (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, &clk_src) ||
           (clk_src != NRF_CLOCK_HFCLK_HIGH_ACCURACY)) {
            nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLK);
            while (!nrfx_clock_is_running(NRF_CLOCK_DOMAIN_HFCLK, &clk_src) ||
                (clk_src != NRF_CLOCK_HFCLK_HIGH_ACCURACY)){}
        }
        started = 1;
    }
    ++hw_clock_hfxo_refcnt;
    portEXIT_CRITICAL();

    return started;
}

int hw_clock_hfxo_release(void) {
    int stopped = 0;

    portENTER_CRITICAL();
    assert(hw_clock_hfxo_refcnt != 0);
    --hw_clock_hfxo_refcnt;
    if (hw_clock_hfxo_refcnt == 0) {
        nrfx_clock_stop(NRF_CLOCK_DOMAIN_HFCLK);
        stopped = 1;
    }
    portEXIT_CRITICAL();

    return stopped;
}


void enterOTADfu(void)
{
  NRF_POWER->GPREGRET = DFU_MAGIC_OTA_RESET;
    NVIC_SystemReset();
}

void waitForEvent(void)
{
#if 0
  // Set bit 7 and bits 4..0 in the mask to one (0x ...00 1001 1111)
  enum { FPU_EXCEPTION_MASK = 0x0000009F };

  /* Clear exceptions and PendingIRQ from the FPU unit */
  __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);
#endif

  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);

#ifdef NRF_CRYPTOCELL
  // manually clear CryptoCell else it could prevent low power mode
  NVIC_ClearPendingIRQ(CRYPTOCELL_IRQn);
#endif

  if ( sd_en )
  {
    (void) sd_app_evt_wait();
  }else
  {
    // SoftDevice is not enabled.
    __WFE();
    __SEV(); // Clear Event Register.
    __WFE();
        }
    }

void systemOff(uint32_t pin, uint8_t wake_logic)
{
//  for(int i=0; i<8; i++)
//  {
//    NRF_POWER->RAM[i].POWERCLR = 0x03UL;
//  }

  pin = g_ADigitalPinMap[pin];

  if ( wake_logic )
  {
    nrf_gpio_cfg_sense_input(pin, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  }else
  {
    nrf_gpio_cfg_sense_input(pin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
}

  uint8_t sd_en;
  (void) sd_softdevice_is_enabled(&sd_en);

  // Enter System OFF state
  if ( sd_en )
  {
    sd_power_system_off();
  }else
  {
    NRF_POWER->SYSTEMOFF = 1;
  }
    }


float readCPUTemperature( void )
{
  uint8_t en;
  int32_t temp;
  (void) sd_softdevice_is_enabled(&en);
  if (en) 
  {
    sd_temp_get(&temp);
}
  else
  {
    NRF_TEMP->EVENTS_DATARDY = 0x00; // Only needed in case another function is also looking at this event flag
    NRF_TEMP->TASKS_START = 0x01; 

    while (!NRF_TEMP->EVENTS_DATARDY);
    temp = NRF_TEMP->TEMP;                      // Per anomaly 29 (unclear whether still applicable), TASKS_STOP will clear the TEMP register.

    NRF_TEMP->TASKS_STOP = 0x01;           // Per anomaly 30 (unclear whether still applicable), the temp peripheral needs to be shut down
    NRF_TEMP->EVENTS_DATARDY = 0x00;
  }
  return temp / 4.0F;
}
#ifdef __cplusplus
}
#endif
