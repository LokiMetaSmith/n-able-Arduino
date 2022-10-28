/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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

/*
 * Modified work Copyright (c) 2021 Ryan Powell.  All right reserved.
 */

#define ARDUINO_MAIN
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"

#if defined(CONFIG_MAIN_TASK_STACK_SIZE)
#  define MAIN_TASK_STACK_SIZE CONFIG_MAIN_TASK_STACK_SIZE
#elif defined(DEVICE_RAM_SIZE)
#  if DEVICE_RAM_SIZE < 32
#    define MAIN_TASK_STACK_SIZE 256
#  else
#    define MAIN_TASK_STACK_SIZE 512
#  endif
#else
#  define MAIN_TASK_STACK_SIZE 512
#endif

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

static TaskHandle_t _loopTaskHandle = NULL;
static StackType_t _mainStack[ MAIN_TASK_STACK_SIZE ];
static StaticTask_t _mainTaskBuffer;

void loopTask(void *pvParameters)
{
  (void) pvParameters;

  #ifdef USE_TINYUSB
  Adafruit_TinyUSB_Core_init();
  #endif

#if CFG_DEBUG
  // If Serial is not begin(), call it to avoid hard fault
  if(!Serial) Serial.begin(115200);
#endif

    setup();

#if CFG_DEBUG
  dbgPrintVersion();
  Bluefruit_printInfo();
#endif

  while (1)
  {
        loop();
    yield(); // yield to run other task

    // Serial events
    if (serialEvent && serialEventRun) serialEventRun();
    }
}

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
  init();

  initVariant();



  _loopTaskHandle = xTaskCreateStatic(loopTask, "mlt", MAIN_TASK_STACK_SIZE,
                                      NULL, 1, _mainStack, &_mainTaskBuffer);

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  vTaskStartScheduler();

  NVIC_SystemReset();

  return 0;
}

TaskHandle_t getMainLoopTaskHandle() {
  return _loopTaskHandle;
}
void suspendLoop(void)
{
  vTaskSuspend(_loopHandle);
}

void resumeLoop(void)
{
  if ( isInISR() ) 
  {
    xTaskResumeFromISR(_loopHandle);
  } 
  else
  {
    vTaskResume(_loopHandle);
  }
}

extern "C"
{

// nanolib printf() retarget
// Logger 0: Serial (CDC), 1 Serial1 (UART), 2 Segger RTT
int _write (int fd, const void *buf, size_t count)
{
  (void) fd;

  size_t ret = 0;

#if CFG_LOGGER == 2 || CFG_SYSVIEW
  SEGGER_RTT_Write(0, buf, count);
  ret = count;

#elif CFG_LOGGER == 1
  if ( Serial1 )
  {
    ret = Serial1.write((const uint8_t *) buf, count);
  }

#else
  if ( Serial )
  {
    ret = Serial.write((const uint8_t *) buf, count);
  }
#endif

  return (int) ret;
}

}
