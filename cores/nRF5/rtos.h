/*
  Copyright (c) 2021 Ryan Powell.  All right reserved.

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
/**************************************************************************/
#ifndef RTOS_H_
#define RTOS_H_

#ifndef NABLERTOS_H
#define NABLERTOS_H
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define DELAY_FOREVER   portMAX_DELAY

enum
{
  TASK_PRIO_LOWEST  = 0, // Idle task, should not be used
  TASK_PRIO_LOW     = 1, // Loop
  TASK_PRIO_NORMAL  = 2, // Timer Task, Callback Task
  TASK_PRIO_HIGH    = 3, // Bluefruit Task
  TASK_PRIO_HIGHEST = 4,
};

#define ms2tick       pdMS_TO_TICKS
#define tick2ms(tck)  ( ( ((uint64_t)(tck)) * 1000) / configTICK_RATE_HZ )
#define tick2us(tck)  ( ( ((uint64_t)(tck)) * 1000000) / configTICK_RATE_HZ )

// legacy thread-safe malloc/free
#define rtos_malloc   malloc
#define rtos_free     free

// Visible only with C++
#ifdef __cplusplus

#define SCHEDULER_STACK_SIZE_DFLT   (512*2)

class nableRtos {
public:
    nableRtos() {}
    ~nableRtos() {}
    uint32_t getIsrStackHwm();
    uint32_t getMainTaskHwm();
    uint32_t getFreeHeap();
    uint32_t getIdleTaskHwm();
    uint32_t getTimerTaskHwm();
    uint32_t getBleHostTaskHwm();
    uint32_t getBleLLTaskHwm();
};

extern nableRtos RTOS;
#endif // __cplusplus

#endif /* RTOS_H_ */
