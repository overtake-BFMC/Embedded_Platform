/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

/* The mbed library */
#include <mbed.h>

#include <drivers/mcp2515.hpp>
// Header file for the microseconds functionality
#include <chrono>
/* Header file for the alerts functionality */
#include <periodics/alerts.hpp>
/* Header file for the blinker functionality */
#include <periodics/blinker.hpp>
/* Header file for the IMU functionality */
#include <periodics/imu.hpp>
/* Header file for the klmanager functionality */
#include <brain/klmanager.hpp>
/* Header file for the resourceMonitor functionality */
#include <periodics/resourcemonitor.hpp>
/* Header file for global variables */
#include <brain/globalsv.hpp>
/* Header file for the robot state machine, which deals with the cars movement (steering and speed) */
#include <brain/robotstatemachine.hpp>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/taskmanager.hpp>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/task.hpp>
/* Header file for the Distancesensor functionality */
#include <periodics/distancesensorfront.hpp>
/* Header file for the Distancesensorright functionality */
#include <periodics/distancesensorright.hpp>
/* Header file for the CanBusMonitor functionality */
#include <drivers/canbusmonitor.hpp>
/* Header file for the Canmask functionality */
#include <drivers/canmask.hpp>
/* Header file for the Irsensor functionality */
#include <periodics/irsensor.hpp>

#include "drivers/TIMDMADriver.h"

/* Header file for the Ws2812 functionality */
#include <periodics/ws2812.hpp>
