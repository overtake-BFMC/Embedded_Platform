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

#ifndef ROBOT_STATE_MACHINE_HPP
#define ROBOT_STATE_MACHINE_HPP

/* The mbed library */
#include <mbed.h>
/* Header file for the servo motor  */
#include <drivers/speedingmotor.hpp>
/* Header file for the burshless motor  */
#include <drivers/steeringmotor.hpp>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/taskmanager.hpp>

#include <brain/globalsv.hpp>

#include <chrono>

#include <drivers/mcp2515.hpp>

#include <drivers/canbusmonitor.hpp>

namespace brain
{
    /**
     * @brief CRobotStateMachine targets to implement the main state machine to control
     *  movement of robot and provide the interfaces to control functionality, like braking and moving.
     *  The state of robot can change by external signal received from a higher level controller.
     *
     */
    class CRobotStateMachine : public utils::CTask
    {
    public:
        /* Constructor */
        CRobotStateMachine(
            std::chrono::milliseconds f_period,
            drivers::CCanBusMonitor &f_canBus,
            drivers::ISteeringCommand &f_steeringControl,
            drivers::ISpeedingCommand &f_speedingControl);
        /* Destructor */
        ~CRobotStateMachine();
        /* Serial callback method for Speed */
        void callbackSPEEDcommand(int64_t a, char *b);
        /* ccallback method for Steering */
        void callbackSTEERcommand(int64_t a, char *b);
        /* ccallback method for braking */
        void callbackBRAKEcommand(int64_t a, char *b);
        /* ccallback method for vcd */
        void callbackVCDcommand(int64_t a, char *response);

        void callbackMAXTHROTTLEcommand(int64_t a, char *b);
        void callbackMINTHROTTLEcommand(int64_t a, char *b);
        void callbackCONFTHROTTLEcommand(int64_t a, char *b);

        void callbackCONFSTEERcommand(int64_t a, char *b);

    private:
        /* Contains the state machine, which control the lower level drivers (motor and steering) based the current state. */
        virtual void _run();
        /* reference to Serial object */
        drivers::CCanBusMonitor &m_canBus;
        /* Steering wheel control interface */
        drivers::ISteeringCommand &m_steeringControl;
        /* Steering wheel control interface */
        drivers::ISpeedingCommand &m_speedingControl;
        /* State machine state */
        //uint8_t m_state;

        uint16_t m_ticksRun;

        uint16_t m_targetTime;

        uint16_t m_period;

        int m_speed;
        int m_steering;

        int m_pwmConfigure;

        
        int m_pwmConfigureSteer;

        bool steerSignP;
        bool steerSignN;

        bool m_speedActivate;
        bool m_steerActivate;
        bool m_brakeActivate;
        bool m_vcdActivate;
        bool m_maxThrottleActivate;
        bool m_minThrottleActivate;
        bool m_configureThrottleActivate;
        bool m_configureSteerActivate;


    }; // class CRobotStateMachine
}; // namespace brain

#endif // ROBOT_STATE_MACHINE_HPP
