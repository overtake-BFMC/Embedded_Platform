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

#include <brain/robotstatemachine.hpp>

#define scale_ds_to_ms 100

namespace brain
{

    /**
     * @brief CRobotStateMachine Class constructor
     *
     * @param f_period              period for controller execution in seconds
     * @param f_serialPort          reference to serial communication object
     * @param f_steeringControl     reference to steering motor control interface
     * @param f_speedingControl     reference to brushless motor control interface
     */
    CRobotStateMachine::CRobotStateMachine(
        std::chrono::milliseconds f_period,
        drivers::CCanBusMonitor &f_canBus,
        drivers::ISteeringCommand &f_steeringControl,
        drivers::ISpeedingCommand &f_speedingControl)
        : utils::CTask(f_period), m_canBus(f_canBus), m_steeringControl(f_steeringControl), m_speedingControl(f_speedingControl), m_ticksRun(0), m_targetTime(0), m_period((uint16_t)(f_period.count())), m_speed(0), m_steering(0), steerSignP(false), steerSignN(false), m_speedActivate(false), m_steerActivate(false), m_brakeActivate(false), m_vcdActivate(false), m_maxThrottleActivate(false), m_minThrottleActivate(false), m_configureThrottleActivate(false), m_configureSteerActivate(false)
    {
    }

    /** @brief  CRobotStateMachine class destructor
     */
    CRobotStateMachine::~CRobotStateMachine() {
    };

    /** \brief  _Run method contains the main application logic, where it controls the lower lever drivers (dc motor and steering) based the given command and state.
     * It has three main states:
     *  - 1 - move state -> control the motor rotation speed by giving a speed reference, which is then converted to PWM
     *  - 2 - steering state -> trigger the steering of the motor
     *  - 3 - Brake state -> make the motor enter into a brake state.
     *  - 4 - State responsible for configuring the vehicle's speed and steering over a specified duration.
     */
    void CRobotStateMachine::_run()
    {
        if (m_configureSteerActivate)
        {
            printf("pwm steer in run method %d\n\r", m_pwmConfigureSteer);
            m_steeringControl.configureSteering(m_pwmConfigureSteer);

            m_canBus.sendMessage(0x96, m_pwmConfigureSteer, CANStandard, CANData, 4);
            m_configureSteerActivate = false;
        }

        if (m_configureThrottleActivate)
        {
            m_speedingControl.configureThrottle(m_pwmConfigure);

            m_canBus.sendMessage(0x95, m_pwmConfigure, CANStandard, CANData, 4);
            m_configureThrottleActivate = false;
        }

        if (m_maxThrottleActivate)
        {
            m_speedingControl.maxThrottle();
            m_maxThrottleActivate = false;
        }

        if (m_minThrottleActivate)
        {
            m_speedingControl.minThrottle();
            m_minThrottleActivate = false;
        }

        // speed state - control the dc motor rotation speed and the steering angle.
        if (m_speedActivate)
        {
            int returnSpeed = m_speedingControl.setSpeed(m_speed); // Set the reference speed

            m_canBus.sendMessage(0x123, returnSpeed, CANStandard, CANData, 4);
            m_speedActivate = false;
        }

        // Steering state
        if (m_steerActivate)
        {
            int returnSteer = m_steeringControl.setAngle(m_steering); // control the steering angle
            printf("return steer %d\n\r", returnSteer);

            if (m_steering > 0)
                steerSignP = true;
            if (m_steering < 0)
                steerSignN = true;

            if (m_steering == 0 && steerSignN)
            {
                steerSignN = false;
                m_steeringControl.setAngle(20);
            }
            if (m_steering == 0 && steerSignP)
            {
                steerSignP = false;
                m_steeringControl.setAngle(-20);
            }

            m_canBus.sendMessage(0x128, returnSteer, CANStandard, CANData, 4);
            m_steerActivate = false;
        }

        // State responsible for configuring the vehicle's speed and steering over a specified duration.
        if (m_vcdActivate)
        {
            // If the accumulated ticks exceed the target time, stop the movement and deactivate the task.
            if (m_ticksRun >= m_targetTime + m_period)
            {

                m_speedingControl.setSpeed(0);
                m_steeringControl.setAngle(0);

                m_vcdActivate = false;

                m_canBus.sendMessage(0x146, m_ticksRun, CANStandard, CANData, 2);
            }
            else
            {
                // Otherwise, increment the tick counter.
                m_ticksRun += m_period;
            }
        }

        // Brake state
        if (m_brakeActivate)
        {
            m_steeringControl.setAngle(m_steering); // control the steering angle
            m_speedingControl.setBrake();

            m_canBus.sendMessage(0x141, 1, CANStandard, CANData, 1); // 1 == true

            m_brakeActivate = false;
        }
    }

    void CRobotStateMachine::callbackCONFSTEERcommand(int64_t a, char *b)
    {

        int pwmSteer = a;
        // uint32_t l_res = sscanf(a, "%d", &pwmSteer);

        //printf("pwm steer in callback %d\n\r", pwmSteer);

        if (uint8_globalsV_value_of_kl == 30)
        {
            m_configureSteerActivate = true;
            m_pwmConfigureSteer = pwmSteer;
        }
        else
        {
            sprintf(b, "kl 30 is required!!");
        }
    }

    void CRobotStateMachine::callbackCONFTHROTTLEcommand(int64_t a, char *b)
    {
        int pwm = a;

        if (uint8_globalsV_value_of_kl == 30)
        {
            m_configureThrottleActivate = true;
            m_pwmConfigure = pwm;
        }
        else
        {
            sprintf(b, "kl 30 is required!!");
        }
    }

    void CRobotStateMachine::callbackMAXTHROTTLEcommand(int64_t a, char *b)
    {
        bool l_res = a;

        if (l_res)
        {
            if (uint8_globalsV_value_of_kl == 30)
            {
                m_maxThrottleActivate = true;
            }
            else
            {
                sprintf(b, "kl 30 is required!!");
            }
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    void CRobotStateMachine::callbackMINTHROTTLEcommand(int64_t a, char *b)
    {
        bool l_res = a;

        if (l_res)
        {
            if (uint8_globalsV_value_of_kl == 30)
            {
                m_minThrottleActivate = true;
            }
            else
            {
                sprintf(b, "kl 30 is required!!");
            }
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    /** \brief  Serial callback method for speed command
     *
     * Serial callback method setting controller to value received for dc motor control values.
     * In the case of pid activated, the dc motor control values has to be express in meter per second, otherwise represent the duty cycle of PWM signal in percent.
     * The steering angle has to express in degree, where the positive values marks the right direction and the negative values noticed the left turning direction.
     *
     * @param a                   string to read data
     * @param b                   string to write data
     *
     */
    void CRobotStateMachine::callbackSPEEDcommand(int64_t a, char *b)
    {
        int l_speed = a;

        if (uint8_globalsV_value_of_kl == 30)
        {
            if (!m_speedingControl.inRange(l_speed))
            { // Check the received reference speed is within range
                sprintf(b, "The reference speed command is too high");
                return;
            }

            // m_state = 1;
            m_speedActivate = true;

            m_speed = l_speed;
        }
        else
        {
            sprintf(b, "kl 30 is required!!");
        }
    }

    /** \brief  Serial callback method for steering command
     *
     * Serial callback method setting controller to value received for steering angle.
     * The steering angle has to express in degree, where the positive values marks the right direction and the negative values noticed the left turning direction.
     *
     * @param a                   string to read data
     * @param b                   string to write data
     *
     */
    void CRobotStateMachine::callbackSTEERcommand(int64_t a, char *b)
    {
        int l_angle = a;

        if (uint8_globalsV_value_of_kl == 30)
        {
            if (!m_steeringControl.inRange(l_angle))
            { // Check the received steering angle
                sprintf(b, "The steering angle command is too high");
                return;
            }

            // m_state = 2;

            m_steerActivate = true;

            m_steering = l_angle;
        }
        else
        {
            sprintf(b, "kl 30 is required!!");
        }
    }

    /** \brief  Serial callback actions for brake command
     *
     * This method aims to change the state of controller to brake and sets the steering angle to the received value.
     *
     * @param a                   string to read data
     * @param b                   string to write data
     *
     */
    void CRobotStateMachine::callbackBRAKEcommand(int64_t a, char *b)
    {
        int l_angle = a;

        if (!m_steeringControl.inRange(l_angle))
        {
            sprintf(b, "The steering angle command is too high");
            return;
        }

        // m_state = 3;
        m_brakeActivate = true;

        m_steering = l_angle;
    }

    /** \brief  Serial callback actions for brake command
     *
     * This method aims to change the state of controller to brake and sets the steering angle to the received value.
     *
     * @param a                   string to read data
     * @param b                   string to write data
     *
     */
    void CRobotStateMachine::callbackVCDcommand(int64_t message, char *response)
    {
        int speed, steer;
        uint16_t time_deciseconds = (message & 0xFF) | ((message >> 8) & 0xFF);
        speed = ((message >> 16) & 0xFF) | ((message >> 24) & 0xFF);
        steer = ((message >> 32) & 0xFF) | ((message >> 40) & 0xFF);

        // uint8_t parsed = sscanf(message, "%d;%d;%hhu", &speed, &steer, &time_deciseconds);

        if (uint8_globalsV_value_of_kl != 30)
        {
            sprintf(response, "kl 30 is required!!");
            return;
        }

        m_targetTime = time_deciseconds;

        if (time_deciseconds > 0 && speed <= 500 && speed >= -500 && steer <= 232 && steer >= -232)
        {
            sprintf(response, "%d;%d;%d", speed, steer, time_deciseconds);

            m_ticksRun = 0;

            m_targetTime = time_deciseconds * scale_ds_to_ms;

            // m_state = 4;
            m_vcdActivate = true;

            m_steeringControl.setAngle(steer);
            m_speedingControl.setSpeed(-speed);
        }
        else
        {
            sprintf(response, "something went wrong");
        }
    }
}; // namespace brain