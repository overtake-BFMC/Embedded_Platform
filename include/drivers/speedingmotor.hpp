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

/* Include guard */
#ifndef SPEEDINGMOTOR_HPP
#define SPEEDINGMOTOR_HPP

/* The mbed library */
#include <mbed.h>
#include <map>

struct SpeedPwmPair 
{
    int returnSpeed;
    int pwm;    // in microseconds
};

namespace drivers
{
    /**
     * @brief Interface to control the brushless motor.
     * 
     */
    class ISpeedingCommand
    {
        public:
            virtual int setSpeed(int) = 0 ;
            virtual bool inRange(int) = 0 ;
            virtual void setBrake() = 0 ;
            virtual void maxThrottle() = 0;
            virtual void minThrottle() = 0;
            virtual void configureThrottle(uint16_t) = 0;
    };

    /**  
     * @brief Speeding motor driver
     * 
     * It is used to control the Brushless motor (more precisely the ESC), which is connected to driving shaft. The reference speed can be accessed through 'setSpeed' method. 
     * 
     */
    class CSpeedingMotor: public ISpeedingCommand
    {
        public:
            /* Constructor */
            CSpeedingMotor(
                PinName     f_pwm_pin,
                int       f_inf_limit,
                int       f_sup_limit
            );
            /* Destructor */
            ~CSpeedingMotor();
            /* Set speed */
            int setSpeed(int); 
            /* Check speed is in range */
            bool inRange(int);
            /* Set brake */
            void setBrake(); 

            void maxThrottle();
            void minThrottle();
            void configureThrottle(uint16_t);

        private:
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /** @brief 0 default */
            uint16_t zero_default = 1500; // 1491; //0.074568(7.4% duty cycle) * 20000µs(ms_period)
            /** @brief 0 default */
            uint8_t ms_period = 20; // 20000µs
            /** @brief step_value */
            //int16_t step_value = 102;  // 0.00051 * 20000µs(ms_period) * 10(scale factor)
            /** @brief Inferior limit */
            const int m_inf_limit;
            /** @brief Superior limit */
            const int m_sup_limit;

            //real max throttle 500
            //real min throttle 2500

            uint16_t max_throttle = 1660;
            uint16_t min_throttle = 1330;

            const SpeedPwmPair forwardTable[10] = 
            {
                { 48, 1405 },// { 70, 73, 1398}, { 90, 93, 1394}, { 95, 96, 1393},
                { 101, 1392},//{ 140, 143, 1383}, 
                { 153, 1382},// { 190, 191, 1375}, { 195, 196, 1374},
                { 201, 1373},
                { 248, 1366},// { 255, 255, 1365}, { 280, 281, 1361},
                { 302, 1358},// { 340, 344, 1352},
                { 355, 1351},// { 390, 396, 1345},
                { 406, 1344},
                { 450, 1339},// { 485, 488, 1335},{ 490, 492, 1334},
                { 498, 1333}// { 515, 516, 1332}, { 520, 522, 1332}
            };

            // Reverse table
            const SpeedPwmPair reverseTable[10] = 
            {
                //{  45, 46, 1550}, {  48, 1551}, {  49, 1552},{  49, 1560}, 
                { 50, 1570},//{  60, 65, 1580}, { 95, 98, 1589}, 
                { 101, 1590}, 
                { 147, 1598}, //{ 155, 155, 1601},
                { 199, 1608}, //{ 210, 208, 1610}, { 240, 239, 1616}, { 245, 1617}, 
                { 251, 1616},// { 340, 345, 1624}, 
                { 299, 1623}, //{ 355, 354, 1632}, { 380, 380, 1636}, 
                { 352, 1630},//{ 440, 442, 1644}, 
                { 400, 1637}, 
                { 448, 1643},
                { 500, 1649}//{ 535, 538, 1655}, { 585, 586, 1659}
            };

            int interpolatePwm(int, const SpeedPwmPair*, int);
            int pwmFromSpeed(int);
    }; // class CSpeedingMotor
}; // namespace drivers

#endif// SPEEDINGMOTOR_HPP