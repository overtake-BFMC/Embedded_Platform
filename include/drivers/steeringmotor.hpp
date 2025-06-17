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
#ifndef STEERINGMOTOR_HPP
#define STEERINGMOTOR_HPP

#include <mbed.h>
#include <utility>
#include <cmath>
#include <climits>
#include <chrono>

struct SteerPwmPair 
{
    int returnSteer;
    int pwm;    // in microseconds
};

const size_t rightTableSize = 16;
const size_t leftTableSize = 21;

namespace drivers
{
    /**
     * @brief Interface to control the steering angle
     * 
     */
    class ISteeringCommand
    {
        public:
            virtual int setAngle(int f_angle) = 0 ;
            virtual bool inRange(int f_angle) = 0 ;
            virtual void configureSteering(uint16_t configure_steer) = 0 ;
    };


    /**  
     * @brief Steering servo motor driver
     * 
     * It is used to control the servo motor, which is connected to the steering wheels. The steering angle can be accessed through 'setAngle' method. 
     * 
     */
    class CSteeringMotor: public ISteeringCommand
    {
        public:
            /* Constructor */
            CSteeringMotor(
                PinName f_pwm_pin,
                int f_inf_limit,
                int f_sup_limit
            );
            /* Destructor */
            ~CSteeringMotor();
            /* Set angle */
            int setAngle(int f_angle); 
            /* Check if angle in range */
            bool inRange(int f_angle);

            void configureSteering(uint16_t configure_steer);
        private:
            /** @brief PWM output pin */
            PwmOut m_pwm_pin;
            /** @brief 0 default */
            int zero_default = 1524;
            //int zero_default = 153441; //0.07672070(7.6% duty cycle) * 20000µs(ms_period) * 100(scale factor)
            /** @brief ms_period */
            int8_t ms_period = 20; // 20000µs
            /** @brief step_value */
            //int step_value = 1901; // 0.0009505 * 20000µs(ms_period) * 100(scale factor)
            /** @brief Inferior limit */
            const int m_inf_limit;
            /** @brief Superior limit */
            const int m_sup_limit;

            const uint16_t min_steer = 899;
            const uint16_t max_steer = 2101;

            /* convert angle degree to duty cycle for pwm signal */
            //int conversion(int f_angle); //angle to duty cycle

            /* interpolate the step value and the zero default based on the steering value */
            //std::pair<int, int> interpolate(int steering, const int steeringValueP[], const int steeringValueN[], const int stepValues[], const int zeroDefaultValues[], int size);

            // Predefined values for steering reference and interpolation
            // All the values have a scale factor applied (*100)
            // const int steeringValueP[3] = {0, 150, 200};
            // const int steeringValueN[3] = {0, -150, -200};
            // const int stepValues[3] = {1901, 1951, 2028};
            // const int zeroDefaultValues[3] = {153441, 153820, 153610};

            const SteerPwmPair rightSteerTable[rightTableSize] = 
            {
                { 50, 1625},
                {104, 1725},
                {111, 1735},
                {130, 1775},
                {150, 1795},
                {162, 1835},
                {170, 1855},
                {179, 1875},
                {182, 1885},
                {191, 1905},
                {199, 1925},
                {204, 1935},
                {240, 2015},
                {251, 2045},
                {259, 2065},
                {270, 2085}
            };

            const SteerPwmPair leftSteerTable[leftTableSize] = 
            {
                { 52, 1355},
                { 60, 1335},
                { 70, 1315},
                { 79, 1295},
                { 89, 1275},
                {101, 1255},
                {121, 1225},
                {129, 1215},
                {140, 1195},
                {150, 1175},
                {160, 1155},
                {171, 1135},
                {189, 1105},
                {199, 1085},
                {209, 1065},
                {221, 1035},
                {230, 1015},
                {239, 1005},
                {241,  995},
                {251,  985},
                {271,  945}
            };
    }; // class ISteeringCommand
}; // namespace drivers


#endif //STEERINGMOTOR_HPP