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

#include <drivers/steeringmotor.hpp>

#define scaling_factor_1 10
// #define scaling_factor_2 100

namespace drivers
{
    /**
     * @brief It initializes the pwm parameters and it sets the steering in zero position, the limits of the input degree value.
     *
     * @param f_pwm               pin connected to servo motor
     * @param f_inf_limit         inferior limit
     * @param f_sup_limit         superior limit
     *
     */
    CSteeringMotor::CSteeringMotor(
        PinName f_pwm_pin,
        int f_inf_limit,
        int f_sup_limit)
        : m_pwm_pin(f_pwm_pin), m_inf_limit(f_inf_limit), m_sup_limit(f_sup_limit)
    {
        // uint16_t off_pulsewidth = 1000;
        //  Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period);
        m_pwm_pin.pulsewidth_us(1000);
        ThisThread::sleep_for(chrono::milliseconds(10));
        // Set position to zero
        m_pwm_pin.pulsewidth_us(zero_default);
    };

    /** @brief  CSteeringMotor class destructor
     */
    CSteeringMotor::~CSteeringMotor() {
    };

    void CSteeringMotor::configureSteering(uint16_t configure_steer)
    {
        if (min_steer < configure_steer && configure_steer < max_steer)
            m_pwm_pin.pulsewidth_us(configure_steer);
        else
            m_pwm_pin.pulsewidth_us(zero_default);
    };

    int CSteeringMotor::setAngle(int f_angle)
    {
        if (f_angle == 0)
        {
            m_pwm_pin.pulsewidth_us(zero_default);
            return 0;
        }

        const SteerPwmPair *table;
        size_t size;
        bool turnSign = false; //turning left


        if (f_angle < 0)
        {
            table = leftSteerTable;
            size = leftTableSize;
            f_angle = -f_angle;
        }
        else
        {
            table = rightSteerTable;
            size = rightTableSize;
            turnSign = true;
        }

        int low = 0, high = size - 1;

        // Handle edge cases
        if (f_angle <= table[0].returnSteer)
        {
            int pwm;
            if( turnSign )
                pwm = interpolationPoint[1] + interpolationFactor[1] * (int)(f_angle/10);
            else
                pwm = interpolationPoint[0] - interpolationFactor[0] * (int)(f_angle/10);

            m_pwm_pin.pulsewidth_us(pwm);
            printf( "PWM STEER %d\n\r", pwm );
            return turnSign ? f_angle : -f_angle;
        }
        if (f_angle >= table[high].returnSteer)
        {
            m_pwm_pin.pulsewidth_us(table[high].pwm);
            return turnSign ? table[high].returnSteer : -table[high].returnSteer;
        }

        while (low <= high)
        {
            int mid = (low + high) / 2;
            int midDeg = table[mid].returnSteer;

            if (midDeg == f_angle)
            {
                m_pwm_pin.pulsewidth_us(table[mid].pwm);
                return turnSign ? table[mid].returnSteer : -table[mid].returnSteer;
            }
            else if (midDeg < f_angle)
                low = mid + 1;
            else
                high = mid - 1;
        }

        int lowDiff = abs(table[low].returnSteer - f_angle);
        int highDiff = abs(table[high].returnSteer - f_angle);

        int index = (lowDiff < highDiff) ? low : high;

        m_pwm_pin.pulsewidth_us(table[index].pwm);
        return turnSign ? table[index].returnSteer : -table[index].returnSteer;      
    }
    /**
     * @brief It verifies whether a number is in a given range
     *
     * @param f_angle value
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool CSteeringMotor::inRange(int f_angle)
    {
        return m_inf_limit <= f_angle && f_angle <= m_sup_limit;
    };
}; // namespace hardware::drivers