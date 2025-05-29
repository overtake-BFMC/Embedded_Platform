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

#include <drivers/speedingmotor.hpp>

namespace drivers{
    /**
     * @brief It initializes the pwm parameters and it sets the speed reference to zero position, and the limits of the car speed.
     * 
     * @param f_pwm_pin               pin connected to servo motor
     * @param f_inf_limit         inferior limit 
     * @param f_sup_limit         superior limit
     * 
     */
    CSpeedingMotor::CSpeedingMotor(
            PinName f_pwm_pin, 
            int f_inf_limit, 
            int f_sup_limit
        )
        : m_pwm_pin(f_pwm_pin)
        , m_inf_limit(f_inf_limit)
        , m_sup_limit(f_sup_limit)
    {
        // Set the ms_period on the pwm_pin
        m_pwm_pin.period_ms(ms_period); 
        // Set position to zero
        m_pwm_pin.pulsewidth_us(zero_default);
    };

    /** @brief  CSpeedingMotor class destructor
     */
    CSpeedingMotor::~CSpeedingMotor()
    {
    };

    int CSpeedingMotor::interpolatePwm(int speed, const SpeedPwmPair* table, int size) 
    {
        if (speed <= table[0].returnSpeed)
            return table[0].pwm;
        if (speed >= table[size - 1].returnSpeed)
            return table[size - 1].pwm;

        for (int i = 1; i < size; ++i) 
        {
            if (speed <= table[i].returnSpeed) 
            {
                int s1 = table[i - 1].returnSpeed;
                int s2 = table[i].returnSpeed;
                int pwm1 = table[i - 1].pwm;
                int pwm2 = table[i].pwm;
                return pwm1 + ((pwm2 - pwm1) * (speed - s1)) / (s2 - s1);
            }
        }

        return table[size - 1].pwm;
    }

    int CSpeedingMotor::pwmFromSpeed(int speed) 
    {
        if (speed > 0)
            return interpolatePwm(speed, forwardTable, sizeof(forwardTable)/sizeof(SpeedPwmPair));
        else
            return interpolatePwm(speed, reverseTable, sizeof(reverseTable)/sizeof(SpeedPwmPair));
    }

    /** @brief  It modifies the speed reference of the brushless motor, which controls the speed of the wheels. 
     *
     *  @param f_speed      speed in m/s, where the positive value means forward direction and negative value the backward direction. 
     */
    int CSpeedingMotor::setSpeed(int f_speed)
    {
        if ( !f_speed  )
        {
            m_pwm_pin.pulsewidth_us(zero_default);
            return 0;
        }

        int absSpeed = f_speed > 0 ? f_speed : -f_speed;

        if( absSpeed% 50 == 0)
        {
            int index = (absSpeed/ 50) - 1;

            if( f_speed > 0)
            {
                //printf( "NAPRED %d", f_speed );
                m_pwm_pin.pulsewidth_us(forwardTable[index].pwm);
                return forwardTable[index].returnSpeed;
            }
            else
            {
                //printf( "UNAZAD %d", f_speed );
                m_pwm_pin.pulsewidth_us(reverseTable[index].pwm);
                return -reverseTable[index].returnSpeed;
            }
        }

        m_pwm_pin.pulsewidth_us(pwmFromSpeed(absSpeed));
        return f_speed;          
    };

    /** @brief  It puts the brushless motor into brake state, 
     */
    void CSpeedingMotor::setBrake()
    {
        m_pwm_pin.pulsewidth_us(zero_default);
    };

    void CSpeedingMotor::maxThrottle()
    {
        m_pwm_pin.pulsewidth_us(min_throttle);
    };

    void CSpeedingMotor::minThrottle()
    {
        m_pwm_pin.pulsewidth_us(max_throttle);
    };

    void CSpeedingMotor::configureThrottle(uint16_t configure_throttle)
    {
        if( min_throttle < configure_throttle && configure_throttle < max_throttle)
            m_pwm_pin.pulsewidth_us(configure_throttle);
        else
            m_pwm_pin.pulsewidth_us(zero_default);
    };

    /**
     * @brief It verifies whether a number is in the given treshold
     * 
     * @param f_speed value 
     * @return true means, that the value is in the range
     * @return false means, that the value isn't in the range
     */
    bool CSpeedingMotor::inRange(int f_speed){
        return (m_inf_limit<=f_speed) && (f_speed<=m_sup_limit);
    };
}; // namespace hardware::drivers