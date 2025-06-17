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

    /**
     * @brief Interpolates values based on steering input.
     *
     * This function interpolates `stepValues` and `zeroDefaultValues` based on the provided `steering` input.
     * The interpolation is made using `steeringValueP` and `steeringValueN` as reference values.
     *
     * @param steering The input steering value for which the values need to be interpolated.
     * @param steeringValueP Positive reference values for steering.
     * @param steeringValueN Negative reference values for steering.
     * @param stepValues Step values corresponding to steeringValueP and steeringValueN which need to be interpolated.
     * @param zeroDefaultValues Zero default values corresponding to steeringValueP and steeringValueN for interpolation.
     * @param size The size of the arrays.
     * @return A pair of interpolated values: { interpolated stepValue, interpolated zeroDefaultValue }.
     */
    // std::pair<int, int> CSteeringMotor::interpolate(int steering, const int steeringValueP[], const int steeringValueN[], const int stepValues[], const int zeroDefaultValues[], int size)
    // {
    //     // If steering is within the bounds of the first positive and negative reference values
    //     if(steering <= steeringValueP[0]){
    //         if (steering >= steeringValueN[0])
    //         {
    //             return {stepValues[0], zeroDefaultValues[0]};
    //         }
    //         else{
    //             for(uint8_t i=1; i<size; i++)
    //             {
    //                 // Find the interval of negative reference values where steering falls into
    //                 if (steering >= steeringValueN[i])
    //                 {
    //                     // Calculate slopes for interpolation
    //                     int slopeStepValue = (stepValues[i] - stepValues[i-1]) / (steeringValueN[i] - steeringValueN[i-1]);
    //                     int slopeZeroDefault = (zeroDefaultValues[i] - zeroDefaultValues[i-1]) / (steeringValueN[i] - steeringValueN[i-1]);

    //                     // Return the interpolated values
    //                     return {stepValues[i-1] + slopeStepValue * (steering - steeringValueN[i-1]), zeroDefaultValues[i-1] + slopeZeroDefault * (steering - steeringValueN[i-1])};
    //                 }
    //             }
    //         }
    //     }

    //     // Boundary conditions for positive and negative reference values
    //     if(steering >= steeringValueP[size-1]) return {stepValues[size-1], zeroDefaultValues[size-1]};
    //     if(steering <= steeringValueN[size-1]) return {stepValues[size-1], zeroDefaultValues[size-1]};

    //     // Interpolation for values between positive reference values
    //     for(uint8_t i=1; i<size; i++)
    //     {
    //         if (steering <= steeringValueP[i])
    //         {
    //             // Calculate slopes for interpolation
    //             int slopeStepValue = (stepValues[i] - stepValues[i-1]) / (steeringValueP[i] - steeringValueP[i-1]);
    //             int slopeZeroDefault = (zeroDefaultValues[i] - zeroDefaultValues[i-1]) / (steeringValueP[i] - steeringValueP[i-1]);

    //             // Return the interpolated values
    //             return {stepValues[i-1] + slopeStepValue * (steering - steeringValueP[i-1]), zeroDefaultValues[i-1] + slopeZeroDefault * (steering - steeringValueP[i-1])};
    //         }
    //     }

    //     // Default return if no interval is found
    //     return {-1, -1};
    // };

    /** @brief  It modifies the angle of the servo motor, which controls the steering wheels.
     *
     *  @param f_angle      angle degree, where the positive value means right direction and negative value the left direction.
     */
    // void CSteeringMotor::setAngle(int f_angle)
    // {
    //     std::pair<int, int> interpolationResult;

    //     interpolationResult = interpolate(f_angle, steeringValueP, steeringValueN, stepValues, zeroDefaultValues, 3);

    //     step_value = interpolationResult.first;
    //     zero_default = interpolationResult.second;

    //     m_pwm_pin.pulsewidth_us(conversion(f_angle));
    // };

    // /** @brief  It converts angle degree to duty cycle for pwm signal.
    //  *
    //  *  @param f_angle    angle degree
    //  *  \return         duty cycle in interval [0,1]
    //  */
    // int CSteeringMotor::conversion(int f_angle)
    // {
    //     return (((step_value * f_angle)/(scaling_factor_1*scaling_factor_2)) + (zero_default/scaling_factor_2));
    // };

    //
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
            m_pwm_pin.pulsewidth_us(table[0].pwm);
            return turnSign ? table[0].returnSteer : -table[0].returnSteer;
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