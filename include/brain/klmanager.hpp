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

#ifndef KLMANAGER_HPP
#define KLMANAGER_HPP

// TODO: Add your code here

/* The mbed library */
#include <mbed.h>
#include <periodics/alerts.hpp>
#include <periodics/imu.hpp>
#include <brain/robotstatemachine.hpp>
#include <periodics/resourcemonitor.hpp>
#include <periodics/distancesensorfront.hpp>
#include <periodics/irsensor.hpp>
#include <periodics/distancesensorright.hpp>
#include <brain/globalsv.hpp>

namespace brain
{
   /**
    * @brief Class klmanager
    *
    */
    class CKlmanager
    {
        public:
            /* Construnctor */
            CKlmanager(
                periodics::CAlerts& f_alerts,
                //periodics::CImu& f_imu,
                brain::CRobotStateMachine& f_robotStateMachine,
                periodics::CResourcemonitor& f_resourceM,
                periodics::CDistancesensorFront& f_distanceF,
                periodics::CIRsensor& f_irSensor,
                periodics::CDistancesensorRight& f_distanceR   
            );
            /* Destructor */
            ~CKlmanager();

            void callbackKLcommand(int64_t message, char* response);

            uint8_t m_klvalue;

        private:
            /* private variables & method member */
            periodics::CAlerts& m_alerts;
            //periodics::CImu& m_imu;
            brain::CRobotStateMachine& m_robotStateMachine;
            periodics::CResourcemonitor& m_resourceM;
            periodics::CDistancesensorFront& m_distanceF;
            periodics::CIRsensor& m_irSensor;
            periodics::CDistancesensorRight& m_distanceR;
    }; // class CKlmanager
}; // namespace brain

#endif // KLMANAGER_HPP
