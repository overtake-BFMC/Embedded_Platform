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

#include "brain/klmanager.hpp"

#define _25_chars 25

// TODO: Add your code here
namespace brain
{
   /**
    * @brief Class constructorklmanager
    *
    */
    CKlmanager::CKlmanager(
        periodics::CAlerts& f_alerts,
        //periodics::CImu& f_imu,
        brain::CRobotStateMachine& f_robotStateMachine,
        periodics::CResourcemonitor& f_resourceM,
        periodics::CDistancesensorFront& f_distanceF,
        periodics::CIRsensor& f_irSensor,
        periodics::CDistancesensorRight& f_distanceR
    )
    : m_klvalue(0)
    , m_alerts(f_alerts)
    //, m_imu(f_imu)
    , m_robotStateMachine(f_robotStateMachine)
    , m_resourceM(f_resourceM)
    , m_distanceF(f_distanceF)
    , m_irSensor(f_irSensor)
    , m_distanceR(f_distanceR)
    {
        /* constructor behaviour */
    }

    /** @brief  CKlmanager class destructor
     */
    CKlmanager::~CKlmanager()
    {
    };

    void CKlmanager::callbackKLcommand(int64_t a, char* b)
    {
        uint32_t l_keyValue = a;

        //(void)sscanf(a,"%hhu",&l_keyValue);

        if(!bool_globalsV_ShuttedDown)
        {
            if(l_keyValue == 15 || l_keyValue == 30 || l_keyValue == 0)
            {
                CKlmanager::m_klvalue = l_keyValue;

                char response[_25_chars];

                if(l_keyValue == 0 && (uint8_globalsV_value_of_kl!=0))
                {
                    sprintf(b,"%d",l_keyValue);
                    //m_imu.callbackIMUcommand(0, response);
                    ThisThread::sleep_for(chrono::milliseconds(50));
                    m_distanceF.callbackDISTANCEFRONTcommand(0, response);
                    ThisThread::sleep_for(chrono::milliseconds(50));
                    m_irSensor.callbackIRSENSORcommand(0, response);
                    ThisThread::sleep_for(chrono::milliseconds(50));
                    m_distanceR.callbackDISTANCERIGHTcommand(0, response);
                    ThisThread::sleep_for(chrono::milliseconds(50));
                    m_robotStateMachine.callbackBRAKEcommand(0, response);
                    ThisThread::sleep_for(chrono::milliseconds(50));
                    m_resourceM.callbackRESOURCEMONcommand(0, response);
                    uint8_globalsV_value_of_kl = 0;
                    m_alerts.alertsCommand("3", response);
                }
                if((l_keyValue == 15 || l_keyValue == 30) && (uint8_globalsV_value_of_kl != 15)) 
                {
                    sprintf(b,"%d",l_keyValue);
                    //m_robotStateMachine.callbackVCDcommand("0;0;2", response);
                    uint8_globalsV_value_of_kl = 15;
                    //if(!bool_globalsV_imu_isActive) m_imu.callbackIMUcommand(1, response);
                    if(!bool_globalsV_resource_isActive) m_resourceM.callbackRESOURCEMONcommand(1, response);
                    if(!bool_globalsV_distanceFront_isActive) m_distanceF.callbackDISTANCEFRONTcommand(1, response);
                    if(!bool_globalsV_irsensor_isActive) m_irSensor.callbackIRSENSORcommand(1, response);
                    if(!bool_globalsV_distanceRight_isActive) m_distanceR.callbackDISTANCERIGHTcommand(1, response);
                    m_alerts.alertsCommand("4", response);
                }
                if(l_keyValue == 30 && (uint8_globalsV_value_of_kl != 30)){
                    sprintf(b,"%d",l_keyValue);
                    uint8_globalsV_value_of_kl = 30;
                    //m_robotStateMachine.callbackVCDcommand("0;0;2", response);
                    m_alerts.alertsCommand("2", response);
                }
            }
            else{
                sprintf(b,"syntax error");
            }
        }
    }

}; // namespace brain