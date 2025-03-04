#include "periodics/distancesensorright.hpp"

#define speed_of_sound 0.0343   // Speed of sound in cm/μs
#define _18_chars 18

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor distancesensorright
    *
    */
    CDistancesensorRight::CDistancesensorRight(
        std::chrono::milliseconds f_period,
        mbed::DigitalOut f_triggerPin,
        mbed::DigitalIn f_echoPin,
        UnbufferedSerial& f_serial
    )
    : utils::CTask(f_period)
    , m_triggerPin(f_triggerPin)
    , m_echoPin(f_echoPin)
    , m_period(f_period.count())
    , m_isActive(false)
    , m_serial(f_serial)
    , m_isConnected(true)
    {
        m_triggerPin = 0;
    }

    /** @brief  CDistancesensorright class destructor
     */
    CDistancesensorRight::~CDistancesensorRight()
    {
    }

    void CDistancesensorRight::serialCallbackDISTANCERIGHTCommand(char const *a, char *b)
    {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_distanceRight_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
        }else{
            sprintf(b,"syntax error");
        }
    }

    bool CDistancesensorRight::isSensorConnected()
    {
        m_triggerPin = 1;
        wait_us(10);
        m_triggerPin = 0;

        mbed::Timer timer;
        timer.start();
        
        // 50ms timeout
        const uint32_t timeout_us = 50000; 

        while (m_echoPin == 0) 
            if (timer.elapsed_time().count() > timeout_us)
            {
                timer.stop();
                return false;
            }
        
        timer.stop();
        return true;
    }

    void CDistancesensorRight::DistanceMeasure()
    {
        m_triggerPin = 1;
        wait_us(10);
        m_triggerPin = 0;

        mbed::Timer timer;
        timer.start();
        while (m_echoPin == 0); 
        timer.reset();
        while (m_echoPin == 1); 
        uint32_t pulse_duration = timer.elapsed_time().count(); 
        timer.stop();

        // Convert to distance in cm
        m_distance = (pulse_duration * speed_of_sound) / 2;

        if( m_distance > 99 )
            m_distance = 99;
    }
    /* Run method */
    void CDistancesensorRight::_run()
    {
        /* Run method behaviour */
        if(!m_isActive && !m_isConnected) 
            return;

        if (!isSensorConnected()) {
            //m_serial.write("@distanceF:DISCONNECTED;;\r\n", strlen("@distanceF:DISCONNECTED;;\r\n"));
            m_isConnected = false;
            return;  // Skip measurement if sensor is disconnected
        }

        DistanceMeasure();

        char buffer[_18_chars];
        
        snprintf(buffer, sizeof(buffer), "@distanceR:%d;;\r\n", (int)m_distance);

        m_serial.write(buffer, strlen(buffer));
    }

}; // namespace periodics