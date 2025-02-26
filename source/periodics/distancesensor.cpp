#include "periodics/distancesensor.hpp"
#include "distancesensor.hpp"

#define g_baseTick 0.0001
#define speed_of_sound 0.0343 // Speed of sound in cm/μs
#define _24_chars 24
// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor distancesensor
    *
    */
    CDistancesensor::CDistancesensor(
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
    {
        m_triggerPin = 0;
    }

    /** @brief  CDistancesensor class destructor
     */
    CDistancesensor::~CDistancesensor()
    {
    }

    void CDistancesensor::serialCallbackDISTANCECommand(char const *a, char *b)
    {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_distance_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
        }else{
            sprintf(b,"syntax error");
        }
    }

    void CDistancesensor::DistanceMeasure()
    {
        // Send 10μs trigger pulse
        m_triggerPin = 1;
        wait_us(10);
        m_triggerPin = 0;

        // Measure echo pulse width
        mbed::Timer timer;
        timer.start();
        while (m_echoPin == 0); // Wait for echo to go high
        timer.reset();
        while (m_echoPin == 1); // Measure duration while echo is high
        uint32_t pulse_duration = timer.elapsed_time().count(); // Microseconds
        timer.stop();

        // Convert to distance in cm
        m_distance = (pulse_duration * speed_of_sound) / 2;
    }

    /* Run method */
    void CDistancesensor::_run()
    {
        /* Run method behaviour */

        if (!m_isActive) return;

        DistanceMeasure();

        char buffer[30];
        snprintf(buffer, sizeof(buffer), "@distance:%d;;\r\n", (int)m_distance);
        m_serial.write(buffer, strlen(buffer));
    }

}; // namespace periodics