#include "periodics/distancesensorfront.hpp"
// #include "distancesensor.hpp"

#define speed_of_sound 0.0343 // Speed of sound in cm/μs

namespace periodics
{
    /**
     * @brief Class constructor distancesensor
     *
     */
    CDistancesensorFront::CDistancesensorFront(
        std::chrono::milliseconds f_period,
        mbed::DigitalOut f_triggerPin,
        mbed::DigitalIn f_echoPin,
        drivers::CCanBusMonitor &f_canBus)
        : utils::CTask(f_period), m_triggerPin(f_triggerPin), m_echoPin(f_echoPin), m_canBus(f_canBus), m_isActive(false)
    {
        m_triggerPin = 0;
    }

    /** @brief  CDistancesensor class destructor
     */
    CDistancesensorFront::~CDistancesensorFront()
    {
    }

    void CDistancesensorFront::callbackDISTANCEFRONTcommand(int64_t a, char *b)
    {
        uint8_t l_isActivate = a;

        if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
        {
            m_isActive = (l_isActivate >= 1);
            bool_globalsV_distanceFront_isActive = (l_isActivate >= 1);
            sprintf(b, "1");
        }
        else
        {
            sprintf(b, "kl 15/30 is required!!");
        }
    }

    void CDistancesensorFront::DistanceMeasure()
    {
        m_triggerPin = 1;
        wait_us(10);
        m_triggerPin = 0;

        mbed::Timer timer;
        timer.start();

        while (m_echoPin == 0)
            ;
        timer.reset();
        while (m_echoPin == 1)
            ;
        uint32_t pulse_duration = timer.elapsed_time().count();
        timer.stop();

        // Convert to distance in cm
        m_distance = (pulse_duration * speed_of_sound) / 2;

        if (m_distance > 99)
            m_distance = 99;
    }

    /* Run method */
    void CDistancesensorFront::_run()
    {
        /* Run method behaviour */

        if (!m_isActive)
            return;

        DistanceMeasure();

        m_canBus.sendMessage(0x11E, m_distance, CANStandard, CANData, 4);
    }

}; // namespace periodics