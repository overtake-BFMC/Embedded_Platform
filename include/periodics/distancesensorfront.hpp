#ifndef DISTANCESENSORFRONT_HPP
#define DISTANCESENSORFRONT_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <drivers/mcp2515.hpp>
#include <drivers/canbusmonitor.hpp>

namespace periodics
{
   /**
    * @brief Class distancesensor
    *
    */
    class CDistancesensorFront: public utils::CTask
    {
        public:
            /* Constructor */
            CDistancesensorFront(
                std::chrono::milliseconds f_period,
                mbed::DigitalOut f_triggerPin,
                mbed::DigitalIn f_echoPin,
                drivers::CCanBusMonitor& f_canBus
            );
            /* Destructor */
            ~CDistancesensorFront();

            void callbackDISTANCEFRONTCommand(char const * a, char * b);
            void DistanceMeasure();
        private:
            /* private variables & method member */
            mbed::DigitalOut m_triggerPin;
            mbed::DigitalIn m_echoPin;
            uint32_t m_distance;

            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;

            drivers::CCanBusMonitor& m_canBus;

    }; // class CDistancesensor
}; // namespace periodics

#endif // DISTANCESENSOR_HPP
