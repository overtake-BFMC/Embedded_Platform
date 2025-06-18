#ifndef DISTANCESENSORRIGHT_HPP
#define DISTANCESENSORRIGHT_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <drivers/mcp2515.hpp>
#include <drivers/canbusmonitor.hpp>

namespace periodics
{
   /**
    * @brief Class distancesensorright
    *
    */
    class CDistancesensorRight: public utils::CTask
    {
        public:
            /* Constructor */
            CDistancesensorRight(
                std::chrono::milliseconds f_period,
                mbed::DigitalOut f_triggerPin,
                mbed::DigitalIn f_echoPin,
                drivers::CCanBusMonitor& f_canBus
            );
            /* Destructor */
            ~CDistancesensorRight();

            void callbackDISTANCERIGHTcommand(int64_t a, char * b);
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

    }; // class CDistancesensorright
}; // namespace periodics

#endif // DISTANCESENSORRIGHT_HPP
