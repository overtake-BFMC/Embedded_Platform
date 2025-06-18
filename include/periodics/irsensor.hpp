#ifndef IRSENSOR_HPP
#define IRSENSOR_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <drivers/mcp2515.hpp>
#include <drivers/canbusmonitor.hpp>

namespace periodics
{
   /**
    * @brief Class irsensor
    *
    */
    class CIRsensor: public utils::CTask
    {
        public:
            /* Constructor */
            CIRsensor(
                std::chrono::milliseconds f_period,
                mbed::DigitalIn f_pin,
                drivers::CCanBusMonitor& f_canBus
            );
            /* Destructor */
            ~CIRsensor();

            void callbackIRSENSORcommand(int64_t a, char * b);
        private:
            /* private variables & method member */
            drivers::CCanBusMonitor& m_canBus;
            mbed::DigitalIn m_pin;
            uint8_t m_value;

            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;
    }; // class CIrsensor
}; // namespace periodics

#endif // IRSENSOR_HPP
