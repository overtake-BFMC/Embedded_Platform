#ifndef DISTANCESENSOR_HPP
#define DISTANCESENSOR_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>

namespace periodics
{
   /**
    * @brief Class distancesensor
    *
    */
    class CDistancesensor: public utils::CTask
    {
        public:
            /* Constructor */
            CDistancesensor(
                std::chrono::milliseconds f_period,
                mbed::DigitalOut f_triggerPin,
                mbed::DigitalIn f_echoPin,
                UnbufferedSerial& f_serial
            );
            /* Destructor */
            ~CDistancesensor();

            void serialCallbackDISTANCECommand(char const * a, char * b);
            void DistanceMeasure();
        private:
            /* private variables & method member */
            mbed::DigitalOut m_triggerPin;
            mbed::DigitalIn m_echoPin;
            uint64_t m_period;
            float m_distance;

            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;

            UnbufferedSerial& m_serial;

    }; // class CDistancesensor
}; // namespace periodics

#endif // DISTANCESENSOR_HPP
