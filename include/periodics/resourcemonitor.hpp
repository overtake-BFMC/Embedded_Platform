#ifndef RESOURCEMONITOR_HPP
#define RESOURCEMONITOR_HPP

// TODO: Add your code here

#include <mbed.h>
#include <utils/task.hpp>
#include "mbed_stats.h"
#include <brain/globalsv.hpp>
#include <drivers/mcp2515.hpp>
#include <drivers/canbusmonitor.hpp>
#include <chrono>

namespace periodics
{
   /**
    * @brief Class resourcemonitor
    *
    */
    class CResourcemonitor : public utils::CTask
    {
        public:
            /* Construnctor */
            CResourcemonitor(
                std::chrono::milliseconds f_period,
                drivers::CCanBusMonitor& f_canBus
            );
            /* Destructor */
            ~CResourcemonitor();

            void callbackRESOURCEMONcommand(int64_t a, char * b);

        private:
            /* private variables & method member */
            virtual void    _run();

            drivers::CCanBusMonitor& m_canBus;

            bool m_isActive;
    }; // class CResourcemonitor
}; // namespace drivers

#endif // RESOURCEMONITOR_HPP
