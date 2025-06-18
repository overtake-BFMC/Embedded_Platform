#ifndef CANMASK_HPP
#define CANMASK_HPP

#include <mbed.h>
#include <brain/globalsv.hpp>
#include <utils/taskmanager.hpp>
#include <utils/queue.hpp>
#include <drivers/mcp2515.hpp>
#include <drivers/canbusmonitor.hpp>
#include <map>

// TODO: Add your code here

namespace drivers
{
   /**
    * @brief Class canmask
    *
    */
    class CCanMask : public utils::CTask
    {
        public:
            typedef mbed::Callback<void(int64_t, char *)> FCallback;
            typedef std::map<uint32_t,FCallback> canSubscriberMap;
            /* Constructor */
            CCanMask(
                CCanBusMonitor& f_can,
                canSubscriberMap f_canSubscriberMap,
                PinName intPin
            );
            /* Destructor */
            ~CCanMask();
        private:
            /* private variables & method member */
            void _run();

            CCanBusMonitor& m_can;
            canSubscriberMap m_canSubscriberMap;
            uint8_t m_shutDownCounter;

            InterruptIn m_interruptPin;


    }; // class CCanmask
}; // namespace drivers

#endif // CANMASK_HPP
