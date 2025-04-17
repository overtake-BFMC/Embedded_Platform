#ifndef CANMASK_HPP
#define CANMASK_HPP

#include <mbed.h>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/taskmanager.hpp>
/* Header file for the queue manager library*/
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
            typedef mbed::Callback<void(char const *, char *)> FCallback;
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

            InterruptIn m_interruptPin;


    }; // class CCanmask
}; // namespace drivers

#endif // CANMASK_HPP
