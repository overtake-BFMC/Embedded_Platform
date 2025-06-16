#ifndef CANBUSMONITOR_HPP
#define CANBUSMONITOR_HPP

// TODO: Add your code here

/* The mbed library */
#include <mbed.h>
/* Header file for the queue manager library*/
#include <utils/queue.hpp>
#include <drivers/mcp2515.hpp>
#include <map>

#define BFPCTRL        0x0C
#define TXRTSCTRL      0x0D

namespace drivers
{
   /**
    * @brief Class canbusmonitor
    *
    */
    class CCanBusMonitor{
        public:

            /* Constructor */
            CCanBusMonitor(
                mcp2515& f_canBus
            );
            /* Destructor */
            ~CCanBusMonitor();

            uint8_t read(CAN_Message *msg); //    int read(CANMessage&    msg);
            void write(CAN_Message* msg);   //int write(CANMessage     msg);
            bool frequency( int canSpeed);
            uint8_t checkReceive(void);
            void sendMessage( int msgId, int msgData, CANFormat msgFormat, CANType msgType, uint8_t msgLength );

        private:
            mcp2515& m_canBus;
    }; // class CCanbusmonitor
}; // namespace drivers

#endif // CANBUSMONITOR_HPP
