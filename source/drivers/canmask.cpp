#include "drivers/canmask.hpp"

// TODO: Add your code here
namespace drivers
{
    /**
     * @brief Class constructor canmask
     *
     */
    CCanMask::CCanMask(CCanBusMonitor &f_can,
                       canSubscriberMap f_canSubscriberMap,
                       PinName intPin)
        : utils::CTask(std::chrono::milliseconds(1)), m_can(f_can), m_canSubscriberMap(f_canSubscriberMap), m_interruptPin(intPin), m_shutDownCounter(0)
    {
        printf("\ndososososso\n");
        /* constructor behaviour */
    }

    /** @brief  CCanmask class destructor
     */
    CCanMask::~CCanMask()
    {
    }

    void CCanMask::_run()
    {
        if (this->m_can.checkReceive() == CAN_NOMSG)
            return;

        struct CAN_Message rxMsg;
        char msgValue[64];

        if (this->m_can.read(&rxMsg) == CAN_OK)
        {
            if (rxMsg.id == 0x98)
            {
                if (++m_shutDownCounter > 1)
                    bool_globalsV_ShuttedDown = true;
            }
            else
            {
                m_shutDownCounter = 0;

                // size_t messageLen = rxMsg.len;
                int64_t messageValue = 0;

                for (size_t i = 0; i < rxMsg.len; i++)
                    messageValue = messageValue | (rxMsg.data[i] << i * 8);

                auto it = m_canSubscriberMap.find(rxMsg.id);
                //sprintf(msgValue, "%d", messageValue);

                if (it != m_canSubscriberMap.end()) // Check the existence of key
                {
                    char l_resp[128] = {0};
                    it->second(messageValue, l_resp);

                    if (strlen(l_resp) > 0)
                    {
                        printf(l_resp);
                    }
                }
            }

            m_can.sendMessage(0x99, rxMsg.id, CANStandard, CANData, 4);
        }
    }

}; // namespace drivers