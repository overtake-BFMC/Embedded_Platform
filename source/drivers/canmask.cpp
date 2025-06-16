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
                if( ++m_shutDownCounter > 1 )
                    bool_globalsV_ShuttedDown = true;
            }
            else
            {
                m_shutDownCounter = 0;

                if (rxMsg.len == 1)
                {
                    uint8_t val = rxMsg.data[0];
                    printf("id %d val %d;\n", rxMsg.id, val);

                    auto it = m_canSubscriberMap.find(rxMsg.id);
                    sprintf(msgValue, "%d", val);

                    if (it != m_canSubscriberMap.end()) // Check the existence of key
                    {
                        char l_resp[128] = {0};
                        it->second(msgValue, l_resp);

                        if (strlen(l_resp) > 0)
                        {
                            printf(l_resp);
                        }
                    }
                }

                if (rxMsg.len == 4)
                {
                    int32_t val = (rxMsg.data[0]) |
                                  (rxMsg.data[1] << 8) |
                                  (rxMsg.data[2] << 16) |
                                  (rxMsg.data[3] << 24);
                    printf("val %d;\n", val);

                    auto it = m_canSubscriberMap.find(rxMsg.id);
                    sprintf(msgValue, "%d", val);

                    if (it != m_canSubscriberMap.end())
                    {
                        char l_resp[128] = {0};
                        it->second(msgValue, l_resp);

                        if (strlen(l_resp) > 0)
                        {
                            printf(l_resp);
                        }
                    }
                }

                if (rxMsg.len == 6)
                {
                    int16_t val1 = (rxMsg.data[0]) |
                                   (rxMsg.data[1] << 8);
                    printf("val %d;\n", val1);

                    int16_t val2 = (rxMsg.data[2]) |
                                   (rxMsg.data[3] << 8);
                    printf("val %d;\n", val1);

                    int16_t val3 = (rxMsg.data[4]) |
                                   (rxMsg.data[5] << 8);
                    printf("val %d;\n", val3);

                    auto it = m_canSubscriberMap.find(rxMsg.id);
                    sprintf(msgValue, "%d;%d;%hhu", val2, val3, val1);

                    if (it != m_canSubscriberMap.end()) // Check the existence of key
                    {
                        char l_resp[128] = {0};
                        it->second(msgValue, l_resp);

                        if (strlen(l_resp) > 0)
                        {
                            printf(l_resp);
                        }
                    }
                }
            }

            m_can.sendMessage( 0x99, rxMsg.id, CANStandard, CANData, 4 );
        }
    }

}; // namespace drivers