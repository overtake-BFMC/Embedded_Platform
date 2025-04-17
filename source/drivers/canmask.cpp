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
        : utils::CTask(std::chrono::milliseconds(1)), m_can(f_can), m_canSubscriberMap(f_canSubscriberMap), m_interruptPin(intPin)
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

        printf("\ndosodosod2222222\n");

        struct CAN_Message rxMsg;
        char msgValue[64];

        if (this->m_can.read(&rxMsg) == CAN_OK)
        {
            printf("\ndosod333333333333\n");
            printf("id %d;", rxMsg.id);

            if (rxMsg.len == 1)
            {
                uint8_t val = rxMsg.data[0];
                printf("val %d;\n", val);

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

            struct CAN_Message txMsgResp;
            txMsgResp.id = 0x99;
            txMsgResp.format = CANStandard;
            txMsgResp.type = CANData;
            txMsgResp.len = 4;

            txMsgResp.data[0] = rxMsg.id & 0xFF;
            txMsgResp.data[1] = (rxMsg.id >> 8) & 0xFF;
            txMsgResp.data[2] = (rxMsg.id >> 16) & 0xFF;
            txMsgResp.data[3] = (rxMsg.id >> 24) & 0xFF;

            m_can.write(&txMsgResp);
        }
    }

}; // namespace drivers