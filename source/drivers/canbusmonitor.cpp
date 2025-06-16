#include "drivers/canbusmonitor.hpp"

// TODO: Add your code here
namespace drivers
{
    /**
     * @brief Class constructor canbusmonitor
     *
     */
    CCanBusMonitor::CCanBusMonitor(mcp2515 &f_canBus)
        : m_canBus(f_canBus)
    {
    }

    /** @brief  CCanbusmonitor class destructor
     */
    CCanBusMonitor::~CCanBusMonitor() {
    };

    bool CCanBusMonitor::frequency(int canSpeed)
    {
        uint8_t res;

        res = m_canBus.init(canSpeed); // CAN_500KBPS_8MHZ
        ThisThread::sleep_for(chrono::milliseconds(1));

        m_canBus.setRegister(MCP_CANINTE, 0x3); // 0x3); //MCP_RX_INT);
        m_canBus.setRegister(MCP_CANINTF, 0x3); // 0xff);

        // RX0,1 as rx0,1 digital interrupt outputs
        //   m_canBus.setRegister(BFPCTRL, 0xf);

        //[Set TX0,1,2 as digital inputs
        //   m_canBus.setRegister(TXRTSCTRL, 0x0);

        // printf("Setting Normal-Mode - \n\r ");
        if (m_canBus.setCANCTRL_Mode(MODE_NORMAL) == MCP2515_OK)
        { // MODE_NORMAL MODE_LOOPBACK
          //     printf("OK\n\r");
        }
        else
        {
            error("failed\n\r");
        }

        m_canBus.dumpExtendedStatus();
        ThisThread::sleep_for(chrono::milliseconds(1));

        if (res != MCP2515_OK)
        {
            return 0;
        }
        return 1;
    }

    void CCanBusMonitor::write(CAN_Message *msg)
    {
        uint8_t txbuf_n;
        m_canBus.getNextFreeTXBuf(&txbuf_n);
        m_canBus.write_canMsg(txbuf_n, msg);
        m_canBus.start_transmit(txbuf_n);
    }

    uint8_t CCanBusMonitor::read(CAN_Message *msg)
    {
        uint8_t stat, res;

        stat = m_canBus.readStatus();

        if (stat & MCP_STAT_RX0IF)
        {
            // Msg in Buffer 0
            m_canBus.read_canMsg(MCP_RXBUF_0, msg);
            m_canBus.modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
            res = CAN_OK;
        }
        else if (stat & MCP_STAT_RX1IF)
        {
            // Msg in Buffer 1
            m_canBus.read_canMsg(MCP_RXBUF_1, msg);
            m_canBus.modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
            res = CAN_OK;
        }
        else
        {
            res = CAN_NOMSG;
        }

        return res;
    }

    uint8_t CCanBusMonitor::checkReceive(void)
    {
        uint8_t res;

        res = m_canBus.readStatus(); // RXnIF in Bit 1 and 0
        if (res & MCP_STAT_RXIF_MASK)
            return CAN_MSGAVAIL;
        else
            return CAN_NOMSG;
    }

    void CCanBusMonitor::sendMessage(int msgId, int msgData, CANFormat msgFormat, CANType msgType, uint8_t msgLength)
    {
        struct CAN_Message txMsg;

        txMsg.id = msgId;
        txMsg.format = msgFormat;
        txMsg.type = msgType;
        txMsg.len = msgLength;

        txMsg.data[0] = msgData & 0xFF;

        for( size_t i = 1; i < msgLength; i++ )
            txMsg.data[i] = ( msgData >> (i*8) ) & 0xFF;
            
        this->write(&txMsg);
    }

}; // namespace drivers