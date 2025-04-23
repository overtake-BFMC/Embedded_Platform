#include "periodics/resourcemonitor.hpp"

#define max_percent_int 10000 // 100.00%

// TODO: Add your code here
namespace periodics
{
    /**
     * @brief Class constructorresourcemonitor
     *
     */
    CResourcemonitor::CResourcemonitor(std::chrono::milliseconds f_period, drivers::CCanBusMonitor &f_canBus)
        : utils::CTask(f_period), m_canBus(f_canBus), m_isActive(false)
    {
        /* constructor behaviour */
    }

    /** @brief  CResourcemonitor class destructor
     */
    CResourcemonitor::~CResourcemonitor() {
    };

    void CResourcemonitor::serialCallbackRESMONCommand(char const *a, char *b)
    {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res)
        {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive = (l_isActivate >= 1);
                bool_globalsV_resource_isActive = (l_isActivate >= 1);
                sprintf(b, "1");
            }
            else
            {
                sprintf(b, "kl 15/30 is required!!");
            }
        }
        else
        {
            sprintf(b, "syntax error");
        }
    }

    void CResourcemonitor::_run()
    {
        if (!m_isActive)
            return;

        mbed_stats_heap_t heap_stats;
        mbed_stats_stack_t stack_stats;

        mbed_stats_heap_get(&heap_stats);
        mbed_stats_stack_get(&stack_stats);

        // Compute the usage percentage
        uint16_t heap_usage_percentage = (heap_stats.current_size * max_percent_int) / heap_stats.reserved_size;
        uint16_t stack_usage_percentage = (stack_stats.max_size * max_percent_int) / stack_stats.reserved_size;

        struct CAN_Message txMsg;
        txMsg.id = 0x12D;
        txMsg.format = CANStandard;
        txMsg.type = CANData;
        txMsg.len = 4;

        txMsg.data[0] = heap_usage_percentage & 0xFF;
        txMsg.data[1] = (heap_usage_percentage >> 8) & 0xFF;
        txMsg.data[2] = (stack_usage_percentage) & 0xFF;
        txMsg.data[3] = (stack_usage_percentage >> 8) & 0xFF;

        m_canBus.write(&txMsg);
    }

}; // namespace periodics