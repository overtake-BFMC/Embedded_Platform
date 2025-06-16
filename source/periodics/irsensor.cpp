#include "periodics/irsensor.hpp"

// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor irsensor
    *
    */
    CIRsensor::CIRsensor(
        std::chrono::milliseconds f_period,
        mbed::DigitalIn f_pin,
        drivers::CCanBusMonitor& f_canBus
    )
    : utils::CTask(f_period)
    , m_pin(f_pin)
    , m_canBus(f_canBus)
    {
    }

    /** @brief  CIRsensor class destructor
     */
    CIRsensor::~CIRsensor()
    {
    }

    void CIRsensor::callbackIRSENSORCommand(char const *a, char *b)
    {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_irsensor_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
        }else{
            sprintf(b,"syntax error");
        }
    }

    /* Run method */
    void CIRsensor::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;

        m_value = m_pin.read();

        m_canBus.sendMessage( 0x11D, m_value, CANStandard, CANData, 1 );

        //printf( "IR %d\n\r", m_value);
    }

}; // namespace periodics