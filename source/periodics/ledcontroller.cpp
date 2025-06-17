#include "periodics/ledcontroller.hpp"

// TODO: Add your code here
namespace periodics
{
    /**
     * @brief Class constructor ledcontroller
     *
     */
    CLedController::CLedController(
        std::chrono::milliseconds f_period)
        : utils::CTask(f_period), m_isActive(false), m_ledsON(false)
    {
        /* constructor behaviour */
        clearAll();
    }

    /** @brief  CLedController class destructor
     */
    CLedController::~CLedController()
    {
    }

    void CLedController::callbackLEDCONTROLLERCommand(char const *a, char *b)
    {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res)
        {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive = (l_isActivate >= 1);
                // bool_globalsV_distanceFront_isActive = (l_isActivate>=1);
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

    // void periodics::CLedController::HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
    // {
    //     if (htim->Instance == TIM1)
    //     {
    //         HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    //         // Optionally add logging or flag to indicate DMA stopped
    //         printf("PWM DMA stopped in CLASSS\n\r");
    //     }
    // }



    void CLedController::clearAll()
    {
        memset(ledData, 0, sizeof(ledData));
    }

    void CLedController::setColor( int index, uint8_t red, uint8_t green, uint8_t blue )
    {
        if( index >= LED_COUNT )
            return;

        ledData[index * 3 + 0] = green;
        ledData[index * 3 + 1] = red;
        ledData[index * 3 + 2] = blue;
    }

    void CLedController::preparePwmBuffer()
    {
        int pos = 0;
        for (int i = 0; i < LED_COUNT * 3; ++i)
        {
            for (int bit = 7; bit >= 0; --bit)
            {
                pwm_data[pos++] = (ledData[i] & (1 << bit)) ? WS2812_HIGH : WS2812_LOW;
            }
        }

        for (int i = 0; i < RESET_SLOTS ;++i)
        {
            pwm_data[pos++] = 0;
        }
    }

    void CLedController::update()
    {
        preparePwmBuffer();

        HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA( &htim3, TIM_CHANNEL_3, (uint32_t *)pwm_data, sizeof(pwm_data)/sizeof(uint16_t) );

        // preparePwmBuffer(green, red, blue);
        // printf("PWM BUFFER PREPARED\n\r");
        // HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_data, LED_COUNT * BITS_PER_LED + RESET_SLOTS);
        printf("DMA start status: %d\n\r", status);
        // printf("HAL DMA SENDED\n\r");
    }

    /* Run method */
    void CLedController::_run()
    {
        /* Run method behaviour */
        if (!m_isActive)
        {
            if (m_ledsON)
            {
                clearAll();
                update();
                m_ledsON = false;
            }
            printf("LED NOT ACTIVE\n\r");
            return;
        }

        printf("LED ACTIVE\n\r");
        if (m_ledsON)
            return;
        setColor(1, 200, 200, 200);
        update();
        m_ledsON = true;
    }

}; // namespace periodics