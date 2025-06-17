#include "periodics/ws2812.hpp"

// TODO: Add your code here
namespace periodics
{
    /**
     * @brief Class constructor ws2812
     *
     */
    CWs2812::CWs2812(
        std::chrono::milliseconds f_period)
        : utils::CTask(f_period), m_fillLedActive(false)
    {
        memset(LED_Data, 0, sizeof(LED_Data));
        memset(LED_Mod, 0, sizeof(LED_Mod));
        memset(pwmData, 0, sizeof(pwmData));
    }

    /** @brief  CWs2812 class destructor
     */
    CWs2812::~CWs2812()
    {
    }

    void CWs2812::setLED(int LEDnum, int Red, int Green, int Blue, int Brightness)
    {
        LED_Data[LEDnum][0] = Green;
        LED_Data[LEDnum][1] = Red;
        LED_Data[LEDnum][2] = Blue;
        LED_Data[LEDnum][3] = Brightness;
    }

    void CWs2812::fillLED(int Red, int Green, int Blue, int Brightness)
    {
        for (int i = 0; i < NUM_LEDS; i++)
            setLED(i, Red, Green, Blue, Brightness);
    }

    void CWs2812::setGlobalBrightness(int brightness)
    {
        if (useBrightness)
        {
            for (int i = 0; i < NUM_LEDS; i++)
                setBrightness(i, brightness);
        }
        updateBrightness();
    }

    void CWs2812::setBrightness(int LEDnum, int brightness)
    {
        if (useBrightness)
        {
            if (brightness > 45)
                brightness = 45;
            if (brightness < 0)
                brightness = 0;
            LED_Data[LEDnum][3] = brightness;
            updateBrightness();
        }
    }

    void CWs2812::updateBrightness()
    {
        if (useBrightness)
        {
            for (int i = 0; i < NUM_LEDS; i++)
            {
                LED_Mod[i][0] = LED_Data[i][0];
                for (int j = 0; j < 3; j++)
                {
                    float angle = 90 - LED_Data[i][3];
                    angle = angle * M_PI / 180;
                    LED_Mod[i][j] = (LED_Data[i][j] / tan(angle));
                }
            }
        }
    }

    void CWs2812::update()
    {
        updateBrightness();

        uint32_t idx = 0;
        uint32_t color;

        for (int i = 0; i < RESET_SLOTS; i++)
        {
            pwmData[idx] = 0;
            idx++;
        }

        for (int i = 0; i < NUM_LEDS; i++)
        {
            if (useBrightness)
            {
                color = ((LED_Mod[i][0] << 16) | (LED_Mod[i][1] << 8) | (LED_Mod[i][2]));
            }
            else
            {
                color = ((LED_Data[i][0] << 16) | (LED_Data[i][1] << 8) | (LED_Data[i][2]));
            }

            for (int j = 23; j >= 0; j--)
            {
                if (color & (1 << j))
                {
                    pwmData[idx] = WS2812_HIGH;
                }
                else
                {
                    pwmData[idx] = WS2812_LOW;
                }

                idx++;
            }
        }

        for (int i = 0; i < RESET_SLOTS; i++)
        {
            pwmData[idx] = 0;
            idx++;
        }

        startDMAHelper(pwmData, idx);
    }

    void CWs2812::callbackFILLLEDCommand(char const *a, char *b)
    {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res)
        {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_fillLedActive = true;
                m_fillLedBool = (l_isActivate>=1);
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

    /* Run method */
    void CWs2812::_run()
    {
        if (m_fillLedActive)
        {
            if( m_fillLedBool )
                fillLED(255,255,255,45);
            else
                fillLED(0,0,0,0);
            
            m_fillLedActive = false;
        }

        update();
    }

}; // namespace periodics