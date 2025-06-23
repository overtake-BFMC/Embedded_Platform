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
        : utils::CTask(f_period), m_fillLedActive(false), m_setSingleLedActive(false)
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

    void CWs2812::fillSideLED(int Red, int Green, int Blue, int Brightness)
    {
        for (int i = 0; i < 8; i++)
            setLED(i, Red, Green, Blue, Brightness);

        for (int i = 16; i < NUM_LEDS; i++)
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

    void CWs2812::callbackFILLLEDcommand(int64_t a, char *b)
    {
        if (uint8_globalsV_value_of_kl == 30)
        {
            m_fillLedActive = true;
            for (size_t i = 0; i < 3; i++)
                RGBdata[i] = (a >> i * 8) & 0xFF;

            desiredBrightness = (a >> 24) & 0xFF;
        }
        else
        {
            sprintf(b, "kl 15/30 is required!!");
        }
    }

    void CWs2812::callbackSETSIDELEDcommand(int64_t a, char *b)
    {
        if (uint8_globalsV_value_of_kl == 30)
        {
            m_setSideLedActive = true;
            for (size_t i = 0; i < 3; i++)
                RGBdata[i] = (a >> i * 8) & 0xFF;

            desiredBrightness = (a >> 24) & 0xFF;
        }
        else
        {
            sprintf(b, "kl 15/30 is required!!");
        }
    }
    
    void CWs2812::callbackSETSINGLELEDcommand(int64_t a, char *b)
    {
        if (uint8_globalsV_value_of_kl == 30)
        {
            m_setSingleLedActive = true;

            singleLedIndex = a & 0xFF;

            for (size_t i = 1; i < 4; i++)
                RGBdata[i-1] = (a >> i * 8) & 0xFF;

            desiredBrightness = (a >> 32) & 0xFF;
        }
        else
        {
            sprintf(b, "kl 15/30 is required!!");
        }
    }

    /* Run method */
    void CWs2812::_run()
    {
        if (m_fillLedActive)
        {
            fillLED(RGBdata[0], RGBdata[1], RGBdata[2], desiredBrightness);
            update();

            m_fillLedActive = false;
        }

        if(m_setSideLedActive)
        {
            fillSideLED(RGBdata[0], RGBdata[1], RGBdata[2], desiredBrightness);
            update();

            m_setSideLedActive = false;
        }

        if(m_setSingleLedActive)
        {
            setLED( singleLedIndex, RGBdata[0], RGBdata[1], RGBdata[2], desiredBrightness );
            update();

            m_setSingleLedActive = false;
        }

        
    }
}; // namespace periodics