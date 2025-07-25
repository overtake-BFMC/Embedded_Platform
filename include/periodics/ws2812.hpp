#ifndef WS2812_HPP
#define WS2812_HPP

// TODO: Add your code here

#include <chrono>
#include <cstdint>
#include <utils/task.hpp>
#include <math.h>
#include <string.h>
#include <brain/globalsv.hpp>
#include "drivers/TIMDMADriver.h"

namespace periodics
{
    /**
     * @brief Class ws2812
     *
     */
    class CWs2812 : public utils::CTask
    {
    public:
        /* Constructor */
        CWs2812(
            std::chrono::milliseconds f_period);
        /* Destructor */
        ~CWs2812();

        void setLED(int, int, int, int, int);
        void setBrightness(int, int);
        void setGlobalBrightness(int);
        void updateBrightness();
        void update(); 
        void callbackFILLLEDcommand(int64_t a, char *b);
        void callbackSETSINGLELEDcommand(int64_t a, char *b);
        void callbackSETSIDELEDcommand(int64_t a, char *b);
        void callbackSETHALFSIDELEDcommand(int64_t a, char *b);
        void fillLED(int Red, int Green, int Blue, int Brightness);
        void fillSideLED(int Red, int Green, int Blue, int Brightness);
        void fillHalfSideLED(int Red, int Green, int Blue, int Brightness);

    private:
        /* private variables & method member */

        static constexpr int NUM_LEDS = 24;
        static constexpr int PWM_PERIOD = 105;
        static constexpr uint16_t WS2812_HIGH = 67; //70
        static constexpr uint16_t WS2812_LOW = 34; //35
        static constexpr uint16_t RESET_SLOTS = 75;
        static constexpr bool useBrightness = true;

        uint8_t LED_Data[NUM_LEDS][4];
        uint8_t LED_Mod[NUM_LEDS][4];
        uint16_t pwmData[24*NUM_LEDS + RESET_SLOTS*2];

        /* Run method */
        virtual void _run();

        /** @brief Active flag  */
        //bool m_isActive;
        bool m_fillLedActive;
        bool m_setSideLedActive;
        bool m_setSingleLedActive;
        bool m_setHalfSideLedActive;

        uint8_t singleLedIndex;
        uint8_t RGBdata[3];
        uint8_t desiredBrightness;

    }; // class CWs2812
}; // namespace periodics

#endif // WS2812_HPP
