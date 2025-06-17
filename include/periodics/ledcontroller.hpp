#ifndef LEDCONTROLLER_HPP
#define LEDCONTROLLER_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>

extern "C" {
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim3;
}


#define LED_COUNT 8
#define RESET_SLOTS 50
#define BITS_PER_LED 24
#define PWM_PERIOD 90
#define WS2812_HIGH (uint16_t)60
#define WS2812_LOW (uint16_t)30

// #define WS2812_FREQ 800000  // 800kHz
// #define TIMER_FREQ 24000000 // 24MHz TIM frequency
// #define TIM_PERIOD (TIMER_FREQ / WS2812_FREQ)

// #define WS2812_HIGH ((uint16_t)(TIMER_FREQ * 0.8 / 1000000))  // ~0.8us
// #define WS2812_LOW  ((uint16_t)(TIMER_FREQ * 0.4 / 1000000))  // ~0.4us

//TIM_HandleTypeDef htim1; 


namespace periodics
{
   /**
    * @brief Class ledcontroller
    *
    */
    class CLedController: public utils::CTask
    {
        public:
            /* Constructor */
            CLedController(
                std::chrono::milliseconds f_period
            );
            /* Destructor */
            ~CLedController();

            void callbackLEDCONTROLLERCommand(char const * a, char * b);
            void preparePwmBuffer();
            void setColor( int, uint8_t, uint8_t, uint8_t );
            void clearAll();
            void update();
            //static void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
        private:
            /* private variables & method member */
            uint8_t ledData[LED_COUNT * 3];
            uint16_t pwm_data[LED_COUNT * BITS_PER_LED + RESET_SLOTS];

            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;
            bool m_ledsON;

    }; // class CLedcontroller
}; // namespace periodics

#endif // LEDCONTROLLER_HPP
