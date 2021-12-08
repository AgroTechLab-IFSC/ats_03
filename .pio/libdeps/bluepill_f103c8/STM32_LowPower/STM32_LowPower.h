/**
******************************************************************************
* @file    STM32_LowPower.h
* @author  Biagio Montaruli, STM32duino team (STMicroelectronics)
* @version V1.0.0
* @date    12-April-2019
* @brief   Mbed Library to manage Low Power modes on STM32 boards
*
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
* <h2><center>&copy; COPYRIGHT(c) 2019 Biagio Montaruli</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

#ifndef _STM32_LOW_POWER_H_
#define _STM32_LOW_POWER_H_

#include "mbed.h"

#define DEBUG 0

/* Check if PWR HAL enable in mbed-os/targets/TARGET_STM/TARGET_STM32YZ/device/stm32yzxx_hal_conf.h */
#ifndef HAL_PWR_MODULE_ENABLED
#error "PWR configuration is missing. Check flag HAL_PWR_MODULE_ENABLED in " \
       "mbed-os/targets/TARGET_STM/TARGET_STM32YZ/device/stm32yzxx_hal_conf.h"
#endif

typedef void (*voidFuncPtrVoid)(void);
typedef void (*voidFuncPtr)(void *) ;

typedef enum {
    STM32_MAIN_REGULATOR = 0,
    STM32_LOWPOWER_REGULATOR = 1
} STM32_RegulatorType;

typedef enum {
    IT_MODE_RISING = 0,
    IT_MODE_FALLING = 1,
    IT_MODE_RISING_FALLING = 3
} Interrupt_Mode;

class STM32_LowPower
{
public:
    STM32_LowPower();

    void init(void);

    void sleep(STM32_RegulatorType reg = STM32_MAIN_REGULATOR, uint32_t timer_s = 0);

    void stop(STM32_RegulatorType reg = STM32_MAIN_REGULATOR, uint32_t timer_s = 0);

    void standby(uint32_t timer_s = 0);

    void attachInterruptWakeup(PinName pin, voidFuncPtrVoid callback,
                               Interrupt_Mode mode);
    void enableWakeupFromRTC(voidFuncPtr callback, void *data = NULL, uint32_t timer_s = 0);

private:
    bool _configured;     /* Low Power mode initialization status */
    bool _rtc_wakeup;
    uint32_t _timer;
    InterruptIn *_wakeupPin;
    
    void programRtcWakeUp(uint32_t timer_s);
};

extern STM32_LowPower LowPower;

#endif // _STM32_LOW_POWER_H_
