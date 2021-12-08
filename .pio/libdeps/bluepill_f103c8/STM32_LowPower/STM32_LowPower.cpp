/**
******************************************************************************
* @file    STM32_LowPower.cpp
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

#include "STM32_LowPower.h"
#include "rtc_api_hal.h"

STM32_LowPower LowPower;

static RTC_HandleTypeDef RtcHandle;
static voidFuncPtr RTCUserCallback = NULL;
static void *callbackUserData = NULL;

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    UNUSED(hrtc);

    if (RTCUserCallback != NULL) {
        RTCUserCallback(callbackUserData);
    }
}

void RTC_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandle);
}

#ifdef STM32G0xx
#define PWR_FLAG_WU PWR_FLAG_WUF
#endif

STM32_LowPower::STM32_LowPower()
{
    _configured = false;
    _rtc_wakeup = false;
    _timer = 0;
}

/**
  * @brief  Initializes the low power mode
  * @param  None
  * @retval None
  */
void STM32_LowPower::init(void)
{
    debug(DEBUG, "Starting STM32_LowPower::init()\n");
    /* Initialize Low Power mode using STM32 HAL */
#if !defined(STM32H7xx) && !defined(STM32WBxx)
    /* Enable Power Clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    debug(DEBUG, "STM32_LowPower::init() -> __HAL_RCC_PWR_CLK_ENABLE()\n");
#endif
    /* Allow access to Backup domain */
    HAL_PWR_EnableBkUpAccess();
    debug(DEBUG, "STM32_LowPower::init() -> HAL_PWR_EnableBkUpAccess()\n");

#ifdef __HAL_RCC_WAKEUPSTOP_CLK_CONFIG
    /* Ensure that HSI is wake-up system clock */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
    debug(DEBUG, "STM32_LowPower::init() -> __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI)\n");
#endif
    /* Check if the system was resumed from StandBy mode */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
        /* Clear Standby flag */
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
        debug(DEBUG, "STM32_LowPower::init() -> __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB)\n");
    }

    /* Clear all related wakeup flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    debug(DEBUG, "STM32_LowPower::init() -> __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU)\n");
    
    /* start RTC */
    set_time(0);

    _configured = true;
    debug(DEBUG, "Exiting STM32_LowPower::init()...\n");
}

/**
  * @brief  Enable the STM32 sleep mode.
  *         Exit this mode on interrupt using the attachInterruptWakeup() method
  *         or in timer_s seconds.
  * @param  reg: type of power regulator.
  * @param  timer_s: optional delay before leaving the sleep mode (default: 0).
  * @retval None
  */
void STM32_LowPower::sleep(STM32_RegulatorType reg, uint32_t timer_s)
{
    debug(DEBUG, "Starting STM32_LowPower::sleep()...\n");
    
    uint32_t regulator;

    /* Enable STM32 Sleep Mode: regulator in main mode */
    if (reg == STM32_MAIN_REGULATOR) {
        regulator = PWR_MAINREGULATOR_ON;
        debug(DEBUG, "STM32_LowPower::sleep() -> regulator: PWR_MAINREGULATOR_ON\n");
    }
    /* Enable STM32 Low-power Sleep Mode: regulator in low-power mode */
    else if (reg == STM32_LOWPOWER_REGULATOR) {
        regulator = PWR_LOWPOWERREGULATOR_ON;
        debug(DEBUG, "STM32_LowPower::sleep() -> regulator: PWR_LOWPOWERREGULATOR_ON\n");
    } else {
        regulator = PWR_MAINREGULATOR_ON;
    }

    if (timer_s > 0) {
        programRtcWakeUp(timer_s);
        debug(DEBUG, "STM32_LowPower::sleep() -> programRtcWakeUp(timer_s)\n");
    }
    else if (_rtc_wakeup && (_timer > 0)) {
        programRtcWakeUp(_timer);
        debug(DEBUG, "STM32_LowPower::sleep() -> programRtcWakeUp(_timer)\n");
    }

    /*
     * Suspend Tick increment to prevent wakeup by Systick interrupt.
     * Otherwise the Systick interrupt will wake up the device within
     * 1ms (HAL time base)
     */
    HAL_SuspendTick();
    debug(DEBUG, "STM32_LowPower::sleep() -> HAL_SuspendTick()\n");

    /* Enter Sleep Mode */
    debug(DEBUG, "STM32_LowPower::sleep() -> HAL_PWR_EnterSLEEPMode()\n");
    HAL_PWR_EnterSLEEPMode(regulator, PWR_SLEEPENTRY_WFI);

    /* Resume Tick interrupt if disabled before switching into Sleep mode */
    HAL_ResumeTick();
    debug(DEBUG, "STM32_LowPower::sleep() -> HAL_ResumeTick()\n");
    
    if ((timer_s > 0) || _rtc_wakeup) {
        rtc_deactivate_wake_up_timer();
    }
    
    debug(DEBUG, "Exiting STM32_LowPower::sleep()...\n");
}

/**
  * @brief  Enable the STM32 stop mode.
  *         Exit this mode on interrupt using the attachInterruptWakeup() method
  *         or in timer_s seconds.
  * @param  reg: type of power regulator.
  * @param  timer_s: optional delay before leaving the stop mode (default: 0).
  * @retval None
  */
void STM32_LowPower::stop(STM32_RegulatorType reg, uint32_t timer_s)
{
    debug(DEBUG, "Starting STM32_LowPower::stop()\n");

    __disable_irq();
    debug(DEBUG, "STM32_LowPower::stop() -> __disable_irq()\n");

    uint32_t regulator;
    /* Use regulator in main mode */
    if (reg == STM32_MAIN_REGULATOR) {
        regulator = PWR_MAINREGULATOR_ON;
        debug(DEBUG, "STM32_LowPower::stop() -> regulator: PWR_MAINREGULATOR_ON\n");
    }
    /* Use regulator in low-power mode */
    else if (reg == STM32_LOWPOWER_REGULATOR) {
        regulator = PWR_LOWPOWERREGULATOR_ON;
        debug(DEBUG, "STM32_LowPower::stop() -> regulator: PWR_LOWPOWERREGULATOR_ON\n");
    } else {
        regulator = PWR_LOWPOWERREGULATOR_ON;
    }
    
    if (timer_s > 0) {
        programRtcWakeUp(timer_s);
        debug(DEBUG, "STM32_LowPower::stop() -> programRtcWakeUp(timer_s)\n");
    }
    else if (_rtc_wakeup && (_timer > 0)) {
        programRtcWakeUp(_timer);
        debug(DEBUG, "STM32_LowPower::stop() -> programRtcWakeUp(_timer)\n");
    }

#if defined(STM32L0xx) || defined(STM32L1xx)
    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();
    debug(DEBUG, "STM32_LowPower::stop() -> HAL_PWREx_EnableUltraLowPower()\n");

    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp();
    debug(DEBUG, "STM32_LowPower::stop() -> HAL_PWREx_EnableFastWakeUp()\n");
#endif
#ifdef __HAL_RCC_WAKEUPSTOP_CLK_CONFIG
    /* Select HSI as system clock source after Wake Up from Stop mode */
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
    debug(DEBUG, "STM32_LowPower::stop() -> __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI)\n");
#endif

    /* Enter Stop mode */
    debug(DEBUG, "STM32_LowPower::stop() -> HAL_PWR_EnterSTOPMode()\n");
    HAL_Delay(20);
    HAL_PWR_EnterSTOPMode(regulator, PWR_STOPENTRY_WFI);

    /* Exit Stop mode reset clocks */
    SetSysClock();

    __enable_irq();
    debug(DEBUG, "STM32_LowPower::stop() -> __enable_irq()\n");
    
    HAL_Delay(100);
    debug(DEBUG, "STM32_LowPower::stop() -> SetSysClock()\n");
    
    if ((timer_s > 0) || _rtc_wakeup) {
        rtc_deactivate_wake_up_timer();
    }
    
    debug(DEBUG, "Exiting STM32_LowPower::stop()...\n");
}

/**
  * @brief  Enable the STM32 shutdown or standby mode.
  *         Exit this mode on interrupt using the attachInterruptWakeup() method
  *         or in timer_s seconds.
  * @param  timer_s: optional delay before leaving the standby mode (default: 0).
  * @retval None
  */
void STM32_LowPower::standby(uint32_t timer_s)
{
    __disable_irq();
    debug(DEBUG, "STM32_LowPower::standby() -> __disable_irq()\n");
    
    debug(DEBUG, "Starting STM32_LowPower::standby()...\n");
    if (timer_s > 0) {
        programRtcWakeUp(timer_s);
        debug(DEBUG, "STM32_LowPower::standby() -> programRtcWakeUp(timer_s)\n");
    }
    else if (_rtc_wakeup && (_timer > 0)) {
        programRtcWakeUp(_timer);
        debug(DEBUG, "STM32_LowPower::standby() -> programRtcWakeUp(_timer)\n");
    }

#if defined(STM32L0xx) || defined(STM32L1xx)
    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower();
    debug(DEBUG, "STM32_LowPower::standby() -> HAL_PWREx_EnableUltraLowPower()\n");

    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp();
    debug(DEBUG, "STM32_LowPower::standby() -> HAL_PWREx_EnableFastWakeUp()\n");
#endif
    debug(DEBUG, "STM32_LowPower::standby() -> HAL_PWR_EnterSTANDBYMode()\n");
    HAL_PWR_EnterSTANDBYMode();
}

/**
  * @brief  Enable GPIO pin in interrupt mode. If the pin is a wakeup pin, it is
  *         configured as wakeup source.
  * @param  pin:  pin name (PX_Y, where X = GPIO port and Y = GPIO number)
  * @param  callback: pointer to callback function.
  * @param  mode: interrupt mode (IT_MODE_RISING, IT_MODE_FALLING, IT_MODE_RISING_FALLING)
  * @retval None
  */
void STM32_LowPower::attachInterruptWakeup(PinName pin, voidFuncPtrVoid callback,
                                           Interrupt_Mode mode)
{
    debug(DEBUG, "Starting STM32_LowPower::attachInterruptWakeup()...\n");
    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> pin: %u\n", pin);
    _wakeupPin = new InterruptIn(pin);

    switch (mode) {
        case IT_MODE_RISING :
            _wakeupPin->rise(callback);
            debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wakeupPin->rise(callback)\n");
            break;
        case IT_MODE_FALLING :
            _wakeupPin->fall(callback);
            debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wakeupPin->fall(callback)\n");
            break;
        case IT_MODE_RISING_FALLING :
        default:
            _wakeupPin->rise(callback);
            _wakeupPin->fall(callback);
            debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wakeupPin->rise & fall\n");
            break;
    }

    /* If Gpio is a Wake up pin activate it in order to wake up the STM32 MCU */
#if !defined(PWR_WAKEUP_PIN1_HIGH)
    UNUSED(mode);
#endif
    uint32_t wkup_pin;
    if (pin != NC) {
        switch (pin) {
#ifdef PWR_WAKEUP_PIN1

#if defined(TARGET_DISCO_F100RB)  || defined(TARGET_DISCO_F407VG)  || \
    defined(TARGET_DISCO_F401VC)  || defined(TARGET_DISCO_F429ZI)  || \
    defined(TARGET_NUCLEO_F103RB) || defined(TARGET_NUCLEO_F207ZG) || \
    defined(TARGET_NUCLEO_F401RE) || defined(TARGET_NUCLEO_F411RE) || \
    defined(TARGET_NUCLEO_F429ZI) || defined(TARGET_NUCLEO_F439ZI) || \
    defined(TARGET_NUCLEO_F469NI) || defined(TARGET_MTB_STM_S2LP)
            case SYS_WKUP :
#else
            case SYS_WKUP1 :
#endif
                wkup_pin = PWR_WAKEUP_PIN1;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN1.\n");
#ifdef PWR_WAKEUP_PIN1_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN1_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN1_LOW.\n");
                }
#endif
                break;
#endif /* PWR_WAKEUP_PIN1 */
#ifdef PWR_WAKEUP_PIN2
            case SYS_WKUP2 :
                wkup_pin = PWR_WAKEUP_PIN2;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN2.\n");
#ifdef PWR_WAKEUP_PIN2_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN2_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN2_LOW\n");
                }
#endif
                break;
#endif /* PWR_WAKEUP_PIN2 */
#ifdef PWR_WAKEUP_PIN3
            case SYS_WKUP3 :
                wkup_pin = PWR_WAKEUP_PIN3;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN3\n");
#ifdef PWR_WAKEUP_PIN3_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN3_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN3_LOW\n");
                }
#endif
                break;
#endif /* PWR_WAKEUP_PIN3 */
#ifdef PWR_WAKEUP_PIN4
            case SYS_WKUP4 :
                wkup_pin = PWR_WAKEUP_PIN4;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN4\n");
#ifdef PWR_WAKEUP_PIN4_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN4_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN4_LOW\n");
                }
#endif
                break;
#endif /* PWR_WAKEUP_PIN4 */
#ifdef PWR_WAKEUP_PIN5
            case SYS_WKUP5 :
                wkup_pin = PWR_WAKEUP_PIN5;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN5\n");
#ifdef PWR_WAKEUP_PIN5_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN5_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN5_LOW\n");
                }
#endif
                break;
#endif /* PWR_WAKEUP_PIN5 */
#ifdef PWR_WAKEUP_PIN6
            case SYS_WKUP6 :
                wkup_pin = PWR_WAKEUP_PIN6;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN6\n");
#ifdef PWR_WAKEUP_PIN6_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN6_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN6_LOW\n");
                }
#endif
                break;
#endif /* PWR_WAKEUP_PIN6 */
#ifdef PWR_WAKEUP_PIN7
            case SYS_WKUP7 :
                wkup_pin = PWR_WAKEUP_PIN7;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN7\n");
                break;
#endif /* PWR_WAKEUP_PIN7 */
#ifdef PWR_WAKEUP_PIN8
            case SYS_WKUP8 :
                wkup_pin = PWR_WAKEUP_PIN8;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN7\n");
                break;
#endif /* PWR_WAKEUP_PIN8 */
            default :
                wkup_pin = PWR_WAKEUP_PIN1;
                debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN1 (default case)\n");
#ifdef PWR_WAKEUP_PIN1_HIGH
                if (mode != IT_MODE_RISING) {
                    wkup_pin = PWR_WAKEUP_PIN1_LOW;
                    debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> wkup_pin = PWR_WAKEUP_PIN1_LOW (default case)\n");
                }
#endif
                break;
        }

        HAL_PWR_EnableWakeUpPin(wkup_pin);
        debug(DEBUG, "STM32_LowPower::attachInterruptWakeup() -> HAL_PWR_EnableWakeUpPin()\n");
        debug(DEBUG, "Exiting STM32_LowPower::attachInterruptWakeup()...\n");
    }
}

/**
  * @brief  Configure the RTC WakeUp Interrupt.
  * @param  millis: time (in seconds) to generate a RTC WakeUp event.
  * @retval None
  */
void STM32_LowPower::programRtcWakeUp(uint32_t timer_s)
{
    uint32_t rtc_clock;
    
    core_util_critical_section_enter();
    debug(DEBUG, "STM32_LowPower::programRtcWakeUp() -> core_util_critical_section_enter()\n");
    
    rtc_clock = RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
    
    /* Clear wakeup flag, just in case. */
    SET_BIT(PWR->CR, PWR_CR_CWUF);

    /* HAL_RTCEx_SetWakeUpTimer_IT will assert that timer_s is 0xFFFF at max */
    if (timer_s > 0xFFFF) {
        timer_s -= 0x10000;
        rtc_clock = RTC_WAKEUPCLOCK_CK_SPRE_17BITS;
    }

    RtcHandle.Instance = RTC;

    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, timer_s, rtc_clock);
    debug(DEBUG, "STM32_LowPower::programRtcWakeUp() -> HAL_RTCEx_SetWakeUpTimer_IT()\n");
    
    NVIC_SetVector(RTC_WKUP_IRQn, (uint32_t)RTC_IRQHandler);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
    
    core_util_critical_section_exit();
    debug(DEBUG, "STM32_LowPower::programRtcWakeUp() -> core_util_critical_section_exit()\n");
}

/**
  * @brief  Attach a callback to a RTC WakeUp event.
  * @param  callback: callback function called when leaving the low power mode.
  * @param  data: optional pointer to callback data parameters (default NULL).
  * @retval None
  */
void STM32_LowPower::enableWakeupFromRTC(voidFuncPtr callback, void *data,
                                         uint32_t timer_s)
{
    _rtc_wakeup = true;
    if (timer_s > 0) {
        _timer = timer_s;
    }
    RTCUserCallback = callback;
    callbackUserData = data;
}