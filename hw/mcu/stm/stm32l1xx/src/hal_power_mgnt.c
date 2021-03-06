/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>
#include <assert.h>
#include <os/mynewt.h>
#include <syscfg/syscfg.h>
#include <mcu/stm32_hal.h>
#include <hal/hal_bsp.h>
#include "stm32l1xx_hal_pwr.h"
#include "stm32l1xx_hal_rtc.h"

extern void SystemClock_RestartPLL(void);
extern void SystemClock_StopPLL(void);
extern bool hal_rtc_wasWake();
extern void hal_rtc_enable_wakeup(uint32_t time_ms);
extern void hal_rtc_disable_wakeup(void);
extern uint32_t hal_rtc_get_elapsed_wakeup_timer(void);
extern void hal_rtc_init(RTC_DateTypeDef *date, RTC_TimeTypeDef *time);
/* maximum time that can be requested from the wakeup timer (and still get the elapsed rtc time correctly) */
#define MAX_RTC_WAKEUP_TIMER_MS (30000)

void stm32_tickless_start(uint32_t timeMS);

uint32_t QQQ_nsleep0;
uint32_t QQQ_nsleepX;
uint32_t QQQ_nstopX;
uint32_t QQQ_total_reqsleepX;
uint32_t QQQ_total_reqstopX;
uint32_t QQQ_total_delta;
uint32_t QQQ_total_slept;
uint32_t QQQ_wakeups;

/* Put MCU  in lowest power stop state, exit only via POR or reset pin */
void 
hal_mcu_halt() 
{

    /* all interupts and exceptions off */
    /* PVD off */
    /* power watchdog off */
    /* Be in lowest power mode forever */

    /* ensure RTC not gonna wake us */
    hal_rtc_disable_wakeup();

    /* Stop SYSTICK */
    NVIC_DisableIRQ(SysTick_IRQn);
    /* Suspend SysTick Interrupt and disable it completely */
//    CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
    SysTick->CTRL = 0;

    /*Disables the Power Voltage Detector(PVD) */                
    HAL_PWR_DisablePVD( );
    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower( );
    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_DisableFastWakeUp( );
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
    /* System clock down to MSI */
    SystemClock_StopPLL();

    while (1) {
        /* Enters STANDBY mode, which can only be exited via reset (normally) */
        HAL_PWR_EnterSTANDBYMode();
        // Shouldn't return but loop to re-enter mode...
    }
}

void 
stm32_tick_init(uint32_t os_ticks_per_sec, int prio)
{
    /* Even for tickless we use SYSTICK for normal tick.*/
    /* nb of ticks per seconds is hardcoded in HAL_InitTick(..) to have 1ms/tick */
    assert(os_ticks_per_sec == OS_TICKS_PER_SEC);
    
    volatile uint32_t reload_val;

    /*Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s) */
    reload_val = ((uint64_t)SystemCoreClock / os_ticks_per_sec) - 1;
    /* Set the system time ticker up */
    SysTick->LOAD = reload_val;
    SysTick->VAL = 0;

    /* CLKSOURCE : 1 -> HCLK, 0-> AHB Clock (which is HCLK/8). Use HCLK, as this is the value of SystemCoreClock as used above */
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);  
        
    /* Set the system tick priority */
    NVIC_SetPriority(SysTick_IRQn, prio);

#ifdef RELEASE_BUILD
    HAL_DBGMCU_DisableDBGSleepMode();
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_DBGMCU_DisableDBGStandbyMode();
#else
    /* Keep clocking debug even when CPU is sleeping, stopped or in standby.*/
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_DBGMCU_EnableDBGStandbyMode();
#endif

#if MYNEWT_VAL(OS_TICKLESS_RTC)

    /* initialise RTC for tickless code if required */
    hal_rtc_init(NULL, NULL);

#endif
}

void 
stm32_tickless_start(uint32_t timeMS)
{

    /* Start RTC alarm for in this amount of time if not 0 (note: case of timeMS==0 is used for HALT ie never coming back) */
    if (timeMS > 0) {
        hal_rtc_enable_wakeup(timeMS);
    }
    /* Suspend SysTick Interrupt */
    NVIC_DisableIRQ(SysTick_IRQn);
    /* Stop SYSTICK */
    CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

void 
stm32_tickless_stop(uint32_t timeMS)
{
    if (hal_rtc_wasWake()) {
        QQQ_wakeups++;
    }
    /* add asleep duration to tick counter */
    uint32_t asleep_ms = hal_rtc_get_elapsed_wakeup_timer();
    int asleep_ticks = os_time_ms_to_ticks32(asleep_ms);
    assert(asleep_ticks >= 0);
    os_time_advance(asleep_ticks);

    /* reenable SysTick Interrupt */
    NVIC_EnableIRQ(SysTick_IRQn);
    /* reenable SysTick */
    SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);

    /* disable RTC wakeup */
    hal_rtc_disable_wakeup();

    QQQ_total_delta += (timeMS-asleep_ms);
    QQQ_total_slept += asleep_ms;
}

void 
stm32_power_enter(int power_mode, uint32_t durationMS)
{
    /* if sleep time was less than MIN_TICKS, it is 0. Just do usual WFI and systick will wake us in 1ms */
    if (durationMS == 0) {
        QQQ_nsleep0++;
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        return;
    }

    /* limit sleep to largest value of wakeuptimer that is supported by RTC - ensure we wake up before RTC timer wraps */
    if (durationMS > MAX_RTC_WAKEUP_TIMER_MS) {
        durationMS = MAX_RTC_WAKEUP_TIMER_MS; 
    }

    /* reduce duration by 10ms to _ensure_ we wake up before required time (avoid OS finding it has events in past to run) */
    durationMS -= 10;
    /* begin tickless */
#if MYNEWT_VAL(OS_TICKLESS_RTC)
    stm32_tickless_start(durationMS);
#endif
    switch (power_mode) {
    case HAL_BSP_POWER_OFF:
    case HAL_BSP_POWER_DEEP_SLEEP: {
        /* Enable Ultra low power mode */
        HAL_PWREx_EnableUltraLowPower( );
        /* Enable the fast wake up from Ultra low power mode as we don't care about Vrefint readiness */
        HAL_PWREx_EnableFastWakeUp( );
        /* System clock down to MSI */
        SystemClock_StopPLL();
        /* Enters StandBy mode */
        HAL_PWR_EnterSTANDBYMode();
        /* If exit standby mode then the RAM has been lost. Reboot cleanly. */
        hal_system_reset();
        break;
    }
    case HAL_BSP_POWER_SLEEP: {
        QQQ_nstopX++;
        QQQ_total_reqstopX += durationMS;
        /* Note: BSP level should decide if it wants to use PVD and disable/enable it */
        /* Enable Ultra low power mode (Vrefint off) */
        HAL_PWREx_EnableUltraLowPower( );
        /* Enable the fast wake up from Ultra low power mode (users of Vrefint should check the flag before enabling eg ADC or PVD) */
        HAL_PWREx_EnableFastWakeUp( );
        /* System clock down to MSI */
        SystemClock_StopPLL();
        /* Enters Stop mode (with LP regulator instead of PWR_MAINREGULATOR_ON) */
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        /* STOP mode has halted the clocks and will be running on MSI - restart correctly */
        SystemClock_RestartPLL();
        break;
    }
    case HAL_BSP_POWER_WFI: {
        QQQ_nsleepX++;
        QQQ_total_reqsleepX += durationMS;
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        /* Clock is not interuppted in SLEEP mode, no need to restart it */
        break;
    }
    case HAL_BSP_POWER_ON:
    default: {
        
        break;
    }
    }
    
#if MYNEWT_VAL(OS_TICKLESS_RTC)
    /* exit tickless low power mode */
    stm32_tickless_stop(durationMS);
#endif
}
