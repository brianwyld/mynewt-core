/*
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
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
 */

#include "stm32l1xx_hal_pwr_ex.h"
#include "stm32l1xx_hal_rcc.h"
#include "stm32l1xx_hal.h"
#include <assert.h>

/*
 * This allows an user to have a custom clock configuration by zeroing
 * every possible clock source in the syscfg.
 */
#if MYNEWT_VAL(STM32_CLOCK_MSI) || MYNEWT_VAL(STM32_CLOCK_HSE) || \
    MYNEWT_VAL(STM32_CLOCK_LSE) || MYNEWT_VAL(STM32_CLOCK_HSI) || \
    MYNEWT_VAL(STM32_CLOCK_LSI)

/*
 * MSI is turned on by default, but can be turned off and use HSE/HSI instead.
 */
#if (((MYNEWT_VAL(STM32_CLOCK_MSI) != 0) + \
      (MYNEWT_VAL(STM32_CLOCK_HSE) != 0) + \
      (MYNEWT_VAL(STM32_CLOCK_HSI) != 0)) < 1)
#error "At least one of MSI, HSE or HSI clock sources must be enabled"
#endif

void
SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_init;
    RCC_ClkInitTypeDef clk_init;
    HAL_StatusTypeDef status;

    /*
     * The voltage scaling allows optimizing the power consumption when the
     * device is clocked below the maximum system frequency, to update the
     * voltage scaling value regarding system frequency refer to product
     * datasheet.
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(MYNEWT_VAL(STM32_CLOCK_VOLTAGESCALING_CONFIG));

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_NONE;

    /*
     * LSI is used to clock the independent watchdog and optionally the RTC.
     * It can be disabled per user request, but will be automatically enabled
     * again when the IWDG is started.
     *
     * XXX currently the watchdog is not optional, so there's no point in
     * disabling LSI through syscfg.
     */
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_LSI;
#if (MYNEWT_VAL(STM32_CLOCK_LSI) == 0)
    osc_init.LSIState = RCC_LSI_OFF;
#else
    osc_init.LSIState = RCC_LSI_ON;
#endif

    /*
     * LSE is only used to clock the RTC.
     */
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_LSE;
#if (MYNEWT_VAL(STM32_CLOCK_LSE) == 0)
    osc_init.LSEState = RCC_LSE_OFF;
#elif MYNEWT_VAL(STM32_CLOCK_LSE_BYPASS)
    osc_init.LSEState = RCC_LSE_BYPASS;
#else
    osc_init.LSEState = RCC_LSE_ON;
#endif

    /*
     * MSI Oscillator
     */
#if MYNEWT_VAL(STM32_CLOCK_MSI)

#if (MYNEWT_VAL(STM32_CLOCK_MSI_CALIBRATION) > 255)
#error "Invalid MSI calibration value"
#endif
#if !IS_RCC_MSI_CLOCK_RANGE(MYNEWT_VAL(STM32_CLOCK_MSI_CLOCK_RANGE))
#error "Invalid MSI clock range"
#endif

    /* NOTE: MSI can't be disabled if it's the current PLL or SYSCLK source;
     * leave it untouched in those cases, and disable later after a new source
     * has been configured.
     */
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_MSI;
    osc_init.MSIState = RCC_MSI_ON;
    osc_init.MSICalibrationValue = MYNEWT_VAL(STM32_CLOCK_MSI_CALIBRATION);
    osc_init.MSIClockRange = MYNEWT_VAL(STM32_CLOCK_MSI_CLOCK_RANGE);
#endif

    /*
     * HSE Oscillator (can be used as PLL, SYSCLK and RTC clock source)
     */
#if MYNEWT_VAL(STM32_CLOCK_HSE)
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_HSE;

#if MYNEWT_VAL(STM32_CLOCK_HSE_BYPASS)
    osc_init.HSEState = RCC_HSE_BYPASS;
#else
    osc_init.HSEState = RCC_HSE_ON;
#endif

#endif

    /*
     * HSI Oscillator (can be used as PLL and SYSCLK clock source). It is
     * already turned on by default but a new calibration setting might be
     * used. If the user chooses to turn it off, it must be turned off after
     * SYSCLK was updated to use HSE/PLL.
     */
#if MYNEWT_VAL(STM32_CLOCK_HSI)
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
    osc_init.HSIState = RCC_HSI_ON;
    /* HSI calibration is not optional when HSI is enabled */
    osc_init.HSICalibrationValue = MYNEWT_VAL(STM32_CLOCK_HSI_CALIBRATION);

#if (MYNEWT_VAL(STM32_CLOCK_HSI_CALIBRATION) > 31)
#error "Invalid HSI calibration value"
#endif
#endif

#if MYNEWT_VAL(STM32_CLOCK_MSI)

    osc_init.PLL.PLLState = RCC_PLL_OFF;

#else

    /*
     * Default to HSE or HSI as PLL source when multiple high-speed
     * sources are enabled.
     */
    osc_init.PLL.PLLState = RCC_PLL_ON;
#if MYNEWT_VAL(STM32_CLOCK_HSE)
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#else
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
#endif

#if !IS_RCC_PLL_MUL(MYNEWT_VAL(STM32_CLOCK_PLL_MUL))
#error "PLL_MUL value is invalid"
#endif

#if !IS_RCC_PLL_DIV(MYNEWT_VAL(STM32_CLOCK_PLL_DIV))
#error "PLL_DIV value is invalid"
#endif

    osc_init.PLL.PLLMUL = MYNEWT_VAL(STM32_CLOCK_PLL_MUL);
    osc_init.PLL.PLLDIV = MYNEWT_VAL(STM32_CLOCK_PLL_DIV);

    status = HAL_RCC_OscConfig(&osc_init);
    if (status != HAL_OK) {
        assert(0);
    }

#endif

    /*
     * Select PLL as system clock source and configure the HCLK*, PCLK* and
     * SYSCLK clocks dividers. HSI, HSE and MSI are also valid system clock
     * sources, although there is no much point in supporting them now.
     */
    clk_init.ClockType =  RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
#if MYNEWT_VAL(STM32_CLOCK_MSI)
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
#else
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
#endif

#if !IS_RCC_HCLK(MYNEWT_VAL(STM32_CLOCK_AHB_DIVIDER))
#error "AHB clock divider is invalid"
#endif

#if !IS_RCC_PCLK(MYNEWT_VAL(STM32_CLOCK_APB1_DIVIDER))
#error "APB1 clock divider is invalid"
#endif

#if !IS_RCC_PCLK(MYNEWT_VAL(STM32_CLOCK_APB2_DIVIDER))
#error "APB2 clock divider is invalid"
#endif

    clk_init.AHBCLKDivider = MYNEWT_VAL(STM32_CLOCK_AHB_DIVIDER);
    clk_init.APB1CLKDivider = MYNEWT_VAL(STM32_CLOCK_APB1_DIVIDER);
    clk_init.APB2CLKDivider = MYNEWT_VAL(STM32_CLOCK_APB2_DIVIDER);

#if (MYNEWT_VAL(STM32_FLASH_LATENCY) != 0) && (MYNEWT_VAL(STM32_FLASH_LATENCY) != 1)
#error "Flash latency value is invalid"
#endif

    status = HAL_RCC_ClockConfig(&clk_init, MYNEWT_VAL(STM32_FLASH_LATENCY));
    if (status != HAL_OK) {
        assert(0);
    }

#if ((MYNEWT_VAL(STM32_CLOCK_HSI) == 0) || (MYNEWT_VAL(STM32_CLOCK_HSE) == 0) || \
     (MYNEWT_VAL(STM32_CLOCK_MSI) == 0))
    /*
     * Turn off HSE/HSI oscillator; this must be done at the end because
     * SYSCLK source has to be updated first.
     */
    osc_init.OscillatorType = RCC_OSCILLATORTYPE_NONE;
#if (MYNEWT_VAL(STM32_CLOCK_HSE) == 0)
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_OFF;
#endif
#if (MYNEWT_VAL(STM32_CLOCK_HSI) == 0)
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
    osc_init.HSIState = RCC_HSI_OFF;
#endif
#if (MYNEWT_VAL(STM32_CLOCK_MSI) == 0)
    osc_init.OscillatorType |= RCC_OSCILLATORTYPE_MSI;
    osc_init.MSIState = RCC_MSI_OFF;
#endif

#endif

    osc_init.PLL.PLLState = RCC_PLL_NONE;

    status = HAL_RCC_OscConfig(&osc_init);
    if (status != HAL_OK) {
        assert(0);
    }

#if PREFETCH_ENABLE
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
}

uint32_t wcnt;
/** Reconfig system clock to use PLL after return from a low power mode. Clock must be on MSI before calling this.
 * Note that the clocking config itself should have been setup at startup by SystemClock_Config().
 */
void
SystemClock_RestartPLL(void)
{
    /** STOP or STANDBY modes have stopped the PLL and made MSI the the system clock. 
     * No changes should have taken place to the PLL MUL/DIV or to the peripheral clocks
     * */
    /* MSI must be enabled, and be the system clock source */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET) {
        assert(0);
    }
    if (__HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_MSI) {
        assert(0);
    }
    /* PLL must be disabled  */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET) {
        assert(0);
    }
    /* Ok, turn on appropriate clock and use it */
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* also check have the right clock source for the config */
#if MYNEWT_VAL(STM32_CLOCK_HSE)
    if (__HAL_RCC_GET_PLL_OSCSOURCE()!=RCC_PLLSOURCE_HSE) {
        assert(0);
    }
    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);

    /* Wait till HSE is ready */
    wcnt=0;
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
    {
        if (wcnt++ > 100000) {
            assert(0);
       }
    }
#endif
#if MYNEWT_VAL(STM32_CLOCK_HSI)
    /* Enable HSI */
    __HAL_RCC_HSI_ENABLE( );

    /* Wait till HSI is ready */
    while( __HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET) {
    }
    if (__HAL_RCC_GET_PLL_OSCSOURCE()!=RCC_PLLSOURCE_HSI) {
        assert(0);
    }
#endif

    /* Enable PLL */
    __HAL_RCC_PLL_ENABLE( );

    /* Wait till PLL is ready */
    wcnt = 0;
    while( __HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET) {
        if (wcnt++ > 100000) {
            assert(0);
       }
    }
    /* From the STM32L151 reference manual section 6.2.7:
        Note:The SYSCLK frequency change has to follow the rule that the final frequency is less then
        4 x initial frequency to limit the VCORE drop due to a current consumption peak when the frequency increases. It must also respect 5 μs delay between two changes.
        For example to switch from 4.2 MHz to 32 MHz, the user can switch from 4.2 MHz to
        16 MHz, wait 5 μs, then switch from 16 MHz to 32 MHz.
    */
    // SO : first select HSE as clock (16MHz), wait 5uS, then switch up to PLL @32MHz
    __HAL_RCC_SYSCLK_CONFIG (RCC_SYSCLKSOURCE_HSE);
    wcnt=0;
    while(__HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_HSE) {
        if (wcnt++ > 100000) {
            assert(0);
       }
    }
    /* 5us... how long is that? 1.25 instruction/cycle for ARM Cortex-M3, cycle time@16MHz is 0.06uS. 5uS is 100x more, so must do 100 instructions (eg inc and compare) */
    for(int cnt=0;cnt<100; cnt++) {
        wcnt++;
    }

    /* Select PLL as system clock source */
    __HAL_RCC_SYSCLK_CONFIG (RCC_SYSCLKSOURCE_PLLCLK);

    /* Wait till PLL is used as system clock source */
    wcnt=0;
    while(__HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
        if (wcnt++ > 100000) {
            assert(0);
       }
    }
}

/* drop sysclock back to MSI in prepration for STOP or STANDBY mode.
 * Normally the STM32 should do this automatically, but there are cases where it hasn't done so...
 */
void SystemClock_StopPLL(void) {
    /* Enable the Multi Speed oscillator (MSI). */
    __HAL_RCC_MSI_ENABLE();

    /* Wait till MSI is ready */
    wcnt=0;
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET)  {
        if (wcnt++ > 100000) {
            assert(0);
        }
    }
    /* Select MSI as system clock source */
    __HAL_RCC_SYSCLK_CONFIG (RCC_SYSCLKSOURCE_MSI);

    /* Wait till MSI is used as system clock source */
    wcnt=0;
    while(__HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_MSI) {
        if (wcnt++ > 100000) {
            assert(0);
       }
    }
    /* Disable the main PLL. */
    __HAL_RCC_PLL_DISABLE();

}

#endif
