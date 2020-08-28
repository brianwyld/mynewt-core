/**
 * Copyright 2019 Wyres
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, 
 * software distributed under the License is distributed on 
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
 * either express or implied. See the License for the specific 
 * language governing permissions and limitations under the License.
 */

/**
 * Utilities for rtc management
 */

#include <assert.h>
#include "os/mynewt.h"

#include "hal/hal_gpio.h"
#include "stm32l1xx_hal_rcc.h"
#include "stm32l1xx_hal_rtc.h"
#include "stm32l1xx_hal_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_hal_rtc_ex.h"

#include "bsp.h"
#include "limits.h"

/* The "Follower" technique is a way to calcultate the amount of time elapsed during sleep waiting for the RTC wakeup timer 
  (as this timer counter cannot be read after wakeup) by using another counter running during the sleep.
  The Follower counter needs a resolution similar to the systick timer resolution (1kHz), so that the elapsed time
  values read from it will be sufficiently precise.
  This is achieved by using the RTC's SSR register. 
  RTC_SSR is the subsecond downcounter used for calendar block and clocked by LSE subdivided    
  by asynchronous prescaler. Note that as we keep the SSR reload value such as to give 1hz to the RTC, we 
  also need to use the RTC seconds counter as part of the elapsed time calculation (for up to 60s of sleeping)
 */
/* Asynchronous prediv to get ck_apre close to 1kHz (SysTick frequency) to clock the RTC_SSR to give a 
 * precision of around 1ms for time stamps/calculations. Nearest we can get is 1024Hz with integer dividers 
 * This has a cost - normally the APRE runs slower (256Hz) which is more energy efficient (see STM32L1xx ref manual 20.3.1)
 * */
#define CK_APRE_FREQUENCY                           1024        /* desired frequency for ck_apre */
#define RTC_PRESCALER_A                             (LSE_VALUE / CK_APRE_FREQUENCY)
#define RTC_PREDIV_A                                (RTC_PRESCALER_A - 1)

/* Synchronous prediv :                                                                         */
/*    The ck_apre clock is used to clock the binary RTC_SSR subseconds downcounter. When it     */
/*    reaches 0, RTC_SSR is reloaded with the content of RTC_PREDIV_S. RTC_SSR is available in      */
/*    in Cat.2, Cat.3, Cat.4, Cat.5 and Cat.6 devices only                                      */
#define CK_SPRE_FREQUENCY                           1                       /* desire 1hz to make rtc clock have 1s resolution */
#define RTC_PRESCALER_S                             (CK_APRE_FREQUENCY / CK_SPRE_FREQUENCY)         // this will be 1024
#define RTC_PREDIV_S                                (RTC_PRESCALER_S - 1)



/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           (( uint32_t )  366U)
#define DAYS_IN_YEAR                                (( uint32_t )  365U)
#define SECONDS_IN_1DAY                             (( uint32_t )86400U)
#define SECONDS_IN_1HOUR                            (( uint32_t ) 3600U)
#define SECONDS_IN_1MINUTE                          (( uint32_t )   60U)
#define MINUTES_IN_1HOUR                            (( uint32_t )   60U)
#define HOURS_IN_1DAY                               (( uint32_t )   24U)

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              (( uint32_t )0x99AAA0)
#define  DAYS_IN_MONTH_CORRECTION_LEAP              (( uint32_t )0x445550)

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC(X, N)                                (((X) + (N) -1) / (N))

#define RTC_CLOCK_PRESCALER                         (16)
#define RTC_WAKEUP_TIMER_FREQUENCY                  (LSE_VALUE/RTC_CLOCK_PRESCALER)
#define MAX_RTC_WAKEUP_PERIOD_MSEC                  (1000*(65536/RTC_WAKEUP_TIMER_FREQUENCY))

static RTC_DateTypeDef DEFAULT_DATE = {
        .Year = 0,
        .Month = RTC_MONTH_JANUARY,
        .Date = 1,
        .WeekDay = RTC_WEEKDAY_MONDAY,
};
static RTC_TimeTypeDef DEFAULT_TIME = {
            .Hours = 0,
            .Minutes = 0,
            .Seconds = 0,
            .SubSeconds = 0,
            .TimeFormat = 0,
            .StoreOperation = RTC_STOREOPERATION_RESET,
            .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
};


/*!
 * RTC timer context 
 */
typedef struct {
    uint32_t Time;                  /* Reference time­ */
    RTC_TimeTypeDef CalendarTime;   /* Reference time in calendar format */
    RTC_DateTypeDef CalendarDate;   /* Reference date in calendar format */
}RtcTimerContext_t;


/*!
 * \brief RTC Handle
 */
static RTC_HandleTypeDef RtcHandle = {
    .Instance = NULL,
    .Init = { 
        .HourFormat = 0,
        .AsynchPrediv = 0,
        .SynchPrediv = 0,
        .OutPut = 0,
        .OutPutPolarity = 0,
        .OutPutType = 0
    },
    .Lock = HAL_UNLOCKED,
    .State = HAL_RTC_STATE_RESET
};
/*!
 * Day of year for 1st day of each month on a normal year. Months should be enumerated as 1-12 please (values provided for 0 and 13 also)
 */
static const uint16_t CumulDayOfYearByMonth[] = { 0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273U, 304U, 334U, 365U };

/* store RTC seconds / RTC SSR counter value when start a wakeup timer to be able to calculate elapsed sleep time after */
static uint32_t RTC_SSR_start;
static uint32_t RTC_seconds_start;

/* Internals predec */
static void rtc_wakeup_time_start();
static uint32_t rtc_wakeup_time_end();
static bool _rtc_wakeup_isr_called;

/**
  * @brief  This function handles  WAKE UP TIMER  interrupt request.
  * @retval None
  */
static void 
RTC_WKUP_IRQHandler(void)
{
    _rtc_wakeup_isr_called = true;
    HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandle);
}

/* was the return from sleep due to wakeup timer? */
bool 
hal_rtc_wasWake() {
    /* Check the WTUF flag in RTC ISR reg. */
    if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&RtcHandle, RTC_FLAG_WUTF) != RESET) {
        return true;
    }
    // might have been cleared by ISR, which sets this flag
    return _rtc_wakeup_isr_called;
}

void 
hal_rtc_init(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
{
    int rc;

    NVIC_DisableIRQ(RTC_WKUP_IRQn);
    NVIC_DisableIRQ(RTC_Alarm_IRQn);
    NVIC_DisableIRQ(TAMPER_STAMP_IRQn);
        
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    __HAL_RCC_RTC_ENABLE( );

    PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

    rc = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    assert(rc == HAL_OK);

    RtcHandle.Instance = RTC;
    RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv = RTC_PREDIV_A;
    RtcHandle.Init.SynchPrediv = RTC_PREDIV_S;
    RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    
    rc = HAL_RTC_Init(&RtcHandle);
    assert(rc == HAL_OK);

    if (date == NULL) {
        date = &DEFAULT_DATE;
    }

    if (time==NULL) {        
        time = &DEFAULT_TIME;
    }

    HAL_RTC_SetDate(&RtcHandle, date, RTC_FORMAT_BIN);
    HAL_RTC_SetTime(&RtcHandle, time, RTC_FORMAT_BIN);

    /* MUST enable bypass of shadow regs when reading from RTC :
     1/ as they are not updated in STOP/STANDBY mode and it takes up to 2xRTCCLK to do so (so can read pre-sleep values and think no time has passed!!) [ref man 20.3.2]
     2/ to avoid waiting on RSF
     3/ avoid bug in reading shadow regs between RTC_TR and RTC_SSR (see STM32L151CC erratra) */
    HAL_RTCEx_EnableBypassShadow(&RtcHandle);  
 
    HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
    HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_B);
    NVIC_DisableIRQ(RTC_Alarm_IRQn);

    /*Prepare WakeUp capabilities */
    HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);
    /*RTC WAKEUP used as tickless may have the same priority than Systick */
    NVIC_SetPriority(RTC_WKUP_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    /* Note : IRQ handler is not configured into hal. Do it here.*/
    NVIC_SetVector(RTC_WKUP_IRQn, (uint32_t)RTC_WKUP_IRQHandler);
    /*Enable IRQ now forever */
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
    
    /* Ensure we have values in the rtc wakeup starts*/
    rtc_wakeup_time_start();
    
}

void 
hal_rtc_enable_wakeup(uint32_t time_ms)
{    
    int rc;

    /* WARNING : works only with time_ms =< 32 s (MAX_RTC_WAKEUP_PERIOD_MSEC)
                (due to the follower which is currently 
                unable to setup lower resolution)
       
       NOTE : for time_ms > 32 follower could be used as is by using 
              ALARM features (assuming RTC clocking very different of 1Hz)
    */


    /*
     * The wakeup timer clock input can be:
     * • RTC clock (RTCCLK) divided by 2, 4, 8, or 16.
     * When RTCCLK is LSE(32.768kHz), this allows configuring the wakeup interrupt period
     * from 122 µs to 32 s, with a resolution down to 61µs
     * • ck_spre (usually 1 Hz internal clock)
     * When ck_spre frequency is 1Hz, this allows achieving a wakeup time from 1 s to
     * around 36 hours with one-second resolution. This large programmable time range is
     * divided in 2 parts:
     * – from 1s to 18 hours when WUCKSEL [2:1] = 10
     * – and from around 18h to 36h when WUCKSEL[2:1] = 11
     */
    

    if (time_ms < (uint32_t)(MAX_RTC_WAKEUP_PERIOD_MSEC)) {
        /* 0 < time_ms < 32 sec */
        /* Handles setting of wakeup counter, enabling wakeup, enabling the EXTI20 line etc */
        rc = HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, (uint32_t)((time_ms * RTC_WAKEUP_TIMER_FREQUENCY) / 1000), 
                                        RTC_WAKEUPCLOCK_RTCCLK_DIV16);
        assert (rc == HAL_OK);
    } else if (time_ms < (uint32_t)(18 * 60 * 60 * 1000)) {   
        /* 32s < time_ms < 18h */
        /* not supported even if possible with clk_srce_sel = RTC_WAKEUPCLOCK_CK_SPRE_16BITS as our ck_spre is very slow due to use of SSR as elapsed time clock
            so its about 1 tick per 32s... */
        assert(0);   
    } else {
        /* 18h < time_ms < 36h */
        assert(0);      /* not supported */
    }

    rtc_wakeup_time_start();
    _rtc_wakeup_isr_called = false;     // to know if the wakeup timer expires
}

uint32_t 
hal_rtc_get_elapsed_wakeup_timer(void)
{
    return rtc_wakeup_time_end();
}

void 
hal_rtc_disable_wakeup(void)
{
    HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle);
    
    /* In order to take into account the pending IRQ and clear      */
    /* wakeup flags (EXTI->PR & RTC_EXTI_LINE_WAKEUPTIMER_EVENT)    */    
    /* after reentring in critical region : DO NOT DISABLE IRQ !!!  */
    /* IRQ will remain enabled forever                              */
    /* NVIC_DisableIRQ(RTC_WKUP_IRQn); */
}

/* RTC access functions */
/* get clock time as unix timestamp (ms since epoch, UTC) from RTC */
uint64_t
hal_rtc_getRTCTimeMS() {
    uint64_t retMS = 0;
    uint32_t secs = 0;
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
    HAL_RTC_GetTime(&RtcHandle, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &date, RTC_FORMAT_BIN);
    /* delta with epoch date (01/01/70 : 00:00.00:0000) -> the 2 digit Year starts from 2000...
       Convert to secs first from date/time
       years (with extra day per leap year after 1972)
    */
    secs = ((date.Year+30) * (365*24*60*60)) + (((date.Year+30-2)/4) * 24*60*60);
    /* months (if valid) */
    if (date.Month>0 && date.Month<13 && date.Date>0 && date.Date<32) {
        secs += (CumulDayOfYearByMonth[date.Month] + date.Date)*(24*60*60);
    }
    /* hours/minutes/secs */
    secs += (time.Hours * 60*60) + (time.Minutes*60) + time.Seconds;
    /* to ms */
    retMS = (uint64_t)secs * 1000;
    /* add on sub second part converted to ms  */
    retMS+= (((RTC_PRESCALER_S - ((RtcHandle.Instance->SSR) & RTC_SSR_SS)) * 1000) / CK_APRE_FREQUENCY);

    return retMS;
}

void
hal_rtc_getRTCTime(uint16_t* year, uint8_t* month, uint8_t* dayOfMonth, uint8_t* hour24, uint8_t* min, uint8_t* sec, uint16_t* ms) {
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;
    HAL_RTC_GetTime(&RtcHandle, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &date, RTC_FORMAT_BIN);

    /* Read subseconds directly from the countdown ssr reg, and convert to ms since last rollover (which incremented the seconds register) */
    *ms = (((RTC_PRESCALER_S - ((RtcHandle.Instance->SSR) & RTC_SSR_SS)) * 1000) / CK_APRE_FREQUENCY);
    /* Note we assume RTC is correctly clocked at 1Hz */
    *year = date.Year;
    *month = date.Month;
    *dayOfMonth = date.Date;
    *hour24 = time.Hours;
    *min = time.Minutes;
    *sec = time.Seconds;
}


/* internals */
/* get secs and ssr regs without checking or converting */
static void 
rtc_get_rtcregs_unsafe(uint32_t* secs, uint32_t* ssr) {
    uint32_t  tmpreg = (uint32_t)(RtcHandle.Instance->TR & RTC_TR_RESERVED_MASK); 
    *secs = (uint32_t)(tmpreg & (RTC_TR_ST | RTC_TR_SU));
    /* Get subseconds structure field from the corresponding register*/
    *ssr = (uint32_t)((RtcHandle.Instance->SSR) & RTC_SSR_SS);
}

/* safely get consistant seconds and ssr values from the RTC [see RefMan STM32L1xx 20.3.6] */
static void 
rtc_get_time_safe(uint32_t* secsp, uint32_t* ssrp) {
    /* get seconds and ssr, handling case where rollover between the reads */
    uint32_t secs;
    uint32_t ssr;
    os_sr_t sr;
    while (true) {
        // get them twice in a row, with no interruptions between please
        OS_ENTER_CRITICAL(sr);
        rtc_get_rtcregs_unsafe(&secs, &ssr);
        rtc_get_rtcregs_unsafe(secsp, ssrp);
        OS_EXIT_CRITICAL(sr);
        // if same data both times its all ok
        if (secs==*secsp && ssr==*ssrp) {
            /* Convert seconds to binary from native BCD in reg. */
            *secsp = ((((secs & 0x000000F0) >> 4) * 10) + (secs & 0x0000000F));
            /* Note we use RTC in 'bypass shadow reg' mode so don't need to read RTC_DR to unblock SR updates [20.3.3] */
            return;
        }
    }
}
/* RTC wakeup timer period starts */
static void rtc_wakeup_time_start() {
    rtc_get_time_safe(&RTC_seconds_start, &RTC_SSR_start);
}

/* RTC wakeup timer period ends : return time elapse in ms */
static uint32_t rtc_wakeup_time_end() {
    uint32_t time_ms; 
    uint32_t now_secs;
    uint32_t now_ssr;   
    rtc_get_time_safe(&now_secs, &now_ssr);
    /* Note RTC_SSR is a downcounter so substract previous value from latest one (in fact this register is the value of the divider used to generate the RTC clock) */
    int32_t follower_counter_elapsed = RTC_SSR_start - now_ssr;  
    /* may be negative if rollover, handled by use of delta of seconds register */
    /* Add on the increment to the seconds register */
    int32_t seconds_elapsed = now_secs - RTC_seconds_start;
    /* deal with rollover during elapse time (only need to deal with seconds, as wakeup timer is always <32s so can't have rolled over to minutes more than once) */
    if (seconds_elapsed<0) {
        seconds_elapsed+=60;
        assert(seconds_elapsed>0);
    }
    /* Add seconds to counter converted to ms */
    time_ms = (uint32_t)((seconds_elapsed*1000) + ((follower_counter_elapsed * 1000) / CK_APRE_FREQUENCY));
    return time_ms;
}
