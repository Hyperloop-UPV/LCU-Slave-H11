#ifndef APP_TIMING_H
#define APP_TIMING_H

#include <stdint.h>

#include "stm32h7xx.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline void app_timing_init(void)
{
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static inline uint32_t app_timing_cycles(void)
{
#if defined(DWT) && defined(CoreDebug) && defined(DWT_CTRL_CYCCNTENA_Msk)
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        app_timing_init();
    }
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

static inline uint32_t app_timing_cycles_to_us(uint32_t cycles)
{
    if (SystemCoreClock == 0U) {
        return 0U;
    }
    return (uint32_t)((((uint64_t)cycles) * 1000000ULL) / SystemCoreClock);
}

static inline uint32_t app_timing_elapsed_us(uint32_t start_cycles)
{
    return app_timing_cycles_to_us(app_timing_cycles() - start_cycles);
}

static inline uint32_t app_timing_elapsed_cycles(uint32_t start_cycles)
{
    return app_timing_cycles() - start_cycles;
}

static inline void app_timing_record_u32(
    volatile uint32_t* last_us,
    volatile uint32_t* max_us,
    uint32_t elapsed_us
)
{
    *last_us = elapsed_us;
    if (elapsed_us > *max_us) {
        *max_us = elapsed_us;
    }
}

#ifdef __cplusplus
}
#endif

#endif // APP_TIMING_H
