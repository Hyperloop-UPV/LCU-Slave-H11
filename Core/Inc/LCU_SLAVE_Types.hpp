#ifndef LCU_SLAVE_TYPES_HPP
#define LCU_SLAVE_TYPES_HPP

#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "Airgap/Airgap.hpp"
#include "Pinout/Pinout.hpp"
#include "ConfigShared.hpp"
#include "SpiShared.hpp"
#include "FlagsShared.hpp"

// Forward declarations
template <typename LPUTuple, typename EnablePinTuple> class LpuArray;

// ============================================
// Hardware Configuration
// ============================================

namespace LCU_Slave {

#ifdef USE_SPI_ERROR
constexpr uint32_t MAX_SPI_ERRORS = 10;
constexpr uint32_t SPI_TIMEOUT_LIMIT = 1000;
#endif

inline constexpr auto led_operational_req =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_operational);
inline constexpr auto led_fault_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_fault);

bool master_fault_triggered = false;

inline uint32_t reset_counter = 0;
inline constexpr auto master_fault_req = ST_LIB::EXTIDomain::Device(
    Pinout::master_fault,
    ST_LIB::EXTIDomain::Trigger::FALLING_EDGE,
    []() {
        master_fault_triggered = true;
        reset_counter++;
        if (reset_counter >= 5) {
            HAL_NVIC_SystemReset();
        }
    }
);
inline constexpr auto slave_fault_req =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::slave_fault);

// Timer and PWM configuration
#ifdef USE_1_DOF
inline constexpr auto pwm_positive = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_1, .channel = Pinout::pwm1_channel_1}
);

inline constexpr auto pwm_negative = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_2, .channel = Pinout::pwm1_channel_2}
);

inline constexpr auto timer = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer15},
    pwm_positive,
    pwm_negative
);

inline constexpr auto en_buff = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_1);

float vbat_buffer = 0.0f;
float shunt_buffer = 0.0f;
float airgap_buffer = 0.0f;

inline constexpr auto adc_vbat = ST_LIB::ADCDomain::ADC(Pinout::vbat_1, vbat_buffer);
inline constexpr auto adc_shunt = ST_LIB::ADCDomain::ADC(Pinout::shunt_1, shunt_buffer);
inline constexpr auto adc_airgap = ST_LIB::ADCDomain::ADC(Pinout::airgap_1, airgap_buffer);
#elif defined(USE_5_DOF)
inline constexpr auto pwm_positive_1 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_1, .channel = Pinout::pwm1_channel_1}
);
inline constexpr auto pwm_negative_1 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_2, .channel = Pinout::pwm1_channel_2}
);
inline constexpr auto timer15 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer15},
    pwm_positive_1,
    pwm_negative_1
);


inline constexpr auto pwm_positive_2 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm2_1, .channel = Pinout::pwm2_channel_1}
);
inline constexpr auto pwm_negative_2 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm2_2, .channel = Pinout::pwm2_channel_2}
);
inline constexpr auto pwm_positive_3 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm3_1, .channel = Pinout::pwm3_channel_1}
);
inline constexpr auto pwm_negative_3 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm3_2, .channel = Pinout::pwm3_channel_2}
);
inline constexpr auto timer3 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer3},
    pwm_positive_2,
    pwm_negative_2,
    pwm_positive_3,
    pwm_negative_3
);

inline constexpr auto pwm_positive_4 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm4_1, .channel = Pinout::pwm4_channel_1}
);
inline constexpr auto pwm_negative_4 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm4_2, .channel = Pinout::pwm4_channel_2}
);
inline constexpr auto timer8 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer8},
    pwm_positive_4,
    pwm_negative_4
);

inline constexpr auto pwm_positive_5 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm5_1, .channel = Pinout::pwm5_channel_1}
);
inline constexpr auto pwm_negative_5 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm5_2, .channel = Pinout::pwm5_channel_2}
);
inline constexpr auto pwm_positive_6 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm6_1, .channel = Pinout::pwm6_channel_1}
);
inline constexpr auto pwm_negative_6 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm6_2, .channel = Pinout::pwm6_channel_2}
);
inline constexpr auto timer4 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer4},
    pwm_positive_5,
    pwm_negative_5,
    pwm_positive_6,
    pwm_negative_6
);

inline constexpr auto pwm_positive_7 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm7_1, .channel = Pinout::pwm7_channel_1}
);
inline constexpr auto pwm_negative_7 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm7_2, .channel = Pinout::pwm7_channel_2}
);
inline constexpr auto timer17 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer17},
    pwm_positive_7
);
inline constexpr auto timer16 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer16},
    pwm_negative_7
);

inline constexpr auto pwm_positive_8 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm8_1, .channel = Pinout::pwm8_channel_1}
);
inline constexpr auto pwm_negative_8 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm8_2, .channel = Pinout::pwm8_channel_2}
);
inline constexpr auto timer12 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer12},
    pwm_positive_8,
    pwm_negative_8
);

inline constexpr auto pwm_positive_9 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm9_1, .channel = Pinout::pwm9_channel_1}
);
inline constexpr auto pwm_negative_9 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm9_2, .channel = Pinout::pwm9_channel_2}
);
inline constexpr auto pwm_positive_10 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm10_1, .channel = Pinout::pwm10_channel_1}
);
inline constexpr auto pwm_negative_10 = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm10_2, .channel = Pinout::pwm10_channel_2}
);
inline constexpr auto timer1 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::timer1},
    pwm_positive_9,
    pwm_negative_9,
    pwm_positive_10,
    pwm_negative_10
);

inline constexpr auto en_buff_1 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_1);
inline constexpr auto en_buff_2 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_2);
inline constexpr auto en_buff_3 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_3);
inline constexpr auto en_buff_4 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_4);
inline constexpr auto en_buff_5 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_5);

float vbat_1_buffer = 0.0f;
float vbat_2_buffer = 0.0f;
float vbat_3_buffer = 0.0f;
float vbat_4_buffer = 0.0f;
float vbat_5_buffer = 0.0f;
float shunt1_buffer = 0.0f;
float shunt2_buffer = 0.0f;
float shunt3_buffer = 0.0f;
float shunt4_buffer = 0.0f;
float shunt5_buffer = 0.0f;
float airgap_1_buffer = 0.0f;
float airgap_2_buffer = 0.0f;
float airgap_3_buffer = 0.0f;
float airgap_4_buffer = 0.0f;
float airgap_5_buffer = 0.0f;

inline constexpr auto adc_vbat_1 = ST_LIB::ADCDomain::ADC(Pinout::vbat_1, vbat_1_buffer);
inline constexpr auto adc_vbat_2 = ST_LIB::ADCDomain::ADC(Pinout::vbat_2, vbat_2_buffer);
inline constexpr auto adc_vbat_3 = ST_LIB::ADCDomain::ADC(Pinout::vbat_3, vbat_3_buffer);
inline constexpr auto adc_vbat_4 = ST_LIB::ADCDomain::ADC(Pinout::vbat_4, vbat_4_buffer);
inline constexpr auto adc_vbat_5 = ST_LIB::ADCDomain::ADC(Pinout::vbat_5, vbat_5_buffer);
inline constexpr auto adc_shunt_1 = ST_LIB::ADCDomain::ADC(Pinout::shunt_1, shunt1_buffer);
inline constexpr auto adc_shunt_2 = ST_LIB::ADCDomain::ADC(Pinout::shunt_2, shunt2_buffer);
inline constexpr auto adc_shunt_3 = ST_LIB::ADCDomain::ADC(Pinout::shunt_3, shunt3_buffer);
inline constexpr auto adc_shunt_4 = ST_LIB::ADCDomain::ADC(Pinout::shunt_4, shunt4_buffer);
inline constexpr auto adc_shunt_5 = ST_LIB::ADCDomain::ADC(Pinout::shunt_5, shunt5_buffer);
inline constexpr auto adc_airgap_1 = ST_LIB::ADCDomain::ADC(Pinout::airgap_1, airgap_1_buffer);
inline constexpr auto adc_airgap_2 = ST_LIB::ADCDomain::ADC(Pinout::airgap_2, airgap_2_buffer);
inline constexpr auto adc_airgap_3 = ST_LIB::ADCDomain::ADC(Pinout::airgap_3, airgap_3_buffer);
inline constexpr auto adc_airgap_4 = ST_LIB::ADCDomain::ADC(Pinout::airgap_4, airgap_4_buffer);
inline constexpr auto adc_airgap_5 = ST_LIB::ADCDomain::ADC(Pinout::airgap_5, airgap_5_buffer);

#endif

// SPI Configuration
inline constexpr auto spi_req =
    ST_LIB::SPIDomain::Device<DMA_Domain::Stream::dma1_stream5, DMA_Domain::Stream::dma1_stream6>(
        ST_LIB::SPIDomain::SPIMode::SLAVE,
        Pinout::spi_peripheral,
        2000000,
        Pinout::spi_sck,
        Pinout::spi_miso,
        Pinout::spi_mosi,
        spi_conf
    );
inline constexpr auto slave_ready = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::spi_nss);

// ============================================
// Type Aliases
// ============================================
using Board = ST_LIB::Board<
    led_operational_req,
    led_fault_req,
    master_fault_req,
    slave_fault_req,
    spi_req,
    slave_ready,
#ifdef USE_1_DOF
    timer,
    en_buff,
    adc_vbat,
    adc_shunt,
    adc_airgap
#elif defined(USE_5_DOF)
    timer15, timer3, timer8, timer4, timer17, timer16, timer12, timer1,
    en_buff_1, en_buff_2, en_buff_3, en_buff_4, en_buff_5,
    adc_vbat_1, adc_vbat_2, adc_vbat_3, adc_vbat_4, adc_vbat_5,
    adc_shunt_1, adc_shunt_2, adc_shunt_3, adc_shunt_4, adc_shunt_5,
    adc_airgap_1, adc_airgap_2, adc_airgap_3, adc_airgap_4, adc_airgap_5
#endif
    >;

using EnablePinType = ST_LIB::DigitalOutputDomain::Instance;

#ifdef USE_1_DOF
using TimerWrapperType = ST_LIB::TimerWrapper<timer>;
using PWMPositiveType = decltype(std::declval<TimerWrapperType>().template get_pwm<pwm_positive>());
using PWMNegativeType = decltype(std::declval<TimerWrapperType>().template get_pwm<pwm_negative>());
using LPUType = LPU<PWMPositiveType, PWMNegativeType>;
using LpuArrayType = LpuArray<std::tuple<LPUType>, std::tuple<EnablePinType>>;
using AirgapArrayType = AirgapArray<std::tuple<Airgap>>;
#elif defined(USE_5_DOF)
using TimerWrapper15Type = ST_LIB::TimerWrapper<timer15>;
using TimerWrapper3Type = ST_LIB::TimerWrapper<timer3>;
using TimerWrapper8Type = ST_LIB::TimerWrapper<timer8>;
using TimerWrapper4Type = ST_LIB::TimerWrapper<timer4>;
using TimerWrapper17Type = ST_LIB::TimerWrapper<timer17>;
using TimerWrapper16Type = ST_LIB::TimerWrapper<timer16>;
using TimerWrapper12Type = ST_LIB::TimerWrapper<timer12>;
using TimerWrapper1Type = ST_LIB::TimerWrapper<timer1>;

using TimerWrapperTypes = std::tuple<
    TimerWrapper15Type,
    TimerWrapper3Type,
    TimerWrapper8Type,
    TimerWrapper4Type,
    TimerWrapper17Type,
    TimerWrapper16Type,
    TimerWrapper12Type,
    TimerWrapper1Type
>;

using PWMPositiveTypes = std::tuple<
    decltype(std::declval<TimerWrapper15Type>().template get_pwm<pwm_positive_1>()),
    decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_positive_2>()),
    decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_positive_3>()),
    decltype(std::declval<TimerWrapper8Type>().template get_pwm<pwm_positive_4>()),
    decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_positive_5>()),
    decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_positive_6>()),
    decltype(std::declval<TimerWrapper17Type>().template get_pwm<pwm_positive_7>()),
    decltype(std::declval<TimerWrapper12Type>().template get_pwm<pwm_positive_8>()),
    decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_positive_9>()),
    decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_positive_10>())
>;

using PWMNegativeTypes = std::tuple<
    decltype(std::declval<TimerWrapper15Type>().template get_pwm<pwm_negative_1>()),
    decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_negative_2>()),
    decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_negative_3>()),
    decltype(std::declval<TimerWrapper8Type>().template get_pwm<pwm_negative_4>()),
    decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_negative_5>()),
    decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_negative_6>()),
    decltype(std::declval<TimerWrapper16Type>().template get_pwm<pwm_negative_7>()),
    decltype(std::declval<TimerWrapper12Type>().template get_pwm<pwm_negative_8>()),
    decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_negative_9>()),
    decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_negative_10>())
>; 

using LPUTypes = std::tuple<
    LPU<decltype(std::declval<TimerWrapper15Type>().template get_pwm<pwm_positive_1>()), decltype(std::declval<TimerWrapper15Type>().template get_pwm<pwm_negative_1>())>,
    LPU<decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_positive_2>()), decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_negative_2>())>,
    LPU<decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_positive_3>()), decltype(std::declval<TimerWrapper3Type>().template get_pwm<pwm_negative_3>())>,
    LPU<decltype(std::declval<TimerWrapper8Type>().template get_pwm<pwm_positive_4>()), decltype(std::declval<TimerWrapper8Type>().template get_pwm<pwm_negative_4>())>,
    LPU<decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_positive_5>()), decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_negative_5>())>,
    LPU<decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_positive_6>()), decltype(std::declval<TimerWrapper4Type>().template get_pwm<pwm_negative_6>())>,
    LPU<decltype(std::declval<TimerWrapper17Type>().template get_pwm<pwm_positive_7>()), decltype(std::declval<TimerWrapper16Type>().template get_pwm<pwm_negative_7>())>,
    LPU<decltype(std::declval<TimerWrapper12Type>().template get_pwm<pwm_positive_8>()), decltype(std::declval<TimerWrapper12Type>().template get_pwm<pwm_negative_8>())>,
    LPU<decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_positive_9>()), decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_negative_9>())>,
    LPU<decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_positive_10>()), decltype(std::declval<TimerWrapper1Type>().template get_pwm<pwm_negative_10>())>
>;

using LpuArrayType = LpuArray<LPUTypes, std::tuple<EnablePinType, EnablePinType, EnablePinType, EnablePinType, EnablePinType>>;

using AirgapArrayType = AirgapArray<std::tuple<Airgap, Airgap, Airgap, Airgap, Airgap, Airgap, Airgap, Airgap>>;

#endif

using Frame = SystemFrame<false>; // false for Slave

using SpiType = ST_LIB::SPIDomain::SPIWrapper<spi_req>;

// ============================================
// Global Hardware Instances (extern declarations)
// ============================================

LpuArrayType* g_lpu_array;
AirgapArrayType* g_airgap_array;
ST_LIB::DigitalOutputDomain::Instance* g_led_operational;
ST_LIB::DigitalOutputDomain::Instance* g_led_fault;
ST_LIB::DigitalOutputDomain::Instance* g_slave_fault;
ST_LIB::EXTIDomain::Instance* g_master_fault;
} // namespace LCU_Slave

#endif // LCU_SLAVE_TYPES_HPP
