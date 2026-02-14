#ifndef LCU_SLAVE_TYPES_HPP
#define LCU_SLAVE_TYPES_HPP

#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "Airgap/Airgap.hpp"
#include "Pinout/Pinout.hpp"
#include "ConfigShared.hpp"
#include "SpiShared.hpp"

// Forward declarations
template <typename LPUTuple, typename EnablePinTuple> class LpuArray;

// ============================================
// Hardware Configuration
// ============================================

namespace LCU_Slave {

inline constexpr auto led_operational_req =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_operational);
inline constexpr auto led_fault_req = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::led_fault);

// Timer and PWM configuration
inline constexpr auto pwm_positive = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_1, .channel = ST_LIB::TimerChannel::CHANNEL_1}
);

inline constexpr auto pwm_negative = ST_LIB::TimerPin(
    {.af = ST_LIB::TimerAF::PWM, .pin = Pinout::pwm1_2, .channel = ST_LIB::TimerChannel::CHANNEL_2}
);

inline constexpr auto timer = ST_LIB::TimerDomain::Timer(
    {.request = ST_LIB::TimerRequest::GeneralPurpose_15},
    pwm_positive,
    pwm_negative
);

inline constexpr auto en_buff_1 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_1);

float vbat_1_buffer = 0.0f;
float shunt1_buffer = 0.0f;
float airgap_1_buffer = 0.0f;

inline constexpr auto adc_vbat = ST_LIB::ADCDomain::ADC(Pinout::vbat_1, vbat_1_buffer);
inline constexpr auto adc_shunt = ST_LIB::ADCDomain::ADC(Pinout::shunt1, shunt1_buffer);
inline constexpr auto adc_airgap = ST_LIB::ADCDomain::ADC(Pinout::airgap_1, airgap_1_buffer);

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
    timer,
    spi_req,
    slave_ready,
    en_buff_1,
    adc_vbat,
    adc_shunt,
    adc_airgap>;
using TimerWrapperType = ST_LIB::TimerWrapper<timer>;
using PWMPositiveType = decltype(std::declval<TimerWrapperType>().template get_pwm<pwm_positive>());
using PWMNegativeType = decltype(std::declval<TimerWrapperType>().template get_pwm<pwm_negative>());
using EnablePinType = ST_LIB::DigitalOutputDomain::Instance;
using LPUType = LPU<PWMPositiveType, PWMNegativeType>;
using LpuArrayType = decltype(LpuArray(
    std::tie(*static_cast<LPUType*>(nullptr)),
    std::tie(*static_cast<EnablePinType*>(nullptr))
));

using Frame = SystemFrame<false>; // false for Slave

using SpiType = ST_LIB::SPIDomain::SPIWrapper<spi_req>;

// ============================================
// Global Hardware Instances (extern declarations)
// ============================================
PWMPositiveType* g_pwm_positive;
PWMNegativeType* g_pwm_negative;
EnablePinType* g_enable_pin;
LPUType* g_lpu;
Airgap* g_airgap;
LpuArrayType* g_lpu_array;
ST_LIB::DigitalOutputDomain::Instance* g_led_operational;
ST_LIB::DigitalOutputDomain::Instance* g_led_fault;

} // namespace LCU_Slave

#endif // LCU_SLAVE_TYPES_HPP
