#ifndef LCU_SLAVE_TYPES_HPP
#define LCU_SLAVE_TYPES_HPP

#include "ST-LIB.hpp"
#include "LPU/LPU.hpp"
#include "Airgap/Airgap.hpp"
#include "Pinout/Pinout.hpp"
#include "ST-LIB_LOW/DigitalOutput2.hpp"
#include "ConfigShared.hpp"

// Forward declarations
template <typename LPUTuple, typename EnablePinTuple>
class LpuArray;

// ============================================
// Hardware Configuration
// ============================================

namespace LCU_Slave {

    // Timer and PWM configuration
    inline constexpr auto pwm_positive = ST_LIB::TimerPin({
        .af = ST_LIB::TimerAF::PWM, 
        .pin = Pinout::pwm1_1, 
        .channel = ST_LIB::TimerChannel::CHANNEL_1
    });

    inline constexpr auto pwm_negative = ST_LIB::TimerPin({
        .af = ST_LIB::TimerAF::PWM, 
        .pin = Pinout::pwm1_2, 
        .channel = ST_LIB::TimerChannel::CHANNEL_2
    });

    inline constexpr auto timer = ST_LIB::TimerDomain::Timer(
        {.request = ST_LIB::TimerRequest::GeneralPurpose_15}, 
        pwm_positive, 
        pwm_negative
    );

    inline constexpr auto en_buff_1 = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::en_buff_1);

    // ============================================
    // Type Aliases
    // ============================================
    using Board = ST_LIB::Board<timer, en_buff_1>;
    using TimerWrapperType = ST_LIB::TimerWrapper<timer>;
    using PWMPositiveType = decltype(std::declval<TimerWrapperType>().template get_pwm<pwm_positive>());
    using PWMNegativeType = decltype(std::declval<TimerWrapperType>().template get_pwm<pwm_negative>());
    using EnablePinType = std::remove_reference_t<decltype(Board::instance_of<en_buff_1>())>;
    using LPUType = LPU<PWMPositiveType, PWMNegativeType>;
    using LpuArrayType = decltype(LpuArray(
        std::tie(*static_cast<LPUType*>(nullptr)), 
        std::tie(*static_cast<EnablePinType*>(nullptr))
    ));

    using Frame = SystemFrame<false>; // false for Slave

    // ============================================
    // Global Hardware Instances (extern declarations)
    // ============================================
    extern TimerWrapperType* g_timer;
    extern PWMPositiveType* g_pwm_positive;
    extern PWMNegativeType* g_pwm_negative;
    extern EnablePinType* g_enable_pin;
    extern LPUType* g_lpu;
    extern Airgap* g_airgap;
    extern LpuArrayType* g_lpu_array;

} // namespace LCU_Slave

#endif // LCU_SLAVE_TYPES_HPP