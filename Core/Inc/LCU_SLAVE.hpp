#ifndef LCU_SLAVE_HPP
#define LCU_SLAVE_HPP

#include "LCU_SLAVE_Types.hpp"
#include "ConfigShared.hpp"
#include "StateMachine/StateMachine.hpp"
#include "Communications/Communications.hpp"

namespace LCU_Slave {

    // ============================================
    // Global Hardware Instances (definitions)
    // ============================================
    inline TimerWrapperType* g_timer = nullptr;
    inline PWMPositiveType* g_pwm_positive = nullptr;
    inline PWMNegativeType* g_pwm_negative = nullptr;
    inline EnablePinType* g_enable_pin = nullptr;
    inline LPUType* g_lpu = nullptr;
    inline Airgap* g_airgap = nullptr;
    inline LpuArrayType* g_lpu_array = nullptr;

    // ============================================
    // Initialization
    // ============================================
    inline void init() {
        Board::init();

        // Create timer wrapper on stack
        static auto my_tim = get_timer_instance(Board, timer);
        g_timer = &my_tim;
        
        static auto my_pwm1 = my_tim.template get_pwm<pwm_positive>();
        static auto my_pwm2 = my_tim.template get_pwm<pwm_negative>();
        static auto& my_en_buff = Board::instance_of<en_buff_1>();

        g_pwm_positive = &my_pwm1;
        g_pwm_negative = &my_pwm2;
        g_enable_pin = &my_en_buff;

        // Create LPU
        static auto my_lpu = LPUType(
            my_pwm1, my_pwm2,
            Pinout::shunt1, Pinout::vbat_1,
            0.0f, 1.0f, 0.0f, 1.0f
        );

        // Create Airgap
        static auto my_airgap_inst = Airgap(Pinout::airgap_1, 0.0f, 1.0f);

        // Create LPU Array
        static auto lpu_array = LpuArray(std::tie(my_lpu), std::tie(my_en_buff));

        g_lpu = &my_lpu;
        g_airgap = &my_airgap_inst;
        g_lpu_array = &lpu_array;

        STLIB::start();

        // Initialize communications
        Communications::init();

        // Initialize state machine
        LCU_SM::start();

        // Initialize frame (TX: LPU + Airgap + Status, RX: LPU + Commands)
        Frame::init(Communications::comms, my_lpu, my_airgap_inst, Communications::comms, my_lpu);
    }

    // ============================================
    // Main Loop
    // ============================================
    inline void update() {
        Communications::update();  // Handles Frame RX/TX and command processing
        LCU_SM::update();          // Update state machine transitions
        STLIB::update();           // Update ST-LIB services
    }

    // ============================================
    // Command Interface (called by Frame or external)
    // ============================================
    inline void enable() {
        LCU_SM::set_enable(true);
    }

    inline void disable() {
        LCU_SM::set_enable(false);
    }

    inline void start_control() {
        LCU_SM::start_levitation(LCU_SM::desired_gap_reference);
    }

    inline void stop_control() {
        LCU_SM::stop_control();
    }

    inline void trigger_fault() {
        LCU_SM::set_fault(true);
    }

} // namespace LCU_Slave

#endif // LCU_SLAVE_HPP