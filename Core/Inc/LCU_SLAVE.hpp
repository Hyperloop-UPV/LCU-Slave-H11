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
    PWMPositiveType* g_pwm_positive;
    PWMNegativeType* g_pwm_negative;
    EnablePinType* g_enable_pin;
    LPUType* g_lpu;
    Airgap* g_airgap;
    LpuArrayType* g_lpu_array;

    // ============================================
    // Initialization
    // ============================================
    inline void init() {
        Board::init();

        // Create timer wrapper on stack
        auto my_tim = get_timer_instance(Board, timer);
        my_tim.set_pwm_frequency(10'000); // 10khz
        
        static auto my_pwm_positive = my_tim.template get_pwm<pwm_positive>();
        static auto my_pwm_negative = my_tim.template get_pwm<pwm_negative>();
        g_pwm_positive = &my_pwm_positive;
        g_pwm_negative = &my_pwm_negative;
        g_enable_pin = &Board::instance_of<en_buff_1>();

        auto& adc_vbat_instance = Board::instance_of<adc_vbat>();
        auto& adc_shunt_instance = Board::instance_of<adc_shunt>();
        auto& adc_airgap_instance = Board::instance_of<adc_airgap>();

        // Create LPU
        static auto my_lpu = LPUType(
            my_pwm_positive, my_pwm_negative,
            adc_vbat_instance, adc_shunt_instance,
            0.0f, 1.0f, 0.0f, 1.0f
        );
        g_lpu = &my_lpu;

        // Create Airgap
        static auto my_airgap = Airgap(adc_airgap_instance, 0.0f, 1.0f);
        g_airgap = &my_airgap;

        // Create LPU Array
        static auto my_lpu_array = LpuArray(std::tie(*g_lpu), std::tie(*g_enable_pin));
        g_lpu_array = &my_lpu_array;

        STLIB::start();

        // // Initialize communications
        // Communications::init();

        // // Initialize state machine
        // LCU_SM::start();

        // // Initialize frame (TX: LPU + Airgap + Status, RX: LPU + Commands)
        // Frame::init(Communications::comms, g_lpu, g_airgap, Communications::comms, g_lpu);


        g_lpu_array->enable_all();
    }

    // ============================================
    // Main Loop
    // ============================================
    inline void update() {
        // Communications::update();  // Handles Frame RX/TX and command processing
        // LCU_SM::update();          // Update state machine transitions
        STLIB::update();           // Update ST-LIB services

        g_lpu_array->update_all();
        g_lpu->set_duty(50.0f);
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