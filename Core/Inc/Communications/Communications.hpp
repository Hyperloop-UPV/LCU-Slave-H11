#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "LCU_SLAVE_Types.hpp"
#include "StateMachine/StateMachine.hpp"
#include "ConfigShared.hpp"
#include "CommunicationsShared.hpp"

namespace Communications {

    using Frame = LCU_Slave::Frame;

    // Communications object to hold command/status packets
    inline CommunicationsBase comms;

    // ============================================
    // Command Processing (from Master via Frame)
    // ============================================

    inline void process_commands() {
        const auto& cmd = comms.command_packet;
        
        // Process enable/disable
        if (has_flag(cmd.commands, CommandFlags::ENABLE)) {
            LCU_SM::set_enable(true);
        }
        if (has_flag(cmd.commands, CommandFlags::DISABLE)) {
            LCU_SM::set_enable(false);
        }
        
        // Process levitation commands
        if (has_flag(cmd.commands, CommandFlags::START_LEVITATE)) {
            LCU_SM::start_levitation(cmd.gap_reference);
        }
        if (has_flag(cmd.commands, CommandFlags::STOP_LEVITATE)) {
            LCU_SM::stop_levitation();
        }
        
        // Process stick down
        if (has_flag(cmd.commands, CommandFlags::STICK_DOWN)) {
            LCU_SM::set_stick_down(true);
        }
        if (has_flag(cmd.commands, CommandFlags::RELEASE_STICK)) {
            LCU_SM::set_stick_down(false);
        }
    }

    // ============================================
    // Status Reporting (to Master via Frame)
    // ============================================

    inline void update_status() {
        auto& status = comms.status_packet;
        
        // Update state information
        status.system_state = static_cast<uint8_t>(LCU_SM::get_system_state());
        status.control_state = static_cast<uint8_t>(LCU_SM::get_control_state());
        
        // Build status flags based on current state
        status.status = StatusFlags::NONE;
        
        auto sys_state = LCU_SM::get_system_state();
        auto ctrl_state = LCU_SM::get_control_state();
        
        if (sys_state == SystemStates::IDLE) {
            status.status = status.status | StatusFlags::IDLE;
        }
        if (sys_state == SystemStates::OPERATIONAL) {
            status.status = status.status | StatusFlags::READY;
        }
        
        if (ctrl_state == ControlStates::RUNNING) {
            status.status = status.status | StatusFlags::RUNNING;
        }
        if (ctrl_state == ControlStates::STICK_DOWN) {
            status.status = status.status | StatusFlags::STICK_DOWN_ACTIVE;
        }
        
        // Update timestamp (you'll need to implement get_time_ms in STLIB)
        // status.timestamp_ms = STLIB::get_time_ms();
    }

    // ============================================
    // Initialization
    // ============================================

    inline void init() {
        // Communications initialized, Frame will handle syncing
    }

    // ============================================
    // Main Update (called from LCU_Slave::update) (TODO)
    // ============================================

    inline void update() {
        Frame::update_rx();         // Receive from Master (updates command_packet)
        process_commands();         // Parse and forward to state machine
        update_status();            // Update status_packet
        Frame::update_tx();         // Send to Master (sends status_packet)
    }

} // namespace Communications

#endif // COMMUNICATIONS_HPP