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

    // SPI Wrapper (owned by Communications)
    inline LCU_Slave::SpiType* g_spi = nullptr;
    inline ST_LIB::DigitalOutputDomain::Instance* g_slave_ready = nullptr;

    volatile bool send_flag = false;
    volatile bool spi_flag = false;
    volatile bool receive_flag = false;
    volatile bool operation_flag = false;

    // ============================================
    // Status Reporting
    // ============================================

    inline void update_status() {
        auto& status = comms.status_packet;
        
        status.control_state = static_cast<uint8_t>(LCU_SM::sm_operational.get_current_state());
        status.acks = comms.command_packet.commands;
    }

    // ============================================
    // Main Update
    // ============================================

    inline void init() {
        Frame::init(comms, *LCU_Slave::g_lpu, *LCU_Slave::g_airgap, comms, *LCU_Slave::g_lpu);
    }

    inline void update() {
        update_status();
        
        if (!operation_flag) {
            operation_flag = true;
            Frame::update_tx(&send_flag);
        } else if (send_flag) {
            send_flag = false;
            g_spi->transceive(Frame::tx_buffer, Frame::rx_buffer, &spi_flag);
            g_spi->set_software_nss(true);
            g_slave_ready->turn_on();
        } else if (spi_flag) {
            spi_flag = false;
            g_slave_ready->turn_off();
            g_spi->set_software_nss(false);
            Frame::update_rx(&receive_flag);
        } else if (receive_flag) {
            receive_flag = false;
            operation_flag = false;
        }
    }
} // namespace Communications

#endif // COMMUNICATIONS_HPP