#include "Communications/Communications.hpp"
#include "LCU_SLAVE.hpp"
#include "StateMachine/LCU_StateMachine.hpp"

namespace Communications {

void on_error() {
    LCU_Slave::slave_ready.turn_off();
    LCU_Slave::spi.set_software_nss(false);
    if (LCU_Slave::spi.was_aborted()) {
        LCU_Slave::spi.clear_abort_flag();
    }
}

using SpiComms = SpiCommunications<
    LCU_Slave::spi,
    LCU_Slave::Frame,
    +[]() { // SpiReady
        return true;
    },
    +[]() { // OnTx
        LCU_Slave::spi.set_software_nss(true);
        LCU_Slave::slave_ready.turn_on();
    },
    +[]() { // OnRxReceived
        LCU_Slave::slave_ready.turn_off();
        LCU_Slave::spi.set_software_nss(false);
    },
    +[]() { // OnRxValid
        // Assumes valid reception
        report.clear();
    },
    +[]() { // OnRxInvalid
        on_error();
    },
    +[]() { // OnTimeout
        on_error();
    },
    +[]() { // OnMaxErrors
        FAULT("Maximum SPI error count exceeded");
    },
    LCU_Slave::MAX_SPI_ERRORS,
    LCU_Slave::SPI_TIMEOUT_LIMIT>;

SpiComms spi_comms{};

void init() {
    if (auto sink_result = Diagnostics::Hub::emplace_sink<SpiReporter>(report); !sink_result) {
        FAULT("Failed to register SPI reporter sink: %d", static_cast<int>(sink_result.error()));
    }
}

void update() { spi_comms.update(); }

bool is_connected() { return spi_comms.is_connected(); }

} // namespace Communications
