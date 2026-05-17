#include "Communications/SpiReporter.hpp"

namespace Communications {

bool SpiReporter::publish(const Diagnostics::DiagnosticRecord& record) {
    if (report_base.is_valid()) {
        return false; // Previous report not yet consumed, can't publish new one yet
    }

    report_base.set_from_record(record);
    return true;
}

} // namespace Communications
