#ifndef SPI_REPORTER_HPP
#define SPI_REPORTER_HPP

#include "HALAL/Services/Diagnostics/Diagnostics.hpp"
#include "ReportShared.hpp"

namespace Communications {

class SpiReporter : public Diagnostics::DiagnosticSink {
public:
    explicit SpiReporter(ReportBase& report_base) : report_base(report_base) {}

    bool publish(const Diagnostics::DiagnosticRecord& record) override;

private:
    ReportBase& report_base;
};

} // namespace Communications

#endif // SPI_REPORTER_HPP
