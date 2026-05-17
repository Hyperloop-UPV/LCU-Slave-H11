#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "ConfigShared.hpp"
#include "SpiCommunications.hpp"
#include "SpiReporter.hpp"

namespace Communications {

inline ReportBase report{};

void init();
void update();
bool is_connected();

} // namespace Communications

#endif // COMMUNICATIONS_HPP
