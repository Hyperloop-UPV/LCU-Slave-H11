Add a new SPI syncable component to the Frame communication

## Overview

Syncables are components that exchange data via SPI between Master and Slave.
Each syncable must implement `get_uplink_layout()` and `get_downlink_layout()` methods.

## Architecture

```mermaid
flowchart LR
    L["LPUArray"] --> A["AirgapArray"] --> S["StateMachineBase"] --> C["ControlBase"] --> R["ReportBase"]
    N["Your New Syncable"] -.-> L
```

## Steps

### 1. Create the syncable class

Your class should inherit from the appropriate base class or create it's own base:

| Component Type  | Base Class         | File                                             |
| --------------- | ------------------ | ------------------------------------------------ |
| LPU-related     | `LPUBase`          | `deps/LCU-Shared-H11/Inc/LPUShared.hpp`          |
| Airgap-related  | `AirgapBase`       | `deps/LCU-Shared-H11/Inc/AirgapShared.hpp`       |
| State-related   | `StateMachineBase` | `deps/LCU-Shared-H11/Inc/StateMachineShared.hpp` |
| Control-related | `ControlBase`      | `deps/LCU-Shared-H11/Inc/ControlShared.hpp`      |
| Diagnostics     | `ReportBase`       | `deps/LCU-Shared-H11/Inc/ReportShared.hpp`       |

### 2. Implement layout methods

```cpp
// Data sent TO master (uplink)
constexpr auto get_uplink_layout() const {
    return std::make_tuple(
        std::pair{&field1, sizeof(field1)},
        std::pair{&field2, sizeof(field2)}
    );
}

// Data received FROM master (downlink)
constexpr auto get_downlink_layout() {
    return std::make_tuple(
        std::pair{&field3, sizeof(field3)}
    );
}
```

### 3. Add to Frame template

In `Core/Inc/LCU_SLAVE.hpp`, find the Frame instantiation:

```cpp
using Frame = FrameType<false, decltype(lpu_array), decltype(airgap_array)>;
```

Add your syncable to the `FrameType` template in `deps/LCU-Shared-H11/Inc/ConfigShared.hpp`:

```cpp
template <bool isMaster, typename LPUArray, typename AirgapArray>
using FrameType = Frame<isMaster, LPUArray, AirgapArray, StateMachineBase, ControlBase, ReportBase, YOUR_NEW_TYPE>;
```

### 4. Initialize the Frame

In `Core/Src/LCU_SLAVE.cpp`, add your syncable to the Frame::init() call:

```cpp
Frame::init(
    lpu_array,
    airgap_array,
    LCU_SM::state_machine,
    Control::control,
    Communications::report,
    YOUR_NEW_INSTANCE  // Add here
);
```

## Example

See existing implementations:
- `LpuArray` in `Core/Inc/LPU/LPU.hpp`
- `AirgapArray` in `Core/Inc/Airgap/Airgap.hpp`
- `StateMachineBase` in `deps/LCU-Shared-H11/Inc/StateMachineShared.hpp`