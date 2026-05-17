Trace information flow through the LCU-Slave firmware

## Overview

This command helps understand how data flows through the system, from hardware sensors through control algorithms to SPI communication with the Master.

## Flow 1: Sensor Reading

```mermaid
flowchart TD
    A["ADC Hardware<br/>(external trigger)"] --> B["AirgapArray::update()<br/>LpuArray::update_all()"]
    B --> C["Raw ADC value<br/>stored in buffer"]
    C --> D["Airgap::read()<br/>LPU::read()"]
    D --> E["Apply calibration<br/>(slope/offset)"]
    E --> F["Apply moving average filter"]
    F --> G["Calibrated value<br/>available for control"]
```

## Flow 2: Control Algorithm

### Levitation State (full control)

```mermaid
flowchart TD
    subgraph "Position Control (1000µs)"
        A1["airgap_array.get_readings()"] --> A2["Sensores[]"]
        A2 --> A3["Control::levitation_update()"]
        A3 --> A4["model.step1()"]
        A4 --> A5["Computes:<br/>Estados, Fe,<br/>CorrienteReferencia"]
    end

    subgraph "Current Control (500µs)"
        B1["lpu_array.get_shunt_readings()"] --> B2["I_HEMS[] and I_EMS[] (in 5DOF)"]
        B2 --> B3["Control::current_update()"]
        B3 --> B4["model.step0()"]
        B4 --> B5["Voltages[]<br/>(PI output)"]
        B5 --> B6["lpu_array.set_out_voltages()"]
        B6 --> B7["PWM duty cycle → H-Bridge"]
    end

    A5 --> B3
    B7 --> B8["Electromagnet"]
```

### Current Control State (direct current)

```mermaid
flowchart TD
    A["lpu_array.get_shunt_readings()"] --> B["I_HEMS[] and I_EMS[] (in 5DOF)"]
    B --> C["Control::current_update(shunt, RefCurrent)"]
    C --> D["model.step0()"]
    D --> E["lpu_array.set_out_voltages()"]
    E --> F["PWM → H-Bridge"]
```

## Flow 3: SPI Communication

```mermaid
flowchart TD
    A["Communications::update()"] --> B["Frame::update_tx()<br/>MDMA transfer"]
    B --> C["SpiCommunications::update()"]
    C --> D["transceive_DMA(tx_buffer, rx_buffer)"]
    D --> E{Receive<br/>Complete?}
    E -->|yes| F["validate()<br/>check START/END bytes"]
    E -->|no| W["Wait"]
    F --> G{valid?}
    G -->|yes| H["Frame::update_rx()<br/>MDMA to syncables"]
    G -->|no| I["error_occurred()<br/>OnRxInvalid()"]
    H --> J["OnRxValid() callback"]
    I -.-> K
    J -.-> K["Next cycle"]
    W -.-> A
```

### Uplink Data (Slave → Master)

| Data            | Source                        |
| --------------- | ----------------------------- |
| LPU voltages    | `lpu_array.vbat_v[]`          |
| LPU currents    | `lpu_array.shunt_v[]`         |
| LPU duty cycles | `lpu_array.duty_cycle[]`      |
| Airgap readings | `airgap_array.airgap_v[]`     |
| Current state   | `state_machine.current_state` |
| Control outputs | `control.output.*`            |
| Diagnostics     | `report.*`                    |

### Downlink Data (Master → Slave)

| Data              | Destination                      |
| ----------------- | -------------------------------- |
| Desired state     | `state_machine.desired_state`    |
| Z reference       | `control.input.RefZ`             |
| Current reference | `control.input.RefCurrent`       |
| Ramp step         | `control.input.ramping`          |
| Manual mode       | `control.input.ManualLevitacion` |

## Flow 4: State Machine Transitions

```mermaid
flowchart TD
    A["Main loop"] --> B["LCU_SM::update()"]
    B --> C["FaultController::check_transitions()"]
    C --> D["LCU_SM::sm_operational.check_transitions()"]
    D --> E["Evaluate guards for each state"]
    E --> F{Guard<br/>satisfied?}
    F -->|yes| G["Transition to new state"]
    F -->|no| H["Stay in current state"]
    G --> I["Call enter/exit handlers"]
    I --> J["Execute cyclic functions<br/>at registered rates"]
    H -.-> J
    J -.-> A
```

*Fault may be entered from anywhere in the program*

## Key Files for Each Flow

| Flow           | Files                                                                                      |
| -------------- | ------------------------------------------------------------------------------------------ |
| Sensor reading | `Core/Inc/Airgap/Airgap.hpp`, `Core/Inc/LPU/LPU.hpp`                                       |
| Control        | `Core/Inc/Control/Control.hpp`, `deps/LCU-Control-H11/ControlTop/`                         |
| SPI comms      | `deps/LCU-Shared-H11/Inc/FrameShared.hpp`, `deps/LCU-Shared-H11/Inc/SpiCommunications.hpp` |
| State machine  | `Core/Inc/StateMachine/LCU_StateMachine.hpp`, `Core/Src/StateMachine/LCU_StateMachine.cpp` |