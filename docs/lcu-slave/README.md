# LCU-Slave Documentation

This directory contains technical documentation for the LCU-Slave firmware.

## Documentation Index

| Document                                 | Description                                                  |
| ---------------------------------------- | ------------------------------------------------------------ |
| [architecture.md](./architecture.md)     | Overall system architecture, hardware abstraction, DOF modes |
| [communications.md](./communications.md) | SPI/Frame protocol, MDMA transfers, syncables                |
| [control.md](./control.md)               | Control system (Simulink), two-step execution, data flow     |

## Quick Links

- [AGENTS.md](../../AGENTS.md) - Agent instructions and conventions
- [CMakePresets.json](../../CMakePresets.json) - Build presets
- [LCU-Shared-H11/](../../deps/LCU-Shared-H11/) - Shared SPI/Frame code
- [LCU-Control-H11/](../../deps/LCU-Control-H11/) - Auto-generated control
- [ST-LIB/](../../deps/ST-LIB/) - Hardware abstraction library

## Common Tasks

1. **Building**: `./hyper build main --preset <preset>`
2. **Adding syncable**: Use `.opencode/commands/add-syncable.md`
3. **Adding state**: Use `.opencode/commands/add-state.md`
4. **Adding LPU**: Use `.opencode/commands/add-lpu.md`
5. **Tracing flow**: Use `.opencode/commands/trace-flow.md`