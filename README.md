# OreSat Reaction Wheel Firmware

Embedded firmware for the Portland State Aerospace Society (PSAS) OreSat reaction wheel controller.

This repository contains the embedded firmware, build system, hardware verification applications, automated experiment firmware, and host-side utilities used to develop and evaluate the OreSat reaction wheel controller.

The firmware targets the **NXP MCXN947** microcontroller and is built using **Zephyr RTOS**.

---

# Repository Goals

The repository is organized around a small set of design goals intended to support long-term development and maintenance.

| Goal | Description |
|------|-------------|
| Modular Design | Organize functionality into reusable subsystems with well-defined ownership. |
| Reuse | Share the same firmware architecture across production, bring-up, and experiment firmware. |
| Maintainability | Keep hardware-specific implementation isolated from reusable software modules. |
| Repeatable Builds | Select firmware applications through build configuration rather than source modifications. |
| Extensibility | Simplify the addition of new hardware revisions, firmware applications, and experiments. |

---

# Hardware Platform

The repository currently targets the first-generation OreSat reaction wheel controller.

| Component | Purpose |
|-----------|---------|
| MCU | NXP MCXN947 |
| Motor | Three-phase BLDC reaction wheel motor |
| Encoder | MA732 magnetic encoder |
| Gate Driver | LMG2100R044 |
| Current Sensing | INA296A3 phase current amplifiers |
| Communications | CAN and UART |
| Non-Volatile Storage | On-board flash calibration storage |

---

# Repository Organization

```text
oresat-reaction-wheel-app/

├── boards/
├── configurations/
├── docs/
├── src/
├── tests/
├── tools/

├── CMakeLists.txt
├── Kconfig
└── prj.conf
```

The repository is divided into reusable firmware modules, standalone firmware applications, project documentation, and host-side utilities.

| Directory | Purpose |
|-----------|---------|
| `boards/` | Custom Zephyr board definitions |
| `configurations/` | Build configuration fragments |
| `docs/` | Project documentation |
| `src/` | Reusable firmware modules |
| `tests/` | Standalone firmware applications |
| `tools/` | Host-side utilities |

---

## boards/

Contains the custom Zephyr board definition used by the reaction wheel controller.

Typical contents include:

- Device tree files
- Board configuration
- Pin multiplexing
- Default board settings

---

## configurations/

Contains build configuration fragments used to select firmware applications.

Configuration fragments allow different firmware images to be built without modifying source code.

Typical categories include:

```text
configurations/

├── bringup/
└── experiments/
```

---

## docs/

Contains project documentation.

The documentation is organized by subsystem so that each document covers a single area of the firmware.

The repository overview is provided by this README. Detailed implementation documentation is located under `docs/`.

---

## src/

Contains the reusable firmware modules shared by every firmware image.

All reusable functionality should be implemented within this directory.

Typical modules include:

```text
src/

├── app/
├── calibration/
├── comms/
├── commutation/
├── config/
├── control/
├── hal/
└── math/
```

| Directory | Description |
|-----------|-------------|
| `app/` | Shared application infrastructure |
| `calibration/` | Persistent calibration framework |
| `comms/` | Communication interfaces |
| `commutation/` | Motor commutation algorithms |
| `config/` | Compile-time configuration |
| `control/` | Closed-loop motor controller |
| `hal/` | Hardware abstraction layer |
| `math/` | Shared mathematical utilities |

Subsystem implementation details are documented separately under `docs/`.

---

## tests/

Contains standalone firmware applications.

Applications are grouped according to their purpose.

```text
tests/

├── bringup/
└── experiments/
```

### Bring-Up Applications

Bring-up applications verify individual hardware subsystems during board development and hardware validation.

Typical applications include:

- PWM verification
- Encoder verification
- Current sensing verification
- Calibration utilities
- Hardware interface verification

### Experiment Applications

Experiment applications execute predefined operating profiles used to evaluate controller performance.

Current experiments include:

- Power Ripple
- Torque-Speed
- Step Response
- Thermal Soak

Each firmware application builds an independent firmware image while reusing the common firmware architecture located in `src/`.

---

## tools/

Contains host-side utilities used during firmware development and testing.

Typical utilities include:

- Telemetry capture
- Data acquisition
- Automated testing
- Graph generation
- Offline analysis

These utilities execute on the development computer and are independent of the embedded firmware.

---

# Repository Workflow

The repository separates reusable firmware from standalone applications.

| Task | Location |
|------|----------|
| Develop reusable firmware | `src/` |
| Create a bring-up application | `tests/bringup/` |
| Create an experiment | `tests/experiments/` |
| Add host-side utilities | `tools/` |
| Write documentation | `docs/` |
| Add build configurations | `configurations/` |

Reusable functionality should be implemented within `src/` and shared across firmware applications whenever practical.

Subsystems should expose clear interfaces and maintain explicit ownership boundaries.

Hardware-specific implementation should remain isolated from reusable firmware modules.

Firmware applications should reuse existing modules rather than duplicating functionality.

---

# Building Firmware

Firmware applications are selected using build configuration fragments.

The default build produces the production firmware.

Bring-up and experiment firmware are selected by providing an additional configuration fragment during the build process.

## Production Firmware

```bash
west build \
    -b mcxn947_reactionwheels/mcxn947/cpu0 .
```

## Bring-Up Firmware

```bash
west build \
    -p always \
    -b mcxn947_reactionwheels/mcxn947/cpu0 \
    . \
    -- \
    -DEXTRA_CONF_FILE=configurations/bringup/<application>.conf
```

## Experiment Firmware

```bash
west build \
    -p always \
    -b mcxn947_reactionwheels/mcxn947/cpu0 \
    . \
    -- \
    -DEXTRA_CONF_FILE=configurations/experiments/<experiment>.conf
```

Changing firmware applications does not require modifying source files. Only the selected configuration fragment changes.

---

# Documentation

The repository documentation is organized by subsystem.

| Document | Description |
|----------|-------------|
| `README.md` | Repository overview |
| `00_Architecture.md` | Software architecture |
| `01_HAL.md` | Hardware Abstraction Layer |
| `02_Controller.md` | Controller |
| `03_Commutation.md` | Commutation algorithms |
| `04_Calibration.md` | Calibration framework |
| `05_Telemetry.md` | Telemetry |
| `06_CAN.md` | CAN communication |
| `07_Experiments.md` | Experiment framework |
| `08_PythonTools.md` | Host-side tools |
| `09_Bringup.md` | Bring-up applications |
| `10_ExtendingFirmware.md` | Extending the firmware |
| `11_CodingStandards.md` | Coding standards |
| `12_Debugging.md` | Debugging |
| `13_Reference.md` | Reference material |