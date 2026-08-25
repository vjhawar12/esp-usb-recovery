# ESP32-S3 USB Recovery & Diagnostics

A low-level embedded recovery and diagnostics platform built around an **ESP32-S3 USB composite device**, a **vendor-specific USB protocol**, and a secondary UART link to a target MCU.

The project is designed around a simple question: **if the target application's normal communications path is broken, how can a technician still inspect, diagnose, and recover the system?**

Rather than treating USB as a black-box serial connection, this project progressively moves below convenience APIs into TinyUSB's device stack. The goal is to understand and control descriptors, endpoints, transfers, buffering, callbacks, class-driver behavior, and eventually the vendor device-class implementation itself.

> **Project focus:** embedded C, USB device internals, recovery architecture, deterministic diagnostics, RTOS integration, and hardware/software interfaces.

---

## System Architecture

```text
                         Linux diagnostic host
                                  |
                                  | USB 2.0 Full Speed
                                  |
                    +-------------+-------------+
                    |       ESP32-S3            |
                    |   Diagnostic Controller   |
                    |                           |
                    |  CDC ACM     Vendor USB   |
                    |  logs /      recovery /   |
                    |  telemetry   diagnostics  |
                    +-------------+-------------+
                                  |
                                  | UART / recovery interface
                                  |
                    +-------------+-------------+
                    |        Target MCU         |
                    | application / firmware    |
                    +---------------------------+
```

The ESP32-S3 acts as an independent diagnostic controller rather than depending on the target MCU's primary application interface.

The intended split is:

- **CDC ACM** — human-readable logs, status messages, and development telemetry.
- **Vendor-specific USB interface** — structured diagnostic and recovery commands.
- **UART target interface** — communication with the target MCU when its normal external interface is unavailable.

This makes the diagnostic path useful even when the target firmware is partially failed or its normal communications stack cannot be trusted.

---

## USB Stack: What This Project Owns

USB is not one monolithic driver. The device path can be viewed as several layers:

```text
+-------------------------------------------------------------+
| Layer 5: Recovery application / diagnostic protocol         |
| Command parser, fault capture, recovery policy, reporting   |
+-------------------------------------------------------------+
| Layer 4: USB device classes                                 |
| CDC ACM + vendor-specific class                             |
| Goal: implement the vendor class-driver callbacks directly  |
+-------------------------------------------------------------+
| Layer 3: TinyUSB device core / endpoint transfer API        |
| Endpoint claim, transfer submission, transfer completion    |
| Current code directly uses usbd_edpt_claim/usbd_edpt_xfer   |
+-------------------------------------------------------------+
| Layer 2: Device Controller Driver (DCD)                     |
| TinyUSB / ESP-IDF interface to the ESP32-S3 USB peripheral  |
+-------------------------------------------------------------+
| Layer 1: ESP32-S3 USB peripheral + internal PHY             |
+-------------------------------------------------------------+
```

The project intentionally focuses on **Layers 3-5**.

The goal is **not** to rewrite the ESP32-S3 USB PHY or hardware Device Controller Driver. Instead, the project is moving below high-level CDC/vendor convenience functions so that endpoint behavior and the vendor class implementation are visible and controllable in C.

### Current low-level work

Transmit paths currently bypass the TinyUSB class TX FIFO and submit transfers through the device-core endpoint API:

```c
if (!usbd_edpt_claim(0, VENDOR_BULK_IN)) {
    return VENDOR_TX_FULL;
}

usbd_edpt_xfer(0, VENDOR_BULK_IN, buffer, size);
```

The same approach is used for CDC IN transfers.

This exposes several concerns that are normally hidden by higher-level APIs:

- endpoint ownership and contention
- transfer lifetime
- buffer lifetime
- direct endpoint submission
- transfer completion behavior
- error propagation
- avoiding unnecessary intermediate copies/FIFOs where possible

### Next low-level milestone: custom vendor class driver

The vendor receive path currently enters through TinyUSB's existing vendor-class callback:

```c
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint16_t bufsize)
```

The next major step is to take ownership of the vendor device-class layer itself and implement the class-driver lifecycle and transfer hooks rather than relying on TinyUSB's stock vendor class implementation.

The target callbacks are:

```text
vendord_init()
vendord_deinit()
vendord_reset()
vendord_open()
vendord_control_xfer_cb()
vendord_xfer_cb()
```

That moves responsibility for the vendor interface down another layer: parsing its descriptors, opening endpoints, handling control requests, processing transfer completions, re-arming OUT endpoints, and managing class-specific state.

This is the main low-level C / USB-driver portion of the project.

---

## Composite USB Device

The ESP32-S3 enumerates as a **USB 2.0 Full-Speed composite device** with CDC ACM and vendor-specific interfaces.

Current endpoint layout:

| Endpoint | Direction | Type | Purpose |
|---|---|---|---|
| EP0 | IN/OUT | Control | USB enumeration and standard control requests |
| EP1 | IN | Interrupt | CDC notification endpoint |
| EP2 | OUT | Bulk | CDC host-to-device data |
| EP2 | IN | Bulk | CDC device-to-host data |
| EP3 | OUT | Bulk | Vendor host-to-device commands/data |
| EP3 | IN | Bulk | Vendor device-to-host commands/data |

The configuration descriptor is constructed explicitly in C, including the vendor-specific interface and endpoint descriptors.

---

## Recovery Command Model

The vendor interface is intended for operations that go beyond ordinary logging or console access.

| Command | Purpose | Intrusiveness |
|---|---|---|
| `PING` | Verify the diagnostic controller is alive | None |
| `IDENTIFY` | Identify the connected target | None |
| `GET STATUS` | Read current target condition | None |
| `CAPTURE STATE` | Preserve volatile state before recovery actions | Low |
| `GET FAULT CONTEXT` | Retrieve evidence related to the failure | Low |
| `DIAGNOSE` | Classify the failure using collected evidence | Low |
| `VERIFY FIRMWARE` | Check firmware/image integrity | Low |
| `RESET TARGET` | Perform a target reset | Destructive to volatile state |
| `ENTER RECOVERY` | Enter a bootloader/debug recovery path | Higher |
| `RECOVER TARGET` | Execute the selected recovery procedure | Potentially destructive |
| `GENERATE REPORT` | Produce a diagnostic/recovery report | None |

The design deliberately separates **observation** from **destructive recovery actions**. A technician should be able to preserve useful failure evidence before resetting or reflashing a target.

Several command handlers are currently scaffolding while the USB transport and target-side recovery architecture are developed.

---

## Firmware Architecture

The ESP32-S3 firmware currently includes:

```text
diagnostic_s3/
└── main/
    ├── main.c          # USB descriptors, transfers, commands, UART, FreeRTOS
    ├── tusb_config.h   # TinyUSB configuration
    ├── CMakeLists.txt
    └── idf_component.yml
```

Internally, the firmware uses FreeRTOS queues to decouple USB reception from command processing.

```text
USB RX callback
      |
      v
FreeRTOS queue
      |
      v
command parser
      |
      +----> local diagnostic response
      |
      +----> target MCU interface
```

This keeps USB callbacks short and allows recovery operations to execute outside callback context.

---

## Target MCU Interface

UART is used as the initial diagnostic-controller-to-target transport.

The abstraction is intentionally separate from USB so the recovery protocol can eventually support different target interfaces without restructuring the host-facing USB stack.

Planned target-side capabilities include:

- target identification
- health/status collection
- reset-reason retrieval
- volatile fault-context capture
- structured log retrieval
- firmware verification
- hardware reset
- bootloader/recovery entry
- firmware recovery

---

## Why Go Below the Convenience APIs?

A high-level USB API is useful when USB is only a transport. This project is specifically about understanding the transport itself.

Working closer to the device core provides visibility into:

- how USB interfaces and endpoints are created from descriptors
- how class drivers claim interfaces during enumeration
- how OUT endpoints are armed for host traffic
- how IN transfers are scheduled
- how transfer-completion callbacks drive the next transaction
- where buffering and memory copies occur
- how endpoint contention and backpressure are handled
- where recovery behavior should live when application software is unhealthy

The long-term objective is a recovery system whose important behavior can be explained from the application command all the way down to the endpoint transfer submitted to the USB controller.

---

## Current Status

### Implemented

- ESP32-S3 USB device initialization using the internal Full-Speed PHY
- explicit USB device and configuration descriptors
- composite CDC ACM + vendor-specific interfaces
- CDC notification/data endpoints
- bidirectional vendor bulk endpoints
- CDC receive callback
- vendor receive callback
- direct device-core endpoint claims and IN transfer submission
- FreeRTOS queues for received USB payloads
- vendor command parser and recovery command model
- UART initialization and basic target read/write helpers

### In Progress

- replacing the stock TinyUSB vendor class with a custom vendor device-class driver
- implementing `vendord_init`, `vendord_deinit`, `vendord_reset`, `vendord_open`, `vendord_control_xfer_cb`, and `vendord_xfer_cb`
- explicit OUT endpoint re-arming and transfer-completion handling
- target fault/status protocol
- recovery command implementations
- robust error and timeout handling

### Planned

- Linux host utility / driver-side interface for deterministic access to the vendor protocol
- structured diagnostic records instead of placeholder command responses
- firmware integrity verification
- target recovery / reflashing workflow
- fault reports containing captured state and recovery actions
- failure-injection and recovery testing

---

## Building

This is an ESP-IDF project targeting the ESP32-S3.

```bash
cd diagnostic_s3
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

USB requires the ESP32-S3 native USB interface rather than a USB-to-UART bridge.

---

## Engineering Goals

This repository is intentionally both a working recovery system and a study of low-level embedded I/O.

The main engineering goals are:

1. Build a useful independent recovery path for embedded targets.
2. Understand USB beyond high-level serial/vendor APIs.
3. Implement meaningful portions of the USB device-class path directly in C.
4. Minimize unnecessary buffering and copying where practical.
5. Make failure handling, endpoint state, and recovery behavior explicit and testable.
6. Keep the architecture modular enough to support additional target transports and recovery strategies.

The end result should demonstrate not just that the ESP32-S3 can communicate over USB, but **how the data moves through the USB device stack, where ownership changes between layers, and how that knowledge can be used to build a more deterministic embedded recovery system.**
