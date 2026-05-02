# ESP32 OBD-II WiFi Diagnostic Tool

![Platform](https://img.shields.io/badge/platform-ESP32-blue.svg)
![Framework](https://img.shields.io/badge/framework-Arduino-00979C.svg)
![Protocol](https://img.shields.io/badge/OBD--II-ISO--15765--4-green.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

## Dashboard Preview

<div align="center">
  <img src="Figures/Testing/Testing_upload_version_02_05/DASH_auto_stoji_motor_off.png" alt="Main Dashboard" width="450">
</div>



An ESP32-based wireless OBD-II diagnostic scanner. This project implements the OBD-II standard (ISO 15765-4 via CAN bus) and provides a real-time web-based dashboard accessible over Wi-Fi using WebSockets.

This project was developed as a part of a Bachelor's thesis implementation.

## Features

- **Wireless Connectivity:** Creates its own Wi-Fi Access Point (AP). No external network required.
- **Real-Time Web Dashboard:** Built-in dashboard served directly from ESP32 flash memory (`PROGMEM`), interacting with the scanner via WebSockets.
- **Dual-Core Architecture:**
  - **Core 0** handles network operations (Wi-Fi, HTTP Server, WebSockets) to ensure smooth communication.
  - **Core 1** handles OBD-II CAN bus communication using FreeRTOS tasks, avoiding interference with the Wi-Fi stack.
- **Thread-safe FreeRTOS Integration:** Safely passes data between cores using FreeRTOS queues.
- **Protocols Supported:** ISO-TP transport protocol over CAN, implementing standard OBD-II services.

### Key Features Showcase

| Comprehensive Mode Support | Emission Readiness Monitors |
| :---: | :---: |
| ![Diagnostic Modes](Figures/Testing/Testing_upload_version_02_05/DIAG_VYBER_MODES.png) | ![Monitor Status](Figures/Testing/Testing_upload_version_02_05/DIAG_MODE_1_Monitor_Status.png) |
| **All Essential Services** <br> Full support for ISO 15031-5 / SAE J1979 diagnostic modes (01-0A). | **Monitor Status** <br> Clear, real-time overview of continuous and non-continuous emission monitors. |

| Vehicle Configuration | Raw CAN & ISO-TP Inspector |
| :---: | :---: |
| ![Vehicle Configuration](Figures/Testing/Testing_upload_version_02_05/CFG_NOT_OPTIMALISED.png) | ![Raw Data](Figures/Testing/Testing_upload_version_02_05/PID_inspector_RAW.png) |
| **Custom Vehicle Profiles** <br> Adjustable engine parameters to tailor internal calculations to specific vehicle types. | **Developer Raw Mode** <br> Real-time deep dive into raw CAN hex frames and ISO-TP payload details. |


## Hardware Requirements

- **Microcontroller:** ESP32 (Any standard development board, e.g., ESP32 DevKit V1)
- **CAN Transceiver:** SN65HVD230 (3.3V compatible) or similar, connected to GPIO 15 (TX) and GPIO 13 (RX).
- **Power Supply:** A DC-DC step-down converter (e.g., LM2596) to regulate the 12V from the vehicle's OBD-II port (Pin 16) down to a safe 5V for the ESP32 `VIN` pin.
- **Connector:** Standard OBD-II Male Connector.

## Technology Stack & Architecture

This architecture was specifically designed to prevent the time-critical CAN bus communication from interfering with the Wi-Fi stack. The system is logically separated into multiple protocol layers running asynchronously across both ESP32 processor cores:

1. **Web & Network Layer (Execution on Core 0):**
   - Manages Wi-Fi Access Point creation, serving HTTP requests, and maintaining live WebSocket connections.
   - Built on top of `ESPAsyncWebServer` for non-blocking network operations.
   - Converts user interface commands to internal triggers and formats outgoing responses using `ArduinoJson`.

2. **OBD-II Application Layer (Execution on Core 1):**
   - The primary diagnostic layer running in a dedicated FreeRTOS task.
   - Safely blocks and waits for CAN responses without triggering Watchdog Timer (WDT) resets on the network stack.
   - Implements the full set of emission-related diagnostic services defined by ISO 15031-5:
     - **Mode 01** — Current powertrain data (live PIDs)
     - **Mode 02** — Freeze frame snapshot at the moment of fault detection
     - **Mode 03** — Confirmed (stored) Diagnostic Trouble Codes
     - **Mode 04** — Clear DTCs and reset emission readiness monitors
     - **Mode 07** — Pending DTCs from the current/last drive cycle
     - **Mode 09** — Vehicle information (VIN, CalID, CVN, IPT, ECU name)
     - **Mode 0A** — Permanent DTCs (immune to Mode 04 clearing)
   - Captures Negative Response Codes (NRC) including `responsePending` (0x78) for diagnostics.
   - Supports ECU discovery via broadcast (0x7DF) and per-ECU binding for unicast queries.

3. **Transport Layer - ISO-TP (Execution on Core 1):**
   - Implements the ISO 15765-2 network protocol.
   - Responsible for fragmenting large outgoing payloads into multiple 8-byte CAN frames and reassembling incoming multi-frame messages using standard Flow Control (FC) logic.

4. **Data Link Layer - TWAI (Execution on Core 1):**
   - Interfaces directly with the ESP32's built-in Two-Wire Automotive Interface (TWAI) peripheral.
   - Drives the external physical CAN transceiver (e.g., SN65HVD230).

**Task Communication:** Data is safely passed between the network handler (Core 0) and the diagnostic task (Core 1) using standard FreeRTOS queues (`xQueueSend` / `xQueueReceive`).

### File Overview

A breakdown of the core project structure, detailing the specific role of each module:

- **`src/config/config.h`**: Centralized configuration file containing all user-adjustable parameters (CAN pins, baud rates, Wi-Fi channels, FreeRTOS task sizes, and ISO-TP/OBD-II timeouts).
- **`src/config/secrets.h`**: Secure storage for sensitive credentials such as Wi-Fi passwords and WebSocket authentication tokens. Designed to be easily excluded from version control.
- **`obdII_wifi.ino`**: The main Arduino sketch serving as the entry point. Responsible for overall system configuration, initializing FreeRTOS queues, mapping tasks to specific processor cores, and configuring the Wi-Fi AP.
- **`src/web/ws_handler.h` / `.cpp`**: The WebSocket messaging infrastructure. It handles asynchronous event dispatching, parses incoming JSON payloads from the client, and safely routes them to the OBD task.
- **`src/web/ws_commands.cpp`**: The implementation of all individual WebSocket OBD command handlers. It explicitly maps string commands (like `"read_pid"`) to their respective core OBD-II functions.
- **`src/core/obd2.h` / `.c`**: The core API for vehicle system interaction. It manages the top-level ECU request/response state machine and payload validation.
- **`src/core/obd2_pids.c`**: Responsible for reading and decoding Parameter IDs for Mode 01 (Live Data) and Mode 02 (Freeze Frame). Contains the mathematical conversions to turn raw hexadecimal CAN payloads into human-readable engineering units (e.g., RPM, Temperature).
- **`src/core/obd2_diag.c`**: Dedicated module for reading Diagnostic Trouble Codes (DTCs), decoding emission readiness statuses, and clearing fault records.
- **`src/core/obd2_internal.h`**: Shared internal definitions and data structures for the OBD-II routing logic.
- **`src/isotp/isotp.h` / `.c`**: The ISO-TP protocol module implementation. Contains standard algorithms for parsing Single Frames (SF), First Frames (FF), Consecutive Frames (CF), and Flow Control (FC) messaging.
- **`src/web/dashboard.h`**: A complete Single-Page Application (SPA) web dashboard built with HTML, CSS, and JS. It is stored directly in the ESP32 `PROGMEM` flash storage to preserve runtime heap memory.

## Testing & Validation (PC-based Unit Tests)

To ensure the reliability of the diagnostic stack, the project includes a comprehensive suite of **153 unit tests** that can be executed on a standard PC. This approach allows for rapid development and verification of the protocol logic without needing physical access to a vehicle or an ESP32 board.

### Architecture
The testing environment is located in the `unit_tests_pc/` directory and utilizes a **Mock Hardware Layer**. This layer simulates the ESP32's TWAI (CAN) peripheral and FreeRTOS timing functions, allowing the real C source code from `src/` to be compiled and executed on an x86/x64 architecture without modification.

The mock simulates time, RX/TX queues and error states (BUS_OFF, recovery, alerts), so the entire stack — including timeouts and retry logic — is exercised deterministically without waiting on real hardware.

### Test Suites (`tests/`)
- **`test_isotp.c` — ISO-TP transport layer (ISO 15765-2)**
  - Frame types: Single Frame, First Frame, Consecutive Frame, Flow Control
  - Multi-frame edge cases: SN wrap-around (>15 CFs), FF_DL=0, overflow, sequence error, CF timeout
  - Buffer-overflow protection (sentinel test) against malicious oversized payloads
  - Broadcast (0x7DF) with multiple ECU responses and ID-range filtering (0x7E8–0x7EF boundaries)
  - BUS_OFF detection with full recovery sequence (success and failure paths)
  - N_As TX timeout handling (distinct from BUS_OFF)
  - Idempotent `init`/`deinit` and Flow Control padding correctness
- **`test_obd2_pids.c` — PID decoding (Annex B coverage)**
  - All 14 decoder formats: `LINEAR_1B/2B/4B`, `SIGNED_OFFSET_1B`, `SIGNED_2B`, `BIT_ENCODED`, `O2_CONV`, `O2_WIDE_EQ_V/I`, `CONFIG`, `ENUM`, `TEMP_4S`, `NOX_4S`, `RAW`
  - Multi-sensor PIDs (PID $78 EGT 4 sensors, $83 NOx) with support-bit and 0xFFFF invalid handling
  - Monitor status (PID $01) MIL/DTC count and continuous/non-continuous monitor parsing
- **`test_obd2_diag.c` — DTC string decoder (SAE J2012)**
  - All four prefixes (P/C/B/U), boundary values, manufacturer-specific bit, NULL safety
- **`test_obd2.c` — OBD-II integration (Modes 01/03/04/09)**
  - Mode 01 PID raw + decoded happy paths, NRC capture, echo mismatch, timeouts
  - Mode 03 Read DTC: zero/two/five DTCs (single and multi-frame), buffer truncation safety
  - Mode 04 Clear DTC: success and `conditionsNotCorrect` rejection
  - Mode 09 VIN multi-frame and ECU name SF, malformed-input edge cases
  - Lifecycle: init/deinit idempotency, NULL guards
- **`test_obd2_modes.c` — Extended services and state machine**
  - Mode 02 Freeze Frame (happy path, PID mismatch detection, NULL guards)
  - Mode 07 Pending DTC and Mode 0A Permanent DTC
  - NRC `responsePending` (0x78) capture and decoding
  - ECU binding state machine (`set_ecu_address`, `bind_active_ecu`, range validation)
  - PID discovery via broadcast (`query_supported_pids`, `is_pid_supported`)
  - Multi-ECU aggregators (`read_dtc_multi`, `read_vin_all`, `read_infotype_all`)
  - Raw query API (unicast vs broadcast variants)
- **`test_main.c`** — Central runner that executes all five suites and reports pass/fail.

### Hardware Mocks (`mocks/`)
- **`mock_twai.c` / `mock_twai.h`** — Simulates TWAI driver: `twai_transmit/receive/get_status_info/initiate_recovery/read_alerts`. Supports injecting RX frames, capturing TX frames, advancing simulated time, forcing TX failures, and forcing recovery to either succeed or stay in BUS_OFF.
- **`driver/twai.h`, `freertos/*.h`, `esp_err.h`** — ESP-IDF header replacements providing only the symbols the stack actually uses.

### Interactive Interpreter (`interpreter/`)
- **`obd_interpreter.c`** — A CLI tool that lets developers inject raw CAN hexadecimal frames and observe how the OBD-II stack parses them in real-time.
- **`full_test_sequence_v2.txt`** — Recorded ECU response sequences used for validation and regression testing against real-world traces.

### Running the Tests
The test suite can be built using either **CMake** (for IDE integration) or a **Bash script** (for quick CLI use).

#### Option A: Using CMake
```bash
cd unit_tests_pc
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
./build/run_tests.exe    # Run automated test suite
./build/obd_interp.exe   # Run interactive interpreter
```

#### Option B: Using Bash Script (requires GCC)
```bash
cd unit_tests_pc
./build.sh               # Compile only
./build.sh run           # Compile and execute all tests
./build.sh memcheck      # Run tests with AddressSanitizer (ASan) for leak/error detection
```

The expected output ends with `=== 153 tests, 0 failed ===` on a clean run.

## Dependencies

To compile this project in the Arduino IDE, you will need to install the following libraries via the Library Manager (`Sketch -> Include Library -> Manage Libraries...`) or by downloading them manually:

1. [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer)
2. [AsyncTCP](https://github.com/ESP32Async/AsyncTCP)
3. [ArduinoJson (v7.x)](https://arduinojson.org/)

*Note: You must have the standard ESP32 board support package installed in your Arduino IDE.*

## Installation and Flashing

1. Clone or download this repository.
2. Open `obdII_wifi.ino` in the Arduino IDE.
3. Select your ESP32 board (`Tools > Board > ESP32 Dev Module`).
4. Install all the required dependencies listed above.
5. Connect your ESP32 via USB and select the correct COM port.
6. Click **Upload** to flash the firmware to the ESP32.

## Usage

1. Plug the constructed device into your vehicle's OBD-II port.
2. Turn on the vehicle's ignition.
3. Wait ~5 seconds for the boot sequence, then connect your phone, tablet, or laptop to the Wi-Fi network:
   - **SSID:** As configured in `WIFI_SSID` (default: `OBD2-Diagnostics`)
   - **Password:** As configured in `WIFI_PASSWORD`
4. Open a web browser and navigate to `http://192.168.4.1`.
5. The web dashboard will load and automatically attempt to establish a WebSocket connection. Once connected, interact with the UI to initialize CAN communication and start reading real-time engine data.

## Testing & Validation

Comprehensive testing was conducted to ensure protocol stability and UI responsiveness. Detailed visual documentation and technical evidence of these tests are available in the repository:

- **[Latest Testing Gallery (May 2nd, 2026)](Figures/Testing/Testing_upload_version_02_05/)** – Complete set of screenshots from the latest stable version, covering all diagnostic modes, real-time telemetry, and system logs.
- **[Testing Archive](Figures/Testing/)** – Historical testing data and earlier development versions used for validation during the development process.

These screenshots serve as technical evidence of the implementation's functionality.


## License

This project is licensed under the [MIT License](LICENSE).

Note: This project was originally developed as a Bachelor's thesis implementation. While it is open-source, it is provided as-is for educational and diagnostic purposes. Use at your own risk on real vehicles.