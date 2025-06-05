# System Overview – Embedded Systems Project (Group 8)

This document summarizes the design, implementation, and testing of our autonomous embedded system buggy, created for the EEEN21000 Embedded Systems Project at the University of Manchester.

---

## Project Objective

Design, build, and validate a two-wheeled autonomous buggy that can:
- Follow a white line on a black floor
- Navigate slopes (<18°), tunnels, and curved paths
- Accurately stop within 20 cm of the track’s end
- Reverse autonomously upon receiving a BLE signal

---

## Project Timeline

| Phase            | Activities |
|------------------|------------|
| **Semester 1**   | Chassis design, sensor and motor characterisation, control strategy planning |
| **Semester 2**   | Full integration, real-time tuning, BLE functionality, system testing, final report |

---

## System Architecture

### 1. **Hardware Components**
| Module           | Description |
|------------------|-------------|
| Microcontroller  | ESP32-WROOM |
| Motors           | 2× PMDC with fixed gearbox |
| Sensors          | 5× TCRT5000 line sensors |
| Communication    | HC-08 BLE module |
| Power Supply     | 6× AA NiMH battery pack |
| Chassis          | Custom laser-cut Acetal base |

### 2. **Submodules**
- **Line Sensor Array**: Senses white line using 5 photoreflective sensors; analog signal conditioned with op-amps.
- **Motor Driver**: Two DC motors controlled with PWM via H-bridge (L298N).
- **Encoder Feedback**: Basic rotational feedback for tuning (optional modules).
- **Control Logic**: PID controller ensures smooth tracking and accurate stopping.
- **BLE Interface**: Waits for return-trigger signal after track completion.

---

## Control Strategy

The buggy uses:
- **Line deviation** (based on sensor readings) as input to the PID controller
- **PWM modulation** to adjust motor speeds and steer accurately
- A **slope detection fallback mechanism** to enhance stability during incline traversal
- **Track-end detection** via middle sensor alignment and drop in total IR reflection

The PID constants were tuned empirically for stable tracking and minimal overshoot. A fallback bang-bang controller was implemented during early testing for comparison.

---

## Design Highlights

- **Custom Chassis**: Compact, symmetric layout minimizing sensor-motor offset. Center of mass optimized for slopes.
- **Sensor Characterisation**: TCRT5000s chosen for their fast response and low-angle reflectivity. Calibrated at 3 cm height.
- **BLE Logic**: Reverse sequence initiated using a BLE-trigger after the buggy stops at the track end. BLE trigger also used for debugging.

---

## Testing Strategy

- Each subproject (under `/projects/`) was used for modular testing:
  - `ESP_Run_A_Square`: Verifies movement accuracy
  - `ESP_TDA3_PWM`: Tunes PID and PWM behavior
  - `ESP_Encoder_Test`: Tests encoder-based feedback
  - `ESP_TDB_BLE`: Implements full BLE-triggered return

- Stress-tested on curves, partial lighting, and carpeted tunnel segments
- Final demo met accuracy criteria for stopping distance and track coverage

---

## Gantt Chart and Journals

Project planning included:
- Proposal and timeline submitted in Semester 1
- Weekly journals tracking task completion and delays
- Final integration and test weeks focused on stress-testing and refinement
