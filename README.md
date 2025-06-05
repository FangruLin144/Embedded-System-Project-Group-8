# Embedded System Project – Group 8

This repository contains multiple self-contained subprojects from our second-year Embedded Systems Project (EEEN21000) at the University of Manchester.

Over the course of the academic year, our team developed and tested various embedded subsystems as part of building a microcontroller-based autonomous line-following buggy. Each subfolder under `projects/` represents an independent prototype or test module.

I am proud to team with: 
- Pengshan Liu
- Jad Merhi
- Isabella Munday
- Michael Mcdonagh



---

## Project Goal

Design, build, and test an autonomous buggy that can:
- Navigate a black-floor track using white-line sensors
- Handle slopes, tunnels and surface changes
- Use sensors to detect track end and return to start via BLE trigger
- Stop precisely within 20 cm of the track’s endpoint

---

## Repository Structure

```
Embedded-System-Project-Group-8/
│
├── projects/               # Self-contained code modules
│   ├── ESP_Encoder_Test/   # Rotary encoder test
│   ├── ESP_RACE/           # Race-day version
│   ├── ESP_Run_A_Square/   # Movement test
│   ├── ESP_TDA/            # Initial PID loop design
│   ├── ESP_TDA3_PWM/       # PWM tuning
│   ├── ESP_TDB/            # Sensor integration
│   └── ESP_TDB_BLE/        # BLE-triggered reverse sequence
│
├── docs/                   # Extended write-ups and designs (WIP)
│   └── overview.md         # System design, Gantt chart, control decisions
│
├── .gitignore
├── LICENSE
└── README.md
```

---

## Running a Subproject

Each folder under `projects/` is standalone and contains:
- Complete code for flashing to an ESP32 (or compatible board)
- No external libraries or packages required
- Pinouts and usage instructions included in `README.md` (inside each subproject)

To run:
1. Open folder in VSCode / PlatformIO / Arduino IDE
2. Flash to your development board
3. Follow serial output and sensor behavior

---

## Reports & Documentation

You can find weekly progress journals, design reports, and testing data under `docs/` (in progress). This includes:
- Line sensor characterisation
- Motor characterisation & gearbox selection
- Control algorithm selection (PID, bang-bang)
- Chassis layout and design rationale

---

## Technologies Used

- Platform: ESP32
- Language: Embedded C/C++
- Sensors: TCRT5000, Encoders, optional BLE modules
- IDEs: Arduino IDE, PlatformIO, VSCode
- Mechanical Design: SolidWorks, Acetal laser-cut chassis

---

## Team

Group 8 — 2nd Year Embedded Systems Project  
University of Manchester, Department of Electrical & Electronic Engineering

---

## License

MIT License — see `LICENSE` file for details.

---

## Acknowledgements

This project was developed as part of the EEEN21000 Embedded Systems Project, supervised by Dr. Subhasish Chakraborty and supported by Dr. Michael O'Toole and lab technical staff at the University of Manchester.
