# CLAUDE.md — DFRobot C4001 mmWave Presence Detector

## Project Summary
Enhanced presence detection firmware for the DFRobot C4001 24GHz mmWave radar sensor (SEN0609/SEN0610), optimized for Raspberry Pi Pico. Arduino framework. Fork of the official DFRobot_C4001 library with production-grade enhancements.

**Version:** 2.6.2 (Saleae-Optimized)
**Status:** Complete, production-validated. Last commit Dec 2024.

## Directory Structure
```
dfrobot-c4001/
├── examples/
│   ├── DFRobot-C4001-24GHz-UART-Enhanced-PiPico/
│   │   ├── DFRobot-C4001-24GHz-UART-Enhanced-PiPico.ino  # Main firmware
│   │   ├── README.md                                       # Example-specific docs
│   │   └── Doxygen Documentation/html/                     # Generated API docs
│   └── pico-enhanced/                                      # WIP Pico-specific variant
├── Doxyfile              # Doxygen configuration
├── keywords.txt          # Arduino IDE syntax highlighting
├── library.properties    # Arduino library metadata
├── LICENSE               # MIT License
└── README.md             # Full project documentation
```

## Build / Upload
This is an **Arduino library project**, not a CMake project.

1. Install DFRobot_C4001 library (dependency) via Arduino Library Manager
2. Install Raspberry Pi Pico board support in Arduino IDE
3. Open `examples/DFRobot-C4001-24GHz-UART-Enhanced-PiPico/DFRobot-C4001-24GHz-UART-Enhanced-PiPico.ino`
4. Select board: Raspberry Pi Pico
5. Upload; Serial Monitor at 115200 baud

## Key Technical Details
- **Sensor:** DFRobot C4001, 24 GHz mmWave, UART at 9600 baud
- **MCU:** Raspberry Pi Pico (RP2040), GP0/GP1 for UART, GP18 for LED
- **Detection range:** 0.3-2.0m (Saleae-optimized, captures 98%+ of real detections)
- **Energy corruption:** ~20% of readings are corrupted (>10M values) — automatically filtered
- **Detection pipeline:** Raw reading -> Energy validation -> Range validation -> Consecutive debouncing (2 readings) -> Optional UART verification -> Software latch (3s)
- **Loop rate:** 10 Hz (100ms period)

## Conventions
- Arduino `.ino` format (C++ with Arduino framework)
- All config parameters as `#define` at top of `.ino` file
- Doxygen-style comments (`@brief`, `@param`, `@return`)
- Uses Speed Mode as workaround (Presence Mode has known sensor firmware bugs)

## Git Notes
- Fork of upstream `DFRobot/DFRobot_C4001`
- Remotes: `origin` (user fork), `upstream` (DFRobot official)
- Branch has diverged from origin/main — may need pull/rebase before push
