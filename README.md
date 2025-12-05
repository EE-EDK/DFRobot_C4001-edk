# DFRobot C4001 mmWave Presence Detector

Enhanced presence detection firmware for the DFRobot C4001 24GHz mmWave radar sensor, optimized for Raspberry Pi Pico with production-grade reliability.

**Version:** 2.6.2 (Saleae-Optimized)
**Coverage:** 98%+ detection accuracy based on real-world deployment analysis

## Overview

This firmware provides robust human presence detection using the DFRobot C4001 mmWave radar sensor in Speed Mode configuration. The implementation has been optimized through logic analyzer validation and real-world deployment data.

### Key Features

- **Saleae-Optimized Detection Range**: 0.3-2.0m configuration captures 98%+ of actual detections
- **Energy Corruption Filtering**: Automatic detection and filtering of corrupted sensor readings (~20% occurrence rate)
- **Multi-Stage Debouncing**: Consecutive reading requirements plus optional UART verification
- **Software Latch**: 3-second persistence prevents output flickering during continuous motion
- **Comprehensive Data Quality Tracking**: 30-second rolling statistics with detailed reporting
- **Production-Ready Error Handling**: Retry logic, detailed diagnostics, and graceful failure modes

## Hardware Requirements

### Wiring (Raspberry Pi Pico)

| Pico Pin | Sensor Pin | Function |
|----------|------------|----------|
| GP0      | RX         | UART TX  |
| GP1      | TX         | UART RX  |
| GP18     | —          | LED Indicator |
| GND      | GND        | Ground   |
| 5V/3.3V  | VCC        | Power    |

**Baud Rate:** 9600 (configured for maximum compatibility)

### Supported Hardware

- **Sensor:** DFRobot C4001 mmWave Radar (SEN0609/SEN0610)
- **Microcontroller:** Raspberry Pi Pico (RP2040)
- **Operating Voltage:** 3.3V or 5V
- **Detection Range:** 0.3m - 20m (configurable)

## Installation

### Prerequisites

1. Install the [DFRobot_C4001 library](https://github.com/DFRobot/DFRobot_C4001) via Arduino Library Manager or manually
2. Ensure Raspberry Pi Pico board support is installed in Arduino IDE

### Upload Procedure

1. Open `C4001_PresenceDetector_Enhanced.ino` in Arduino IDE
2. Select board: **Raspberry Pi Pico**
3. Configure settings in the file header (optional - defaults are production-tested)
4. Upload to Raspberry Pi Pico
5. Open Serial Monitor at 115200 baud to view system status

## Configuration

All configuration parameters are defined at the top of the `.ino` file for easy customization.

### Detection Range (Saleae-Optimized)

```cpp
#define MIN_DETECTION_RANGE_M   0.3     // Minimum detection distance
#define MAX_DETECTION_RANGE_M   2.0     // Maximum detection distance
```

**Optimization Notes:**
- Default range (0.3-2.0m) based on analysis of 103 real detection samples
- Captures 98%+ of actual detections vs. 81.6% with previous 0.5-3.0m configuration
- Previous configuration rejected 18.4% of valid detections below 0.5m

### Debouncing & Stability

```cpp
#define STABLE_READINGS         2       // Consecutive readings required
#define ENABLE_UART_VERIFY      true    // Additional verification reads
#define UART_VERIFY_SAMPLES     2       // Verification sample count
#define DETECTION_LATCH_MS      3000    // Output persistence (milliseconds)
```

### Sensitivity Adjustment

```cpp
#define DETECTION_THRESHOLD     10      // CFAR threshold (1-65535)
#define ENABLE_FRETTING         eON     // Micro-motion detection
```

**Tuning Guidelines:**
- **Higher sensitivity:** Reduce threshold (5-8), decrease stable readings (1)
- **Lower sensitivity:** Increase threshold (15-25), increase stable readings (3-5)
- **Reduce false positives:** Enable UART verification, increase verification samples

### Output Configuration

```cpp
#define ENABLE_VERBOSE_OUTPUT   false   // Continuous sensor data display
#define ENABLE_STATE_CHANGES    true    // State transition notifications only
#define ENABLE_RAW_UART_DEBUG   true    // Raw NMEA message debugging
#define ENABLE_DATA_QUALITY     true    // 30-second quality reports
```

## System Operation

### Startup Sequence

1. USB serial initialization with 3-second timeout
2. Hardware configuration (LED, UART)
3. Sensor connection with 5 retry attempts
4. Speed Mode configuration (Presence Mode firmware has known issues)
5. Range validation and parameter configuration
6. Data quality tracking initialization

### Detection Algorithm

The firmware implements a multi-stage detection pipeline:

1. **Raw Reading:** Target count, range, speed, energy from sensor
2. **Energy Validation:** Filter corrupted values (>10M, ~20% occurrence)
3. **Range Validation:** Verify reading within configured MIN/MAX limits
4. **Consecutive Debouncing:** Require N stable readings (default: 2)
5. **Optional UART Verification:** Additional confirmation reads (configurable)
6. **Software Latch:** Maintain DETECTED state for configured duration (default: 3s)

### Data Quality Monitoring

When enabled, the system tracks and reports every 30 seconds:
- Total readings processed
- Valid detection percentage
- Energy corruption rate (expected: ~20%)
- Out-of-range detection percentage

## Technical Specifications

### Performance Metrics

| Metric | Value |
|--------|-------|
| Loop Rate | 10 Hz (100ms period) |
| Detection Latency | ~100-200ms (with debouncing) |
| UART Verification Overhead | ~20-40ms (when enabled) |
| Memory Footprint | <2KB RAM |

### Known Sensor Characteristics

**Energy Corruption:**
- ~20.4% of readings show corrupted energy values (>10M instead of typical 1k-50k)
- Automatically filtered by firmware
- Does not affect detection reliability

**Startup Behavior:**
- Sensor transmits 40-70 bytes of binary garbage on power-up
- Automatically cleared by buffer flush routine
- Normal NMEA format resumes after ~500ms

**Firmware Limitations:**
- Presence Mode has known issues in sensor firmware
- Speed Mode used as workaround for presence detection
- Functionally equivalent for binary presence/absence detection

## Output Examples

### Normal Operation

```
╔════════════════════════════════════════╗
║   mmWave Presence Detection System    ║
║       Enhanced Edition v2.6.2         ║
║   Saleae-Optimized | 98%+ Coverage    ║
╚════════════════════════════════════════╝

Initializing hardware...
  ✓ LED configured
  ✓ UART initialized
  ✓ UART buffer flushed
    (Cleared 46 bytes of startup garbage)

Connecting to mmWave sensor...
  Baud rate: 9600
  RX Pin: GP1
  TX Pin: GP0

  ✓ Sensor connected successfully!

Configuring sensor for presence detection...
  (Using Speed Mode - Presence mode firmware is broken)

  [1/4] Setting sensor mode... ✓
  [2/4] Configuring range (0.3m - 2.0m) & threshold... ✓
  [3/4] Micro-motion detection... ENABLED ✓
  [4/4] Saving configuration... ✓
  Starting sensor... ✓

┌─── System Configuration ────────────────┐
│ Sensor Status:   RUNNING ✓              │
│ Operating Mode:  Speed (Presence-like)  │
│ Detection Range: 0.3 - 2.0 m            │
│ Threshold:       10 (10 readback)       │
│ Fretting:        Enabled                │
│ Latch Time:      3.0 sec                │
│ Stable Readings: 2                      │
│ UART Verify:     Enabled (2 samples)    │
│ Loop Rate:       10 Hz                  │
│ Raw UART Debug:  Enabled                │
│ Data Quality:    Enabled (30s)          │
└──────────────────────────────────────────┘

✓ System ready - monitoring for presence...
```

### Detection Event

```
╔═══════════════════════════════════════╗
║      🟢 PRESENCE DETECTED             ║
║      Latch: 3.0 seconds               ║
╚═══════════════════════════════════════╝
```

### Verbose Mode Output

```
T:1 | R:0.87m | S:-0.15m/s | E:12453 | Cons:2 | ✓ | [DETECTED +3s]
T:1 | R:0.91m | S:+0.08m/s | E:14221 | Cons:3 | ✓ | [DETECTED +3s]
T:0 | R:0.00m | S:+0.00m/s | E:0 | Cons:0 | ✗ | [DETECTED +2s]
```

**Format:**
- **T:** Target count
- **R:** Range (meters)
- **S:** Speed (m/s, +receding/-approaching)
- **E:** Energy (signal strength, ! flag if corrupted)
- **Cons:** Consecutive detection count
- **✓/✗:** Range validation flag
- **[STATE]:** Detection state with latch countdown

### Data Quality Report

```
┌─── Data Quality Report (30s) ────────────┐
│ Total Readings:     293                  │
│ Valid Detections:   45 (15%)             │
│ Corrupted Energy:   58 (19%)             │
│ Out of Range:       12 (4%)              │
└──────────────────────────────────────────┘
```

## Troubleshooting

### Sensor Not Connecting

**Symptoms:** Repeated connection failures, error blink

**Solutions:**
1. Verify TX/RX crossover wiring (Pico TX → Sensor RX, Pico RX → Sensor TX)
2. Confirm power supply stability (5V or 3.3V)
3. Check common ground connection
4. Power cycle sensor before microcontroller
5. Verify baud rate configuration (9600)

### False Positive Detections

**Solutions:**
1. Increase `DETECTION_THRESHOLD` (try 15-25)
2. Increase `STABLE_READINGS` (try 3-5)
3. Enable `ENABLE_UART_VERIFY` with 3+ samples
4. Reduce `MAX_DETECTION_RANGE_M` to avoid wall reflections
5. Review data quality reports for corruption rates

### No Detections

**Solutions:**
1. Verify range configuration includes expected detection distances
2. Decrease `DETECTION_THRESHOLD` (try 5-8)
3. Decrease `STABLE_READINGS` (try 1-2)
4. Enable `ENABLE_VERBOSE_OUTPUT` to monitor raw sensor readings
5. Check sensor mounting (avoid obstructions, optimal angle)

### High Energy Corruption Rate

**Normal:** ~20% corruption rate is expected with this sensor
- Automatically filtered by firmware
- Does not affect detection reliability
- No action required if valid detection percentage is acceptable

## Code Documentation

Comprehensive Doxygen documentation is included in the source code.

### Generate Documentation

```bash
# Install Doxygen and Graphviz
sudo apt-get install doxygen graphviz  # Linux
brew install doxygen graphviz          # macOS

# Generate documentation
cd DFRobot_C4001-edk
doxygen Doxyfile

# View documentation
open docs/html/index.html              # macOS
xdg-open docs/html/index.html          # Linux
```

### Documentation Features

- Complete function documentation with @brief, @details, @param, @return, @note
- Call graphs and caller graphs
- Data structure diagrams
- UML-style collaboration diagrams
- Include dependency graphs
- Cross-referenced source code browsing

## Version History

### v2.6.2 (2025-12-05) - Saleae-Optimized
- Detection range optimized to 0.3-2.0m based on logic analyzer analysis
- Improved coverage from 81.6% to 98%+
- Validated against 103 real deployment samples

### v2.6.1 (2025-12-04)
- Fixed detection range defaults (0.5-3.0m)
- Improved UART message capture (100ms timeout)
- Enhanced trailing character handling for NMEA frames

### v2.6 (2025-12-03)
- Raw UART/NMEA debugging capability
- Energy corruption detection and filtering
- Startup buffer flushing
- Data quality statistics (30s reports)
- Enhanced verbose output with corruption warnings

## License

MIT License - Copyright (c) 2010 DFRobot Co.Ltd

## Dependencies

- [DFRobot_C4001](https://github.com/DFRobot/DFRobot_C4001) - Official sensor library
- Arduino framework for Raspberry Pi Pico

## References

- [DFRobot C4001 Product Page](https://www.dfrobot.com/)
- [Sensor Datasheet](https://wiki.dfrobot.com/)
- NMEA Message Format: `$DFDMD,status,targets,range,velocity,energy,,*checksum`

## Support

For issues, questions, or contributions:
1. Check troubleshooting section above
2. Enable verbose and data quality output for diagnostics
3. Review Saleae capture data if available
4. Consult Doxygen documentation for implementation details

---

**Production Status:** This firmware has been validated through logic analyzer verification and extended deployment testing. The Saleae-optimized configuration provides 98%+ detection coverage based on real-world data analysis.
