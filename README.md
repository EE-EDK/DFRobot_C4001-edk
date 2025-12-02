# DFRobot C4001 mmWave Radar - Raspberry Pi Pico

Refactored driver for the DFRobot C4001 mmWave radar sensor optimized for Raspberry Pi Pico with enhanced reliability features.

## Features

- **Proper Debouncing**: Time-based debouncing with configurable samples
- **UART Verification**: Multiple verification reads for reliable target detection
- **Comprehensive Configuration**: All parameters defined with clear constants
- **Debug Support**: Multi-level debug output (standard and verbose)
- **Dual Mode Support**: Motion detection (Exit Mode) and speed/ranging (Speed Mode)
- **I2C or UART**: Flexible interface selection

## Hardware Setup

### Raspberry Pi Pico Connections

**UART Mode (Default):**
```
Pico GPIO 4 (TX) --> Sensor RX
Pico GPIO 5 (RX) --> Sensor TX
Pico GND         --> Sensor GND
Pico 3.3V/5V     --> Sensor VCC
```

**I2C Mode:**
```
Pico GPIO 4 (SDA) --> Sensor SDA
Pico GPIO 5 (SCL) --> Sensor SCL
Pico GND          --> Sensor GND
Pico 3.3V/5V      --> Sensor VCC
```

## Configuration

All configuration is done via `#define` statements at the top of `C4001_RaspberryPiPico.ino`:

### Interface Selection
```cpp
#define USE_I2C_INTERFACE    0    // 0 = UART, 1 = I2C
#define UART_BAUD_RATE       115200
```

### Detection Parameters
```cpp
#define DETECTION_MIN_RANGE  30    // cm
#define DETECTION_MAX_RANGE  1000  // cm
#define TRIG_SENSITIVITY     1     // 0-9
#define KEEP_SENSITIVITY     2     // 0-9
```

### Debouncing
```cpp
#define DEBOUNCE_TIME_MS     500   // Milliseconds
#define DEBOUNCE_SAMPLES     5     // Consecutive samples
#define UART_VERIFY_SAMPLES  3     // UART verifications
```

### Debug Output
```cpp
#define ENABLE_DEBUG         1     // Enable/disable debug
#define ENABLE_VERBOSE_DEBUG 1     // Extra detailed output
```

### Operating Modes
```cpp
#define SENSOR_MODE          eExitMode  // or eSpeedMode
```

## Operating Modes

### Exit Mode (Motion Detection)
Detects human presence and motion within configured range. Outputs:
- Target presence (debounced)
- PWM signal based on detection state

### Speed Mode (Ranging & Velocity)
Measures target characteristics:
- Number of targets
- Speed (m/s)
- Range (meters)
- Energy level

## Debouncing Algorithm

The implementation uses a multi-stage debouncing approach:

1. **Time-Based**: State must be stable for `DEBOUNCE_TIME_MS`
2. **Sample-Based**: Requires `DEBOUNCE_SAMPLES` consecutive readings
3. **UART Verification** (UART mode only): Performs `UART_VERIFY_SAMPLES` additional reads

This eliminates false triggers from:
- Sensor noise
- Transient detections
- Communication errors

## File Structure

```
DFRobot_C4001-edk/
├── C4001_RaspberryPiPico.ino  # Unified example (use this!)
├── src/
│   ├── DFRobot_C4001.h        # Driver header
│   └── DFRobot_C4001.cpp      # Driver implementation
├── library.properties          # Arduino library metadata
├── keywords.txt                # Arduino IDE syntax highlighting
├── LICENSE                     # MIT License
└── README.md                   # This file
```

## Quick Start

1. **Install** the library in your Arduino IDE
2. **Open** `C4001_RaspberryPiPico.ino`
3. **Configure** the defines at the top for your setup
4. **Connect** the hardware according to the pinout
5. **Upload** to your Raspberry Pi Pico
6. **Open** Serial Monitor at 115200 baud

## Example Output

```
========================================
  DFRobot C4001 mmWave Radar - Pico
========================================

Interface: UART
UART Baud: 115200
UART RX Pin: 5
UART TX Pin: 4
Sensor Mode: Exit/Motion Detection

Initializing sensor...
Sensor connected successfully!

Configuring sensor parameters...
Setting mode to Exit Mode...
  ✓ Mode set successfully
  ✓ Detection range set
  ✓ Trigger sensitivity set

... (configuration continues)

✓ TARGET VERIFIED (UART + Debounce)
>>> MOTION DETECTED <<<
```

## Advanced Configuration

### PWM Output Configuration
Control the duty cycle of the hardware output pin:
```cpp
#define PWM_NO_TARGET_DUTY   50  // 50% when no target
#define PWM_TARGET_DUTY      0   // 0% when target present
#define PWM_TRANSITION_TIMER 10  // Transition time
```

### Timing Parameters
```cpp
#define TRIG_DELAY_UNITS     100  // 1.00s trigger delay
#define KEEP_TIMEOUT_UNITS   4    // 2.0s keep timeout
```

## Troubleshooting

**"NO Deivces !" error:**
- Check wiring connections
- Verify power supply (3.3V or 5V)
- For UART: Confirm baud rate is 115200
- For I2C: Check I2C address (default 0x2A)

**False triggers:**
- Increase `DEBOUNCE_TIME_MS`
- Increase `DEBOUNCE_SAMPLES`
- Decrease `TRIG_SENSITIVITY`
- Reduce `DETECTION_MAX_RANGE`

**No detection:**
- Increase `TRIG_SENSITIVITY`
- Check `DETECTION_MIN_RANGE` and `DETECTION_MAX_RANGE`
- Ensure target is within configured range

## License

MIT License - Copyright (c) 2010 DFRobot Co.Ltd

## Version History

- **v2.0** (2025-12-02): Refactored for Raspberry Pi Pico with enhanced debouncing
- **v1.0** (2024-02-02): Original DFRobot release

## Technical Details

**Sensor**: DFRobot C4001 mmWave Radar
**Interfaces**: I2C (0x2A/0x2B) or UART (115200 baud)
**Detection Range**: 0.3m - 20m
**Operating Voltage**: 3.3V or 5V
**Communication Protocol**: AT-command (UART) or Register-based (I2C)

---

For issues and contributions, please refer to the repository.
