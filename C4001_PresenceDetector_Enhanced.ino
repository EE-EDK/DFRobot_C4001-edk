/*!
 * @file    C4001_PresenceDetector_Enhanced.ino
 * @brief   Enhanced mmWave Presence Detection - Best of Both Worlds
 * @details Combines reliable presence detection with robust debouncing
 *          - Uses Speed Mode (Presence mode firmware is broken)
 *          - Software latch for stable detection
 *          - LED indicator with optional inversion
 *          - UART verification for reliability
 *          - Comprehensive configuration via #defines
 *
 * @note    IMPORTANT LIMITATIONS (from sensor manual):
 *          - Distance measurements are NOT calibrated (reference only)
 *          - 1m transition zone exists beyond MAX_DETECTION_RANGE_M
 *          - Sensitivity commands (setSensitivity) only work in Presence mode
 *          - Micro-motion (setMicroMotion) only works in Speed mode
 *          - Default baud rate varies: manual says 9600, protocol says 115200
 *
 * @see     Communication Protocol PDF - Section 1 (Configuration Commands)
 * @see     Communication Protocol PDF - Section 1.7 (Micro-motion for Speed mode)
 * @see     Sensor Manual PDF - Technical Specifications & Limitations
 * @see     Sensor Manual PDF - Section 1.1 (Distance measurement disclaimer)
 *
 * @author  Ethan + Claude Enhanced
 * @date    2025-12-03
 * @version 2.6 - NEW FEATURES:
 *          - Raw UART/NMEA message debugging (see actual sensor output)
 *          - Energy value corruption detection & filtering
 *          - Startup buffer flushing (clears sensor garbage data)
 *          - Data quality statistics (30s reports)
 *          - Enhanced verbose output with corruption warnings
 *
 *          Based on real Saleae captures showing:
 *          - Binary garbage at startup: \x08\xC23\x9F\xDD\xF5\xFF\xFF
 *          - Corrupted energy values (99M+ instead of normal 1k-50k)
 *          - NMEA format: $DFDMD,status,targets,range,velocity,energy, , *
 *
 * @author  Ethan + Claude Enhanced
 * @date    2025-12-05
 * @version 2.6
 */

#include "src/DFRobot_C4001.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// --- Pin Assignments (Raspberry Pi Pico) ---
#define PIN_STATUS_LED          18      // LED indicator pin
#define PIN_UART_RX             1       // GP1 - Connect to sensor TX
#define PIN_UART_TX             0       // GP0 - Connect to sensor RX

// --- LED Behavior ---
#define INVERT_LED_LOGIC        false   // true if LED is active LOW
#define LED_BLINK_ON_ERROR      true    // Blink LED if sensor fails

// ============================================================================
// COMMUNICATION CONFIGURATION
// ============================================================================

#define BAUD_DEBUG              115200  // USB Serial for debugging

// --- Sensor UART Baud Rate ---
// Note: Sensor manual specifies 9600 default, but protocol document mentions 115200.
// If communication fails, try 115200 or use setUart command to configure.
// See Communication Protocol PDF Section 1.6 (setUart/getUart commands)
#define BAUD_SENSOR             9600    // Sensor UART (9600 is most compatible)
                                        // Alternative: 115200 (if 9600 fails)
                                        // Range: 4800-115200 bps

#define SERIAL_TIMEOUT_MS       3000    // Time to wait for USB serial

// ============================================================================
// DETECTION RANGE CONFIGURATION (meters)
// ============================================================================

// ⚠️ CRITICAL: Distance measurements are NOT calibrated and may have errors!
//    The C4001 is NOT designed for precise distance measurement.
//    Use distance values as REFERENCE ONLY, not for accurate measurements.
//    See Sensor Manual Section 1.1 for details.

// ⚠️ 1-METER TRANSITION ZONE: The sensor has a 1m transition zone beyond MAX range.
//    Example: If MAX_DETECTION_RANGE_M = 6.0m, the sensor may still detect
//    movement at 6-25m (requires significant movement). This is NORMAL behavior.
//    Configure MAX range to accurately define your desired detection area.
//    See Sensor Manual Section 1.1 (setRange) for details.

// Smart range filtering to avoid false triggers
#define MIN_DETECTION_RANGE_M   2.0     // Ignore closer objects (walls, furniture)
                                        // Minimum: 0.6m (sensor limit)
                                        // Recommended: 2.0m+ to avoid walls
#define MAX_DETECTION_RANGE_M   6.0     // Maximum useful detection distance
                                        // Maximum: 25m (sensor limit)
                                        // Note: +1m transition zone applies

// Note: Using 2.0m minimum prevents triggering on nearby walls/objects
// Adjust based on your room size and sensor mounting
// See Communication Protocol PDF Section 1.1 (setRange/getRange commands)

// ============================================================================
// DETECTION BEHAVIOR CONFIGURATION
// ============================================================================

// --- Sensor Sensitivity / Threshold Factor ---
// Note: In Speed mode, this maps to setThrFactor (Protocol Section 1.8)
//       Library method: setDetectThres() appears to abstract this
//       Protocol: Threshold factor (default 5) - larger value = bigger movements needed
//       Library: Threshold (1-65535) - lower value = more sensitive
// See Communication Protocol PDF Section 1.8 (setThrFactor/getThrFactor)
#define DETECTION_THRESHOLD     10      // Lower = more sensitive (1-65535)
                                        // Recommended: 10-20 for presence
                                        // Higher values = fewer false positives
                                        // Protocol equivalent: factor ~5

// --- Debouncing & Stability ---
#define STABLE_READINGS         2       // Consecutive detections required (2-5 recommended)
#define ENABLE_UART_VERIFY      true    // Extra UART verification (reduces false positives)
#define UART_VERIFY_SAMPLES     2       // Number of verification reads (if enabled)

// --- Software Latch (prevents flickering) ---
#define DETECTION_LATCH_MS      3000    // Stay ON for 3 seconds after last detection
                                        // Prevents rapid ON/OFF cycling
                                        // Adjust: 2000-5000ms typical

// --- Micro-motion Detection (Speed Mode Only) ---
// Protocol command: setMicroMotion (Section 1.7)
// Library method: setFrettingDetection()
// ⚠️ This setting ONLY works in Speed/Distance measurement mode
// See Communication Protocol PDF Section 1.7 (setMicroMotion/getMicroMotion)
#define ENABLE_FRETTING         eON     // eON = detect small movements (recommended)
                                        // eOFF = only detect larger movements
                                        // Only effective in Speed mode (current)

// --- Energy Value Filtering ---
#define MAX_VALID_ENERGY        10000000 // Filter out absurdly large energy values
                                         // Values > this are likely corrupted data
                                         // Normal range: 1000-50000, but allow headroom

// ============================================================================
// OUTPUT & DISPLAY CONFIGURATION
// ============================================================================

// --- Debug Output Modes ---
#define ENABLE_VERBOSE_OUTPUT   false   // true = show every sensor reading
#define ENABLE_STATE_CHANGES    true    // true = show only when state changes
#define ENABLE_DETAILED_STARTUP true    // true = show full config on startup
#define ENABLE_RAW_UART_DEBUG   true    // true = show raw NMEA messages from sensor
#define ENABLE_DATA_QUALITY     true    // true = track and show data quality statistics

// --- Status Update Frequency (when verbose) ---
#define VERBOSE_PRINT_EVERY_N   10      // Print every Nth reading (reduce spam)
#define RAW_UART_PRINT_EVERY_N  5       // Print raw UART every Nth message (if enabled)

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#define LOOP_DELAY_MS           100     // Main loop delay (100ms = 10Hz update rate)
#define ERROR_BLINK_MS          200     // LED blink speed on error
#define SENSOR_STARTUP_DELAY_MS 2000    // Wait time after sensor config changes

// ============================================================================
// ADVANCED OPTIONS (Normally don't need to change)
// ============================================================================

// --- Operating Mode Selection ---
// 0 = Presence Detection mode (uses sensitivity, latency, trigger distance)
// 1 = Speed/Distance mode (uses threshold factor, micro-motion)
// Note: Currently using Speed mode because Presence mode firmware has issues
// See Communication Protocol PDF Section 2.6 (setRunApp)
#define SENSOR_MODE             1       // 0=Presence, 1=Speed (currently must be 1)

// --- Sensitivity Settings (Presence Mode Only) ---
// ⚠️ These commands ONLY work in Presence Detection mode (SENSOR_MODE = 0)
// See Communication Protocol PDF Section 1.3 (setSensitivity/getSensitivity)
// Hold sensitivity: sensitivity after sensor is triggered (0-9, default 7)
// Trigger sensitivity: ease of initial trigger (0-9, default 5)
// Higher = more sensitive, Lower = requires larger movement
// ⚠️ WARNING: NOT available in Speed mode - use DETECTION_THRESHOLD instead
#define HOLD_SENSITIVITY        7       // 0-9 (default 7) - Presence mode only
#define TRIGGER_SENSITIVITY     5       // 0-9 (default 5) - Presence mode only
                                        // Recommended: 2-6 to avoid false alarms

// --- Trigger Distance (Presence Mode Only) ---
// ⚠️ This command ONLY works in Presence Detection mode (SENSOR_MODE = 0)
// Defines distance to transition from "no one" to "someone"
// Helps reduce false triggers at detection edge
// Example: MAX=10m, TRIG=6m → only triggers when within 6m
// See Communication Protocol PDF Section 1.2 (setTrigRange/getTrigRange)
#define TRIGGER_RANGE_M         6.0     // 0-25m (default 6m) - Presence mode only

// --- Hardware Latency (Presence Mode Only) ---
// ⚠️ These settings ONLY work in Presence Detection mode (SENSOR_MODE = 0)
// Note: These are HARDWARE delays in the sensor itself (separate from software latch)
// Confirmation: delay before OUT pin goes HIGH (reduces false positives)
// Disappearance: delay before OUT pin goes LOW (reduces missed detections)
// See Communication Protocol PDF Section 1.4 (setLatency/getLatency)
#define USE_HARDWARE_LATENCY    false   // Set true to use sensor's built-in latency
#define CONFIRM_LATENCY_S       0.5     // 0-100s (default 0.050s) - Presence mode only
#define DISAPPEAR_LATENCY_S     15.0    // 0.5-1500s (default 15s) - Presence mode only
                                        // Recommended: 0.5s+ confirm, 15s+ disappear

// Convert meters to centimeters for sensor API
#define MIN_DETECTION_RANGE_CM  ((uint16_t)(MIN_DETECTION_RANGE_M * 100))
#define MAX_DETECTION_RANGE_CM  ((uint16_t)(MAX_DETECTION_RANGE_M * 100))

// Note: Range validation happens at runtime in configureSensor()
// Preprocessor #if can't use floating-point constants, so we validate in code instead

// ============================================================================
// GLOBAL OBJECTS & STATE
// ============================================================================

DFRobot_C4001_UART radarSensor(&Serial1, BAUD_SENSOR, PIN_UART_RX, PIN_UART_TX);

// Detection states
enum class DetectionState : uint8_t {
    NO_TARGET,
    DETECTED
};

// Sensor reading structure
struct SensorReading {
    uint8_t targetCount;
    float targetRange;      // meters
    float targetSpeed;      // m/s (negative = approaching)
    uint32_t targetEnergy;
    bool valid;             // Reading is within configured range
    bool energyCorrupted;   // Energy value exceeds reasonable limits
};

// Data quality tracking
struct DataQuality {
    uint32_t totalReadings;
    uint32_t validReadings;
    uint32_t corruptedEnergy;
    uint32_t outOfRange;
    uint32_t detectionCount;
    unsigned long lastReportTime;
};

// State tracking variables
DetectionState currentState = DetectionState::NO_TARGET;
DetectionState previousState = DetectionState::NO_TARGET;
unsigned long lastDetectionTime = 0;
uint8_t consecutiveDetections = 0;
uint8_t consecutiveClears = 0;
uint32_t loopCounter = 0;
uint32_t rawUartCounter = 0;

// Data quality tracking
DataQuality dataQuality = {0, 0, 0, 0, 0, 0};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSystem(void);
void initializeHardware(void);
void initializeSensor(void);
void configureSensor(void);
void flushUARTBuffer(void);
void showConfigSummary(void);
SensorReading readSensor(void);
bool verifyDetectionUART(void);
DetectionState evaluateDetection(const SensorReading& reading);
void updateLED(DetectionState state);
void errorBlinkLED(void);
void printStateChange(DetectionState state);
void printVerboseStatus(const SensorReading& reading, DetectionState state);
void printRawUART(void);
void printDataQualityReport(void);
void printBanner(void);
void printSeparator(void);

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

void setup() {
    initializeSystem();
}

void loop() {
    // Optional: Print raw UART data for debugging
    if (ENABLE_RAW_UART_DEBUG) {
        if (rawUartCounter % RAW_UART_PRINT_EVERY_N == 0) {
            printRawUART();
        }
        rawUartCounter++;
    }

    // Read sensor data
    SensorReading reading = readSensor();

    // Evaluate detection with debouncing
    currentState = evaluateDetection(reading);

    // Update LED output
    updateLED(currentState);

    // Print status based on configuration
    if (ENABLE_VERBOSE_OUTPUT) {
        if (loopCounter % VERBOSE_PRINT_EVERY_N == 0) {
            printVerboseStatus(reading, currentState);
        }
    } else if (ENABLE_STATE_CHANGES && currentState != previousState) {
        printStateChange(currentState);
        previousState = currentState;
    }

    // Print data quality report periodically
    if (ENABLE_DATA_QUALITY) {
        unsigned long now = millis();
        if (now - dataQuality.lastReportTime > 30000) { // Every 30 seconds
            printDataQualityReport();
            dataQuality.lastReportTime = now;
        }
    }

    loopCounter++;
    delay(LOOP_DELAY_MS);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initializeSystem(void) {
    // Start USB serial for debugging
    Serial.begin(BAUD_DEBUG);

    // Wait for serial with timeout (don't block if no USB)
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < SERIAL_TIMEOUT_MS)) {
        delay(10);
    }

    printBanner();

    // Initialize hardware (LED, UART)
    initializeHardware();

    // Initialize and configure sensor
    initializeSensor();
    configureSensor();

    // Display configuration
    if (ENABLE_DETAILED_STARTUP) {
        showConfigSummary();
    }

    printSeparator();
    Serial.println(F("✓ System ready - monitoring for presence..."));
    printSeparator();
    Serial.println();

    // Initialize timing
    lastDetectionTime = millis();
}

void initializeHardware(void) {
    Serial.println(F("Initializing hardware..."));

    // Setup LED
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, INVERT_LED_LOGIC ? HIGH : LOW);
    Serial.println(F("  ✓ LED configured"));

    // Setup sensor UART
    Serial1.begin(BAUD_SENSOR);
    delay(200);
    Serial.println(F("  ✓ UART initialized"));

    // Flush any garbage data from sensor startup
    flushUARTBuffer();
    Serial.println(F("  ✓ UART buffer flushed"));

    Serial.println();
}

void initializeSensor(void) {
    Serial.println(F("Connecting to mmWave sensor..."));
    Serial.print(F("  Baud rate: "));
    Serial.println(BAUD_SENSOR);
    Serial.print(F("  RX Pin: GP"));
    Serial.println(PIN_UART_RX);
    Serial.print(F("  TX Pin: GP"));
    Serial.println(PIN_UART_TX);
    Serial.println();

    uint8_t attempts = 0;
    const uint8_t maxAttempts = 5;

    while (!radarSensor.begin()) {
        attempts++;
        Serial.print(F("  Connection attempt "));
        Serial.print(attempts);
        Serial.print(F("/"));
        Serial.println(maxAttempts);

        if (attempts >= maxAttempts) {
            Serial.println();
            Serial.println(F("╔════════════════════════════════════════╗"));
            Serial.println(F("║  ✗ ERROR: Cannot connect to sensor!   ║"));
            Serial.println(F("╚════════════════════════════════════════╝"));
            Serial.println();
            Serial.println(F("Troubleshooting:"));
            Serial.println(F("  1. Check wiring:"));
            Serial.println(F("     - Pico GP0 (TX) → Sensor RX"));
            Serial.println(F("     - Pico GP1 (RX) → Sensor TX"));
            Serial.println(F("     - GND → GND"));
            Serial.println(F("     - 5V → VCC (or 3.3V depending on sensor)"));
            Serial.println();
            Serial.println(F("  2. Check power supply"));
            Serial.println(F("  3. Try power cycling the sensor"));
            Serial.println(F("  4. Verify sensor is C4001 model"));
            Serial.println();

            if (LED_BLINK_ON_ERROR) {
                errorBlinkLED();  // Never returns
            } else {
                while (1) { delay(1000); }
            }
        }
        delay(1000);
    }

    Serial.println(F("  ✓ Sensor connected successfully!"));
    Serial.println();
}

void configureSensor(void) {
    Serial.println(F("Configuring sensor for presence detection..."));
    Serial.println(F("  (Using Speed Mode - Presence mode firmware is broken)"));
    Serial.println();

    // ============ Configuration Validation ============
    bool configErrors = false;

    Serial.println(F("Validating configuration..."));

    // Range validation
    if (MIN_DETECTION_RANGE_M >= MAX_DETECTION_RANGE_M) {
        Serial.println(F("  ✗ ERROR: MIN_DETECTION_RANGE_M must be < MAX_DETECTION_RANGE_M"));
        configErrors = true;
    }
    if (MIN_DETECTION_RANGE_M < 0.6) {
        Serial.println(F("  ⚠ Warning: MIN range < 0.6m (sensor minimum)"));
    }
    if (MAX_DETECTION_RANGE_M > 25.0) {
        Serial.println(F("  ⚠ Warning: MAX range > 25m (sensor maximum)"));
    }

    // Latch validation
    if (DETECTION_LATCH_MS < 500) {
        Serial.println(F("  ⚠ Warning: DETECTION_LATCH_MS < 500ms may cause flickering"));
    }

    // Stable readings validation
    if (STABLE_READINGS < 1) {
        Serial.println(F("  ✗ ERROR: STABLE_READINGS must be >= 1"));
        configErrors = true;
    }

    // Threshold validation
    if (DETECTION_THRESHOLD < 1 || DETECTION_THRESHOLD > 65535) {
        Serial.println(F("  ✗ ERROR: DETECTION_THRESHOLD must be 1-65535"));
        configErrors = true;
    }

    // Mode validation
    if (SENSOR_MODE != 0 && SENSOR_MODE != 1) {
        Serial.println(F("  ✗ ERROR: SENSOR_MODE must be 0 (Presence) or 1 (Speed)"));
        configErrors = true;
    }

    if (configErrors) {
        Serial.println();
        Serial.println(F("✗ Configuration errors detected! Fix defines and restart."));
        while (1) { delay(1000); }
    }

    Serial.println(F("  ✓ Configuration validation passed"));
    Serial.println();

    // Step 1: Set to Speed/Range mode
    Serial.print(F("  [1/4] Setting sensor mode... "));
    if (radarSensor.setSensorMode(eSpeedMode)) {
        Serial.println(F("✓"));
    } else {
        Serial.println(F("✗ (continuing anyway)"));
    }
    delay(500);

    // Step 2: Set detection threshold and range
    Serial.print(F("  [2/4] Configuring range ("));
    Serial.print(MIN_DETECTION_RANGE_M, 1);
    Serial.print(F("m - "));
    Serial.print(MAX_DETECTION_RANGE_M, 1);
    Serial.print(F("m) & threshold... "));

    if (radarSensor.setDetectThres(MIN_DETECTION_RANGE_CM, MAX_DETECTION_RANGE_CM, DETECTION_THRESHOLD)) {
        Serial.println(F("✓"));
    } else {
        Serial.println(F("✗ (continuing anyway)"));
    }
    delay(500);

    // Step 3: Enable/disable fretting detection
    Serial.print(F("  [3/4] Micro-motion detection... "));
    radarSensor.setFrettingDetection(ENABLE_FRETTING);
    Serial.println(ENABLE_FRETTING == eON ? F("ENABLED ✓") : F("DISABLED"));
    delay(500);

    // Step 4: Save configuration and start
    Serial.print(F("  [4/4] Saving configuration... "));
    radarSensor.setSensor(eSaveParams);
    delay(SENSOR_STARTUP_DELAY_MS);
    Serial.println(F("✓"));

    Serial.print(F("  Starting sensor... "));
    radarSensor.setSensor(eStartSen);
    delay(SENSOR_STARTUP_DELAY_MS);
    Serial.println(F("✓"));

    Serial.println();
    Serial.println(F("✓ Configuration complete"));
    Serial.println();
}

void flushUARTBuffer(void) {
    // Clear any garbage/leftover data from sensor startup
    // The sensor often sends binary garbage before settling down
    unsigned long startTime = millis();
    int bytesCleared = 0;

    while (Serial1.available() && (millis() - startTime < 500)) {
        Serial1.read();
        bytesCleared++;
        delay(1);
    }

    if (bytesCleared > 0) {
        Serial.print(F("    (Cleared "));
        Serial.print(bytesCleared);
        Serial.print(F(" bytes of startup garbage)"));
    }
}

void showConfigSummary(void) {
    Serial.println(F("┌─── System Configuration ────────────────┐"));

    // Get sensor status
    sSensorStatus_t status = radarSensor.getStatus();

    Serial.print(F("│ Sensor Status:   "));
    if (status.workStatus && status.initStatus) {
        Serial.println(F("RUNNING ✓             │"));
    } else {
        Serial.println(F("ERROR ✗               │"));
    }

    Serial.print(F("│ Operating Mode:  "));
    Serial.println(F("Speed (Presence-like) │"));

    Serial.print(F("│ Detection Range: "));
    Serial.print(MIN_DETECTION_RANGE_M, 1);
    Serial.print(F(" - "));
    Serial.print(MAX_DETECTION_RANGE_M, 1);
    Serial.println(F(" m          │"));

    Serial.print(F("│ Threshold:       "));
    Serial.print(DETECTION_THRESHOLD);
    Serial.print(F(" ("));
    Serial.print(radarSensor.getThresRange());
    Serial.println(F(" readback)     │"));

    Serial.print(F("│ Fretting:        "));
    Serial.println(ENABLE_FRETTING == eON ? F("Enabled              │") : F("Disabled             │"));

    Serial.print(F("│ Latch Time:      "));
    Serial.print(DETECTION_LATCH_MS / 1000.0, 1);
    Serial.println(F(" sec             │"));

    Serial.print(F("│ Stable Readings: "));
    Serial.print(STABLE_READINGS);
    Serial.println(F("                      │"));

    Serial.print(F("│ UART Verify:     "));
    if (ENABLE_UART_VERIFY) {
        Serial.print(F("Enabled ("));
        Serial.print(UART_VERIFY_SAMPLES);
        Serial.println(F(" samples)   │"));
    } else {
        Serial.println(F("Disabled             │"));
    }

    Serial.print(F("│ Loop Rate:       "));
    Serial.print(1000 / LOOP_DELAY_MS);
    Serial.println(F(" Hz                 │"));

    Serial.print(F("│ Raw UART Debug:  "));
    Serial.println(ENABLE_RAW_UART_DEBUG ? F("Enabled              │") : F("Disabled             │"));

    Serial.print(F("│ Data Quality:    "));
    Serial.println(ENABLE_DATA_QUALITY ? F("Enabled (30s)        │") : F("Disabled             │"));

    Serial.println(F("└──────────────────────────────────────────┘"));

    // Warning if sensor not working correctly
    if (!status.workStatus || !status.initStatus) {
        Serial.println();
        Serial.println(F("⚠ WARNING: Sensor not initialized correctly!"));
        Serial.println(F("   Try: Power cycle the sensor (unplug/replug 5V)"));
        Serial.println();
    }
}

// ============================================================================
// SENSOR READING & DETECTION LOGIC
// ============================================================================

SensorReading readSensor(void) {
    SensorReading reading;

    // Read basic data from sensor
    reading.targetCount = radarSensor.getTargetNumber();
    reading.targetRange = radarSensor.getTargetRange();
    reading.targetSpeed = radarSensor.getTargetSpeed();
    reading.targetEnergy = radarSensor.getTargetEnergy();

    // Track data quality
    dataQuality.totalReadings++;

    // Check for energy corruption
    // Based on real sensor data: normal range is ~1000-50000
    // Values > 10M are likely corrupted
    reading.energyCorrupted = (reading.targetEnergy > MAX_VALID_ENERGY);
    if (reading.energyCorrupted) {
        dataQuality.corruptedEnergy++;
    }

    // Validate range
    reading.valid = (reading.targetRange >= MIN_DETECTION_RANGE_M &&
                     reading.targetRange <= MAX_DETECTION_RANGE_M);

    if (!reading.valid && reading.targetCount > 0) {
        dataQuality.outOfRange++;
    }

    if (reading.valid && reading.targetCount > 0 && !reading.energyCorrupted) {
        dataQuality.validReadings++;
    }

    return reading;
}

bool verifyDetectionUART(void) {
    // Perform additional UART reads to verify detection
    // This reduces false positives from communication glitches

    uint8_t verifiedCount = 0;

    for (uint8_t i = 0; i < UART_VERIFY_SAMPLES; i++) {
        delay(10);  // Small delay between reads

        uint8_t targetCount = radarSensor.getTargetNumber();
        float targetRange = radarSensor.getTargetRange();

        bool validRange = (targetRange >= MIN_DETECTION_RANGE_M &&
                          targetRange <= MAX_DETECTION_RANGE_M);

        if (targetCount > 0 && validRange) {
            verifiedCount++;
        }
    }

    // Require majority of verifications to agree
    return (verifiedCount >= (UART_VERIFY_SAMPLES / 2 + 1));
}

DetectionState evaluateDetection(const SensorReading& reading) {
    // Check if we have a valid detection
    bool hasTarget = (reading.targetCount > 0);

    // Reject readings with corrupted energy values - likely bad sensor data
    bool currentlyDetecting = (hasTarget && reading.valid && !reading.energyCorrupted);

    // Apply consecutive detection filter (reduce false positives)
    if (currentlyDetecting) {
        consecutiveDetections++;
        consecutiveClears = 0;

        // Require multiple consecutive detections
        if (consecutiveDetections >= STABLE_READINGS) {

            // Optional: Additional UART verification
            if (ENABLE_UART_VERIFY) {
                if (verifyDetectionUART()) {
                    lastDetectionTime = millis();
                    dataQuality.detectionCount++;
                    return DetectionState::DETECTED;
                } else {
                    // Failed verification, reset counter
                    consecutiveDetections = 0;
                }
            } else {
                // No verification needed
                lastDetectionTime = millis();
                dataQuality.detectionCount++;
                return DetectionState::DETECTED;
            }
        }
    } else {
        consecutiveClears++;
        consecutiveDetections = 0;
    }

    // Software latch: Stay detected for DETECTION_LATCH_MS after last detection
    // This prevents flickering when person is moving
    unsigned long timeSinceDetection = millis() - lastDetectionTime;
    if (timeSinceDetection < DETECTION_LATCH_MS) {
        return DetectionState::DETECTED;
    }

    return DetectionState::NO_TARGET;
}

// ============================================================================
// OUTPUT CONTROL
// ============================================================================

void updateLED(DetectionState state) {
    bool ledShouldBeOn = (state == DetectionState::DETECTED);

    // Handle inverted logic if configured
    if (INVERT_LED_LOGIC) {
        ledShouldBeOn = !ledShouldBeOn;
    }

    digitalWrite(PIN_STATUS_LED, ledShouldBeOn ? HIGH : LOW);
}

void errorBlinkLED(void) {
    // Blink LED rapidly to indicate error - never returns
    while (true) {
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        delay(ERROR_BLINK_MS);
    }
}

// ============================================================================
// STATUS PRINTING
// ============================================================================

void printBanner(void) {
    Serial.println();
    Serial.println(F("╔════════════════════════════════════════╗"));
    Serial.println(F("║   mmWave Presence Detection System    ║"));
    Serial.println(F("║        Enhanced Edition v2.6          ║"));
    Serial.println(F("║    With UART Debugging & Data QC      ║"));
    Serial.println(F("╚════════════════════════════════════════╝"));
    Serial.println();
}

void printSeparator(void) {
    Serial.println(F("──────────────────────────────────────────"));
}

void printStateChange(DetectionState state) {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════╗"));

    if (state == DetectionState::DETECTED) {
        Serial.println(F("║      🟢 PRESENCE DETECTED             ║"));
        Serial.print(F("║      Latch: "));
        Serial.print(DETECTION_LATCH_MS / 1000.0, 1);
        Serial.println(F(" seconds             ║"));
    } else {
        Serial.println(F("║      ⚫ NO PRESENCE                   ║"));
        Serial.println(F("║      Monitoring...                    ║"));
    }

    Serial.println(F("╚═══════════════════════════════════════╝"));
    Serial.println();
}

void printVerboseStatus(const SensorReading& reading, DetectionState state) {
    // Compact single-line output for continuous monitoring

    // Target info
    Serial.print(F("T:"));
    Serial.print(reading.targetCount);

    // Range
    Serial.print(F(" | R:"));
    Serial.print(reading.targetRange, 2);
    Serial.print(F("m"));

    // Speed
    Serial.print(F(" | S:"));
    if (reading.targetSpeed >= 0) Serial.print(F("+"));
    Serial.print(reading.targetSpeed, 2);
    Serial.print(F("m/s"));

    // Energy (with corruption warning)
    Serial.print(F(" | E:"));
    Serial.print(reading.targetEnergy);
    if (reading.energyCorrupted) {
        Serial.print(F("!"));  // Warning flag for corrupted energy
    }

    // Consecutive count
    Serial.print(F(" | Cons:"));
    Serial.print(consecutiveDetections);

    // Valid flag
    Serial.print(F(" | "));
    Serial.print(reading.valid ? F("✓") : F("✗"));

    // State
    Serial.print(F(" | "));
    if (state == DetectionState::DETECTED) {
        // Show time remaining in latch
        unsigned long timeSinceDetection = millis() - lastDetectionTime;
        unsigned long latchRemaining = 0;
        if (timeSinceDetection < DETECTION_LATCH_MS) {
            latchRemaining = (DETECTION_LATCH_MS - timeSinceDetection) / 1000;
        }

        Serial.print(F("[DETECTED"));
        if (latchRemaining > 0) {
            Serial.print(F(" +"));
            Serial.print(latchRemaining);
            Serial.print(F("s"));
        }
        Serial.println(F("]"));
    } else {
        Serial.println(F("[CLEAR]"));
    }
}

void printRawUART(void) {
    // Read and display raw UART data if available
    // This helps debug what's actually coming from the sensor
    if (Serial1.available()) {
        Serial.print(F("RAW UART: "));

        String message = "";
        unsigned long startTime = millis();

        // Read available data (with timeout)
        while (Serial1.available() && (millis() - startTime < 50)) {
            char c = Serial1.read();

            // Print printable characters, hex for non-printable
            if (isPrintable(c)) {
                Serial.print(c);
                message += c;
            } else {
                Serial.print(F("\\x"));
                if (c < 0x10) Serial.print(F("0"));
                Serial.print(c, HEX);
            }

            // Check if we got a complete message (ends with *)
            if (c == '*' || c == '\n') {
                break;
            }
        }

        Serial.println();

        // If it's a NMEA message, parse and display
        if (message.startsWith("$DFDMD,")) {
            Serial.print(F("  └─> NMEA parsed: "));
            Serial.println(message);
        }
    }
}

void printDataQualityReport(void) {
    if (dataQuality.totalReadings == 0) {
        return;  // No data yet
    }

    Serial.println();
    Serial.println(F("┌─── Data Quality Report (30s) ────────────┐"));

    Serial.print(F("│ Total Readings:     "));
    Serial.print(dataQuality.totalReadings);
    Serial.println(F("                  │"));

    Serial.print(F("│ Valid Detections:   "));
    Serial.print(dataQuality.validReadings);
    Serial.print(F(" ("));
    Serial.print((dataQuality.validReadings * 100) / dataQuality.totalReadings);
    Serial.println(F("%)          │"));

    Serial.print(F("│ Corrupted Energy:   "));
    Serial.print(dataQuality.corruptedEnergy);
    Serial.print(F(" ("));
    Serial.print((dataQuality.corruptedEnergy * 100) / dataQuality.totalReadings);
    Serial.println(F("%)          │"));

    Serial.print(F("│ Out of Range:       "));
    Serial.print(dataQuality.outOfRange);
    Serial.print(F(" ("));
    Serial.print((dataQuality.outOfRange * 100) / dataQuality.totalReadings);
    Serial.println(F("%)          │"));

    Serial.println(F("└──────────────────────────────────────────┘"));
    Serial.println();

    // Reset counters for next period
    dataQuality.totalReadings = 0;
    dataQuality.validReadings = 0;
    dataQuality.corruptedEnergy = 0;
    dataQuality.outOfRange = 0;
}

// ============================================================================
// TROUBLESHOOTING GUIDE
// ============================================================================
/*
 * PROBLEM: Sensor not detecting
 * SOLUTION:
 *   - Reduce DETECTION_THRESHOLD (try 5)
 *   - Reduce STABLE_READINGS to 1
 *   - Increase MAX_DETECTION_RANGE_M
 *   - Ensure ENABLE_FRETTING is eON
 *   - Check sensor is powered (5V recommended, some work on 3.3V)
 *   - Verify sensor is in correct mode (Speed mode = eSpeedMode)
 *
 * PROBLEM: Too many false positives
 * SOLUTION:
 *   - Increase DETECTION_THRESHOLD (try 20)
 *   - Increase STABLE_READINGS to 3-5
 *   - Reduce MAX_DETECTION_RANGE_M
 *   - Increase MIN_DETECTION_RANGE_M to avoid walls/furniture
 *   - Enable ENABLE_UART_VERIFY for extra verification
 *   - Check for sources of interference (fans, heaters, moving curtains)
 *
 * PROBLEM: Flickering LED (rapid on/off)
 * SOLUTION:
 *   - Increase DETECTION_LATCH_MS (try 5000)
 *   - Increase STABLE_READINGS (try 3-5)
 *   - Enable ENABLE_UART_VERIFY
 *   - Increase DETECTION_THRESHOLD
 *   - Check for edge-of-range detections (transition zone effect)
 *
 * PROBLEM: Distance readings seem inaccurate
 * SOLUTION:
 *   - This is NORMAL! Distances are NOT calibrated.
 *   - The C4001 is NOT designed for precise distance measurement.
 *   - Use distances as reference only, not for accurate measurements.
 *   - See Sensor Manual Section 1.1 for details.
 *
 * PROBLEM: Detects beyond MAX_DETECTION_RANGE_M
 * SOLUTION:
 *   - This is NORMAL! 1m transition zone exists beyond MAX range.
 *   - Example: MAX=6m can still detect movement at 6-25m (requires large movement).
 *   - This is sensor hardware behavior, not a bug.
 *   - Reduce MAX_DETECTION_RANGE_M to tighten detection area.
 *   - See Sensor Manual Section 1.1 (transition zone) for details.
 *
 * PROBLEM: Can't communicate with sensor
 * SOLUTION:
 *   - Try BAUD_SENSOR = 115200 instead of 9600
 *   - Check wiring (TX/RX crossover is common issue):
 *     - Pico GP0 (TX) → Sensor RX
 *     - Pico GP1 (RX) → Sensor TX
 *     - GND → GND
 *     - 5V → VCC (or 3.3V depending on sensor variant)
 *   - Verify sensor power supply (5V recommended)
 *   - Check for TX/RX pin swapping
 *   - Try power cycling the sensor
 *   - Verify sensor is C4001 model
 *
 * PROBLEM: Sensor works but performance is poor
 * SOLUTION:
 *   - Try configuring via serial terminal using protocol commands:
 *     - Use setThrFactor to adjust sensitivity (Protocol Section 1.8)
 *     - Use setMicroMotion to enable/disable micro-motion (Protocol Section 1.7)
 *     - Use setRange to adjust detection range (Protocol Section 1.1)
 *   - Consider switching to Presence mode if firmware is fixed:
 *     - Change SENSOR_MODE to 0
 *     - Use setSensitivity command (Protocol Section 1.3)
 *     - Use setLatency for hardware latency (Protocol Section 1.4)
 *     - Use setTrigRange for trigger distance (Protocol Section 1.2)
 *
 * PROBLEM: Want to use Presence mode features (sensitivity, latency)
 * SOLUTION:
 *   - ⚠️ Presence mode firmware currently has issues!
 *   - If you want to try anyway:
 *     - Set SENSOR_MODE = 0
 *     - Change setSensorMode(eSpeedMode) to setSensorMode(ePresenceMode)
 *     - Uncomment sensitivity/latency configuration code
 *     - Test thoroughly - may not work reliably
 *   - Alternative: Stay in Speed mode and tune DETECTION_THRESHOLD
 *
 * PROBLEM: Need to reconfigure sensor via serial commands
 * REFERENCE: Communication Protocol PDF has full command list
 *   - Section 1: Configuration commands (setRange, setSensitivity, etc.)
 *   - Section 2: Control commands (sensorStart, sensorStop, saveConfig, etc.)
 *   - Section 3: Active data reporting ($DFHPD, $DFDMD formats)
 *   - Section 4: Other commands (getHWV, getSWV version info)
 *   - Section 5: Complete configuration examples
 *
 * COMMON SERIAL COMMANDS (connect at 9600 or 115200 baud):
 *   getRange              - Read current detection range
 *   setRange 0.6 10       - Set range 0.6-10m
 *   getThrFactor          - Read threshold factor
 *   setThrFactor 5        - Set threshold factor
 *   getMicroMotion        - Read micro-motion setting
 *   setMicroMotion 1      - Enable micro-motion (0=disable)
 *   sensorStop            - Stop sensor (required before config changes)
 *   saveConfig            - Save settings to flash
 *   sensorStart           - Start sensor with new settings
 *   resetCfg              - Reset to factory defaults
 *   getHWV                - Get hardware version
 *   getSWV                - Get software version
 *
 * For detailed protocol documentation, see Communication Protocol PDF.
 */

// ============================================================================
// END OF CODE
// ============================================================================
