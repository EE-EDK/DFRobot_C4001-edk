/*!
 * @file    C4001_PresenceDetector_Enhanced.ino
 * @brief   Enhanced mmWave Presence Detection - Best of Both Worlds
 * @details Combines reliable presence detection with robust debouncing
 *          - Uses Speed Mode (Presence mode firmware is broken)
 *          - Software latch for stable detection
 *          - LED indicator with optional inversion
 *          - UART verification for reliability
 *          - Comprehensive configuration via #defines
 * @author  Ethan + Claude Enhanced
 * @date    2025-12-02
 * @version 2.5
 */

#include "DFRobot_C4001.h"

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
#define BAUD_SENSOR             9600    // Sensor UART (9600 is most compatible)
#define SERIAL_TIMEOUT_MS       3000    // Time to wait for USB serial

// ============================================================================
// DETECTION RANGE CONFIGURATION (meters)
// ============================================================================

// Smart range filtering to avoid false triggers
#define MIN_DETECTION_RANGE_M   2.0     // Ignore closer objects (walls, furniture)
#define MAX_DETECTION_RANGE_M   6.0     // Maximum useful detection distance

// Note: Using 2.0m minimum prevents triggering on nearby walls/objects
// Adjust based on your room size and sensor mounting

// ============================================================================
// DETECTION BEHAVIOR CONFIGURATION
// ============================================================================

// --- Sensor Sensitivity ---
#define DETECTION_THRESHOLD     10      // Lower = more sensitive (1-65535)
                                        // Recommended: 10-20 for presence
                                        // Higher values = fewer false positives

// --- Debouncing & Stability ---
#define STABLE_READINGS         3       // Consecutive detections required (2-5 recommended)
#define ENABLE_UART_VERIFY      true    // Extra UART verification (reduces false positives)
#define UART_VERIFY_SAMPLES     3       // Number of verification reads (if enabled)

// --- Software Latch (prevents flickering) ---
#define DETECTION_LATCH_MS      2000    // Stay ON for 3 seconds after last detection
                                        // Prevents rapid ON/OFF cycling
                                        // Adjust: 2000-5000ms typical

// --- Micro-motion Detection ---
#define ENABLE_FRETTING         eON     // eON = detect small movements (recommended)
                                        // eOFF = only detect larger movements

// ============================================================================
// OUTPUT & DISPLAY CONFIGURATION
// ============================================================================

// --- Debug Output Modes ---
#define ENABLE_VERBOSE_OUTPUT   false   // true = show every sensor reading
#define ENABLE_STATE_CHANGES    true    // true = show only when state changes
#define ENABLE_DETAILED_STARTUP true    // true = show full config on startup

// --- Status Update Frequency (when verbose) ---
#define VERBOSE_PRINT_EVERY_N   10      // Print every Nth reading (reduce spam)

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#define LOOP_DELAY_MS           100     // Main loop delay (100ms = 10Hz update rate)
#define ERROR_BLINK_MS          200     // LED blink speed on error
#define SENSOR_STARTUP_DELAY_MS 2000    // Wait time after sensor config changes

// ============================================================================
// ADVANCED OPTIONS (Normally don't need to change)
// ============================================================================

// Convert meters to centimeters for sensor API
#define MIN_DETECTION_RANGE_CM  ((uint16_t)(MIN_DETECTION_RANGE_M * 100))
#define MAX_DETECTION_RANGE_CM  ((uint16_t)(MAX_DETECTION_RANGE_M * 100))

// Validation
#if MIN_DETECTION_RANGE_M < 0.3
  #warning "Minimum range below 0.3m may cause issues"
#endif
#if MAX_DETECTION_RANGE_M > 20.0
  #warning "Maximum range above 20m exceeds sensor capability"
#endif
#if MIN_DETECTION_RANGE_M >= MAX_DETECTION_RANGE_M
  #error "MIN_DETECTION_RANGE must be less than MAX_DETECTION_RANGE"
#endif

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
};

// State tracking variables
DetectionState currentState = DetectionState::NO_TARGET;
DetectionState previousState = DetectionState::NO_TARGET;
unsigned long lastDetectionTime = 0;
uint8_t consecutiveDetections = 0;
uint8_t consecutiveClears = 0;
uint32_t loopCounter = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSystem(void);
void initializeHardware(void);
void initializeSensor(void);
void configureSensor(void);
void showConfigSummary(void);
SensorReading readSensor(void);
bool verifyDetectionUART(void);
DetectionState evaluateDetection(const SensorReading& reading);
void updateLED(DetectionState state);
void errorBlinkLED(void);
void printStateChange(DetectionState state);
void printVerboseStatus(const SensorReading& reading, DetectionState state);
void printBanner(void);
void printSeparator(void);

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

void setup() {
    initializeSystem();
}

void loop() {
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

    // Validate range
    reading.valid = (reading.targetRange >= MIN_DETECTION_RANGE_M &&
                     reading.targetRange <= MAX_DETECTION_RANGE_M);

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
    bool currentlyDetecting = (hasTarget && reading.valid);

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
                    return DetectionState::DETECTED;
                } else {
                    // Failed verification, reset counter
                    consecutiveDetections = 0;
                }
            } else {
                // No verification needed
                lastDetectionTime = millis();
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
    Serial.println(F("║        Enhanced Edition v2.5          ║"));
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

    // Energy
    Serial.print(F(" | E:"));
    Serial.print(reading.targetEnergy);

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

// ============================================================================
// END OF CODE
// ============================================================================
