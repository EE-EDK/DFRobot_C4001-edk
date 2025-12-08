/*!
 * @file    C4001_PresenceDetector_LCD_Final.ino
 * @brief   Enhanced mmWave Presence Detection - Pulse Mode + LCD
 * @version 2.9.0 - PROFESSIONAL ENHANCEMENTS
 */

#include <DFRobot_C4001.h>
#include <Wire.h>                // Required for I2C LCD
#include "DFRobot_RGBLCD1602.h"  // DFRobot Gravity LCD Library
#include <math.h>                // [Added] Required for fabs() speed calc

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define PIN_STATUS_LED          18      // LED indicator pin
#define PIN_UART_RX             1       // GP1 - Connect to sensor TX
#define PIN_UART_TX             0       // GP0 - Connect to sensor RX
#define PIN_MOTION_INPUT        22      // GP22 - Connect to sensor "OUT" pin

// --- LCD Configuration ---
// Default I2C address for DFRobot RGB LCD is LCD:0x2D, RGB:0x3E
DFRobot_RGBLCD1602 lcd(0x2D, 0x3E);

// --- LED Behavior ---
#define INVERT_LED_LOGIC        false   
#define LED_BLINK_ON_ERROR      true    

// ============================================================================
// COMMUNICATION CONFIGURATION
// ============================================================================

#define BAUD_DEBUG              115200  
#define BAUD_SENSOR             9600    
#define SERIAL_TIMEOUT_MS       3000    

// ============================================================================
// DETECTION RANGE & SPEED CONFIGURATION
// ============================================================================

#define MIN_DETECTION_RANGE_M   1.0     
#define MAX_DETECTION_RANGE_M   10.0    

// [Added] Speed Filter
#define MAX_HUMAN_SPEED_MS      7.0     // Filter out targets moving faster than 7m/s

// Signal Smoothing (EMA Filter)
#define EMA_ALPHA               0.3     // Smoothing factor (0-1): lower = more smoothing

// ============================================================================
// DETECTION BEHAVIOR CONFIGURATION
// ============================================================================

#define DETECTION_THRESHOLD     20      // Lower = more sensitive
#define STABLE_READINGS         3       
#define ENABLE_UART_VERIFY      true    
#define UART_VERIFY_SAMPLES     3       

// --- Software Latch (One-Shot Pulse) ---
#define DETECTION_LATCH_MS      3000    // Device stays ON for exactly this long
#define STARTUP_BUFFER_MS       5000    // 5 Second buffer at startup

#define ENABLE_FRETTING         eON
#define MIN_VALID_ENERGY        1           // Reject zero or negative energy
#define MAX_VALID_ENERGY        100000      // More realistic threshold 

// ============================================================================
// OUTPUT & DISPLAY CONFIGURATION
// ============================================================================

#define ENABLE_VERBOSE_OUTPUT   false   
#define ENABLE_STATE_CHANGES    true    
#define ENABLE_DETAILED_STARTUP true    
#define ENABLE_RAW_UART_DEBUG   true    
#define ENABLE_DATA_QUALITY     true    

#define VERBOSE_PRINT_EVERY_N           10
#define RAW_UART_PRINT_EVERY_N          5
#define LCD_UPDATE_MS                   250
#define DATA_QUALITY_REPORT_INTERVAL_MS 30000   // 30 seconds
#define LOADING_FLASH_INTERVAL_MS       500     // Loading screen flash rate
#define UART_READ_TIMEOUT_MS            100     // UART read timeout
#define MAX_IDENTICAL_READINGS_ALERT    50      // ~5 seconds at 100ms loop     

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#define LOOP_DELAY_MS           100
#define ERROR_BLINK_MS          200
#define SENSOR_STARTUP_DELAY_MS 2000
#define MILLIS_SANITY_CHECK_MS  4000    // Sanity check for millis() overflow    

// ============================================================================
// ADVANCED OPTIONS
// ============================================================================

#define MIN_DETECTION_RANGE_CM  ((uint16_t)(MIN_DETECTION_RANGE_M * 100))
#define MAX_DETECTION_RANGE_CM  ((uint16_t)(MAX_DETECTION_RANGE_M * 100))

// ============================================================================
// GLOBAL OBJECTS & STATE
// ============================================================================

DFRobot_C4001_UART radarSensor(&Serial1, BAUD_SENSOR, PIN_UART_RX, PIN_UART_TX);

enum class DetectionState : uint8_t {
    NO_TARGET,    
    DETECTED      
};

struct SensorReading {
    uint8_t targetCount;
    float targetRange;
    float targetSpeed;
    uint32_t targetEnergy;
    bool valid;
    bool energyCorrupted;
    bool speedValid; // [Added] New Flag
};

struct DataQuality {
    uint32_t totalReadings;
    uint32_t validReadings;
    uint32_t corruptedEnergy;
    uint32_t outOfRange;
    uint32_t highSpeedRejections; // [Added] New Counter
    uint32_t detectionCount;
    unsigned long lastReportTime;
};

struct SensorHealthMonitor {
    uint32_t identicalReadings;
    float lastRange;
    uint32_t lastEnergy;
    unsigned long lastChangeTime;
};

struct FilteredReading {
    float smoothedRange;
    float smoothedSpeed;
    uint32_t smoothedEnergy;
    bool initialized;
};

struct SystemMetrics {
    unsigned long uptimeSeconds;
    unsigned long totalDetections;
    unsigned long falseRejectionsSpeed;
    unsigned long falseRejectionsRange;
    float averageDetectionRange;
    unsigned long lastMetricUpdate;
};

// State tracking variables
DetectionState currentState = DetectionState::NO_TARGET;
DetectionState previousState = DetectionState::NO_TARGET;
unsigned long lastDetectionTime = 0;
unsigned long lastLcdUpdate = 0;
uint8_t consecutiveDetections = 0;
uint8_t consecutiveClears = 0;
uint32_t loopCounter = 0;
uint32_t rawUartCounter = 0;
unsigned long lastMotionTime = 0;

// Data quality tracking
DataQuality dataQuality = {0, 0, 0, 0, 0, 0, 0};

// Sensor health monitoring
SensorHealthMonitor healthMonitor = {0, 0.0, 0, 0};

// Signal smoothing (EMA filter)
FilteredReading filtered = {0.0, 0.0, 0, false};

// System performance metrics
SystemMetrics metrics = {0, 0, 0, 0, 0.0, 0};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSystem(void);
void initializeHardware(void);
void createLCDCustomChars(void);
void initializeSensor(void);
void configureSensor(void);
void flushUARTBuffer(void);
void performStartupBuffer(void); 
void showConfigSummary(void);
SensorReading readSensor(void);
bool verifyDetectionUART(void);
DetectionState evaluateDetection(const SensorReading& reading);
void updateLED(DetectionState state);
void updateLCD(const SensorReading& reading, DetectionState state);
void errorBlinkLED(void);
void printStateChange(DetectionState state);
void printVerboseStatus(const SensorReading& reading, DetectionState state);
void printRawUART(void);
void printDataQualityReport(void);
void printBanner(void);
void printSeparator(void);
void checkSensorHealth(const SensorReading& reading);
float getMaxSpeedForRange(float range);
void updateSystemMetrics(void);
int getFreeRAM(void);

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

    // Check sensor health
    checkSensorHealth(reading);

    // Evaluate detection with debouncing
    currentState = evaluateDetection(reading);

    // Update LED output
    updateLED(currentState);

    // Update LCD Output
    updateLCD(reading, currentState);

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
        if (now - dataQuality.lastReportTime > DATA_QUALITY_REPORT_INTERVAL_MS) {
            printDataQualityReport();
            dataQuality.lastReportTime = now;
        }
    }

    // Update system performance metrics
    updateSystemMetrics();

    loopCounter++;
    delay(LOOP_DELAY_MS);
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initializeSystem(void) {
    // Start USB serial for debugging
    Serial.begin(BAUD_DEBUG);
    
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < SERIAL_TIMEOUT_MS)) {
        delay(10);
    }

    printBanner();

    // Initialize hardware (LED, UART, LCD)
    initializeHardware();

    // Initialize and configure sensor
    initializeSensor();
    configureSensor();

    if (ENABLE_DETAILED_STARTUP) {
        showConfigSummary();
    }

    // --- Perform 5 Second Startup Buffer ---
    performStartupBuffer();
    // ---------------------------------------

    printSeparator();
    Serial.println(F("✓ System ready - monitoring for presence..."));
    printSeparator();
    Serial.println();

    // Initialize timing to ensure we start CLEARED
    lastDetectionTime = millis() - DETECTION_LATCH_MS - 1000;
}

/**
 * @brief Handles the 5-second delay with Purple/Loading LCD
 */
void performStartupBuffer(void) {
    Serial.println();
    Serial.println(F("⏳ STARTUP BUFFER: Stabilizing for 5 seconds..."));
    
    // Set LCD to Purple (Red + Blue)
    lcd.setRGB(255, 0, 255); 
    lcd.clear();
    
    unsigned long startBuffer = millis();
    bool flashState = false;
    unsigned long lastFlash = 0;

    // Run for 5 seconds
    while (millis() - startBuffer < STARTUP_BUFFER_MS) {
        unsigned long elapsed = millis() - startBuffer;
        int percentage = (elapsed * 100) / STARTUP_BUFFER_MS;

        // Flash "LOADING..." logic
        if (millis() - lastFlash > LOADING_FLASH_INTERVAL_MS) {
            flashState = !flashState;
            lastFlash = millis();
            
            lcd.setCursor(3, 0); // Center text
            if (flashState) {
                lcd.print("LOADING...");
            } else {
                lcd.print("          "); // Clear text
            }
        }

        // Draw Progress Bar on Bottom Row
        int blocks = map(percentage, 0, 100, 0, 16);
        lcd.setCursor(0, 1);
        for (int i = 0; i < 16; i++) {
            if (i < blocks) {
                lcd.write(0xFF); // Solid block character
            } else {
                lcd.print(" ");
            }
        }
        
        // Important: Keep reading/flushing UART so buffer doesn't fill up
        while(Serial1.available()) Serial1.read();
        
        delay(20); 
    }
    
    lcd.clear();
    Serial.println(F(" DONE"));
    
    // Final Flush
    flushUARTBuffer();
}

void initializeHardware(void) {
    Serial.println(F("Initializing hardware..."));

    // Setup LED
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, INVERT_LED_LOGIC ? HIGH : LOW);
    Serial.println(F("  ✓ LED configured"));

    // Setup Motion Input Pin
    pinMode(PIN_MOTION_INPUT, INPUT);
    Serial.println(F("  ✓ Motion Input Pin configured"));

    // Setup LCD
    Serial.print(F("  Initializing LCD... "));
    lcd.init();
    lcd.setRGB(0, 0, 255); // Blue start
    lcd.setCursor(0, 0);
    lcd.print("System Booting..");
    Serial.println(F("✓"));

    // Create custom LCD characters
    createLCDCustomChars();

    // Setup sensor UART
    Serial1.begin(BAUD_SENSOR);
    delay(200);
    Serial.println(F("  ✓ UART initialized"));

    flushUARTBuffer();
    Serial.println(F("  ✓ UART buffer flushed"));

    Serial.println();
}

void createLCDCustomChars(void) {
    // Custom character 0: Human icon
    byte humanIcon[8] = {
        0b01110,
        0b01110,
        0b00100,
        0b11111,
        0b00100,
        0b01010,
        0b10001,
        0b00000
    };
    lcd.createChar(0, humanIcon);

    // Custom character 1: Wave/Motion icon
    byte waveIcon[8] = {
        0b00001,
        0b00010,
        0b00100,
        0b01000,
        0b10000,
        0b01000,
        0b00100,
        0b00010
    };
    lcd.createChar(1, waveIcon);

    Serial.println(F("  ✓ Custom LCD characters created"));
}

void initializeSensor(void) {
    Serial.println(F("Connecting to mmWave sensor..."));
    
    uint8_t attempts = 0;
    const uint8_t maxAttempts = 5;

    while (!radarSensor.begin()) {
        attempts++;
        Serial.print(F("  Connection attempt "));
        Serial.print(attempts);
        Serial.print(F("/"));
        Serial.println(maxAttempts);

        if (attempts >= maxAttempts) {
            lcd.setRGB(255, 0, 0);
            lcd.setCursor(0,0);
            lcd.print("SENSOR ERROR!   ");

            Serial.println(F("✗ ERROR: Cannot connect to sensor!"));
            if (LED_BLINK_ON_ERROR) {
                errorBlinkLED();
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
    Serial.println(F("Configuring sensor parameters..."));

    if (radarSensor.setSensorMode(eSpeedMode)) {
        Serial.println(F("  ✓ Mode Set"));
    }
    delay(500);

    if (radarSensor.setDetectThres(MIN_DETECTION_RANGE_CM, MAX_DETECTION_RANGE_CM, DETECTION_THRESHOLD)) {
        Serial.println(F("  ✓ Range/Threshold Set"));
    }
    delay(500);

    radarSensor.setFrettingDetection(ENABLE_FRETTING);
    Serial.println(F("  ✓ Fretting Configured"));
    delay(500);

    radarSensor.setSensor(eSaveParams);
    delay(SENSOR_STARTUP_DELAY_MS);
    radarSensor.setSensor(eStartSen);
    delay(SENSOR_STARTUP_DELAY_MS);
    
    Serial.println(F("✓ Configuration complete"));
    Serial.println();
}

void flushUARTBuffer(void) {
    unsigned long startTime = millis();
    int bytesCleared = 0;
    while (Serial1.available() && (millis() - startTime < 500)) {
        Serial1.read();
        bytesCleared++;
        delay(1);
    }
}

void showConfigSummary(void) {
    Serial.println(F("┌─── Configuration ───────────────────────┐"));
    Serial.print(F("│ Mode:            "));
    Serial.println(F("Pulse (3s One-Shot)   │")); 
    Serial.print(F("│ Range:           "));
    Serial.print(MIN_DETECTION_RANGE_M, 1);
    Serial.print(F("-"));
    Serial.print(MAX_DETECTION_RANGE_M, 1);
    Serial.println(F("m                │"));
    // [Added] Show Speed Config
    Serial.print(F("│ Max Speed:       "));
    Serial.print(MAX_HUMAN_SPEED_MS, 1);
    Serial.println(F(" m/s               │"));
    Serial.print(F("│ Startup Buffer:  "));
    Serial.print(STARTUP_BUFFER_MS / 1000);
    Serial.println(F(" sec                 │"));
    Serial.println(F("└─────────────────────────────────────────┘"));
}

/**
 * @brief Adaptive speed threshold based on detection range
 * @details Closer targets have stricter limits (human motion)
 *          Farther targets allow more variance (measurement uncertainty)
 */
float getMaxSpeedForRange(float range) {
    if (range < 3.0) return 7.0;        // 0-3m: Human sprint speed
    else if (range < 6.0) return 10.0;  // 3-6m: Allow measurement variance
    else return 15.0;                    // 6-10m: Greater tolerance
}

/**
 * @brief Reads sensor data, applies EMA smoothing, and validates
 */
SensorReading readSensor(void) {
    SensorReading reading;
    reading.targetCount = radarSensor.getTargetNumber();
    reading.targetRange = radarSensor.getTargetRange();
    reading.targetSpeed = radarSensor.getTargetSpeed();
    reading.targetEnergy = radarSensor.getTargetEnergy();

    // Apply EMA filter for signal smoothing
    if (!filtered.initialized && reading.targetCount > 0) {
        // Initialize filter with first reading
        filtered.smoothedRange = reading.targetRange;
        filtered.smoothedSpeed = reading.targetSpeed;
        filtered.smoothedEnergy = reading.targetEnergy;
        filtered.initialized = true;
    } else if (reading.targetCount > 0) {
        // Apply exponential moving average
        filtered.smoothedRange = EMA_ALPHA * reading.targetRange +
                                 (1 - EMA_ALPHA) * filtered.smoothedRange;
        filtered.smoothedSpeed = EMA_ALPHA * reading.targetSpeed +
                                 (1 - EMA_ALPHA) * filtered.smoothedSpeed;
        filtered.smoothedEnergy = EMA_ALPHA * reading.targetEnergy +
                                  (1 - EMA_ALPHA) * filtered.smoothedEnergy;

        // Use filtered values for validation
        reading.targetRange = filtered.smoothedRange;
        reading.targetSpeed = filtered.smoothedSpeed;
        reading.targetEnergy = filtered.smoothedEnergy;
    } else {
        // Reset filter when no target
        filtered.initialized = false;
    }

    dataQuality.totalReadings++;
    reading.energyCorrupted = (reading.targetEnergy > MAX_VALID_ENERGY ||
                              reading.targetEnergy < MIN_VALID_ENERGY);
    if (reading.energyCorrupted) dataQuality.corruptedEnergy++;

    // Range Check with Hysteresis (prevents boundary oscillation)
    static bool wasInRange = false;
    bool rangeValid;

    if (wasInRange) {
        // Use wider bounds when already tracking (hysteresis)
        rangeValid = (reading.targetRange >= 0.8 &&
                     reading.targetRange <= 10.2);
    } else {
        // Use normal bounds when acquiring new target
        rangeValid = (reading.targetRange >= MIN_DETECTION_RANGE_M &&
                     reading.targetRange <= MAX_DETECTION_RANGE_M);
    }
    wasInRange = rangeValid;

    if (!rangeValid && reading.targetCount > 0) dataQuality.outOfRange++;

    // [Added] Adaptive Speed Check (range-dependent threshold)
    float maxAllowedSpeed = getMaxSpeedForRange(reading.targetRange);
    reading.speedValid = (fabs(reading.targetSpeed) <= maxAllowedSpeed);
    if (!reading.speedValid && reading.targetCount > 0) dataQuality.highSpeedRejections++;

    // [Updated] Validity logic - includes range, speed, AND energy validation
    reading.valid = (rangeValid && reading.speedValid && !reading.energyCorrupted);

    if (reading.valid && reading.targetCount > 0) {
        dataQuality.validReadings++;
    }

    return reading;
}

/**
 * @brief Verifies detection with extra UART reads, applying SPEED FILTER
 */
bool verifyDetectionUART(void) {
    uint8_t verifiedCount = 0;
    for (uint8_t i = 0; i < UART_VERIFY_SAMPLES; i++) {
        delay(10);
        uint8_t targetCount = radarSensor.getTargetNumber();
        float targetRange = radarSensor.getTargetRange();
        float targetSpeed = radarSensor.getTargetSpeed(); // [Added] Read speed
        
        bool validRange = (targetRange >= MIN_DETECTION_RANGE_M &&
                          targetRange <= MAX_DETECTION_RANGE_M);

        // [Added] Adaptive Speed Check in Verification
        float maxAllowedSpeed = getMaxSpeedForRange(targetRange);
        bool validSpeed = (fabs(targetSpeed) <= maxAllowedSpeed);

        if (targetCount > 0 && validRange && validSpeed) {
            verifiedCount++;
        }
    }
    return (verifiedCount >= (UART_VERIFY_SAMPLES / 2 + 1));
}

DetectionState evaluateDetection(const SensorReading& reading) {

    unsigned long timeSinceDetection = millis() - lastDetectionTime;

    // --- LATCH ACTIVE PHASE (with millis() overflow protection) ---
    bool latchActive = (timeSinceDetection < DETECTION_LATCH_MS) &&
                       (timeSinceDetection < MILLIS_SANITY_CHECK_MS);
    if (latchActive) {
        return DetectionState::DETECTED;
    }

    // --- SCANNING PHASE ---
    bool hasTarget = (reading.targetCount > 0);
    // Note: reading.valid now includes range, speed, AND energy validation
    bool validData = (hasTarget && reading.valid);

    if (validData) {
        consecutiveDetections++;
        consecutiveClears = 0;

        if (consecutiveDetections >= STABLE_READINGS) {
            bool confirmed = true;
            if (ENABLE_UART_VERIFY) {
                confirmed = verifyDetectionUART();
            }

            if (confirmed) {
                // Full confidence detection
                lastDetectionTime = millis(); // Trigger the 3s Latch
                dataQuality.detectionCount++;
                metrics.totalDetections++;
                return DetectionState::DETECTED;
            } else {
                // Graceful degradation: partial confidence fallback
                if (consecutiveDetections >= STABLE_READINGS * 2) {
                    Serial.println(F("⚠️  Detection with degraded confidence"));
                    lastDetectionTime = millis();
                    metrics.totalDetections++;
                    return DetectionState::DETECTED;
                }
                // Continue accumulating detections for degraded mode
            }
        }
    } else {
        consecutiveClears++;
        consecutiveDetections = 0;
    }

    return DetectionState::NO_TARGET;
}

void updateLED(DetectionState state) {
    bool ledShouldBeOn = (state == DetectionState::DETECTED);
    if (INVERT_LED_LOGIC) ledShouldBeOn = !ledShouldBeOn;
    digitalWrite(PIN_STATUS_LED, ledShouldBeOn ? HIGH : LOW);
}

/**
 * @brief Updated LCD Logic - Data Only (No Countdown)
 * @details 
 * White = Human Latch (Shows Live Data if available)
 * Green = Motion Detected (Shows Live Data)
 * Red   = Scanning
 */
void updateLCD(const SensorReading& reading, DetectionState state) {
    
    bool rawMotion = digitalRead(PIN_MOTION_INPUT);
    if (rawMotion == HIGH) lastMotionTime = millis();

    bool isHumanActive = (state == DetectionState::DETECTED);
    unsigned long timeSinceMotion = millis() - lastMotionTime;
    bool isMotionActive = (timeSinceMotion < 3000) && (timeSinceMotion < MILLIS_SANITY_CHECK_MS); 

    bool timeToUpdateText = (millis() - lastLcdUpdate > LCD_UPDATE_MS);

    // PRIORITY 1: Human Latch Active (White)
    if (isHumanActive) {
        lcd.setRGB(255, 255, 255); // White
        
        if (timeToUpdateText) {
            lcd.setCursor(0, 0);
            lcd.write((byte)0);  // Human icon
            lcd.print(" Human Found!   ");

            lcd.setCursor(0, 1);

            // SHOW DATA (CountDown Removed)
            // Only display if reading is valid (range, speed, and energy are all good)
            if (reading.targetCount > 0 && reading.valid) {
                 lcd.print("R:");
                 lcd.print(reading.targetRange, 2);
                 lcd.print("m E:");
                 lcd.print(reading.targetEnergy);
                 lcd.print("    ");
            } else {
                 lcd.print("                "); // Clear invalid data
            }
            lastLcdUpdate = millis();
        }
    }
    // PRIORITY 2: Raw Motion Only (Green)
    else if (isMotionActive) {
        lcd.setRGB(0, 255, 0); // Green

        if (timeToUpdateText) {
            lcd.setCursor(0, 0);
            lcd.write((byte)1);  // Wave icon
            lcd.print(" Motion Detect ");

            lcd.setCursor(0, 1);
            // Only display if reading is valid (range, speed, and energy are all good)
            if (reading.targetCount > 0 && reading.valid) {
                 lcd.print("R:");
                 lcd.print(reading.targetRange, 2);
                 lcd.print("m E:");
                 lcd.print(reading.targetEnergy);
                 lcd.print("    ");
            } else {
                 lcd.print("                "); // Clear invalid data
            }
            lastLcdUpdate = millis();
        }
    }
    // PRIORITY 3: Clear / Scanning (Red)
    else {
        lcd.setRGB(255, 0, 0); // Red

        if (timeToUpdateText) {
            lcd.setCursor(0, 0);
            lcd.print("Scanning...     ");
            lcd.setCursor(0, 1);
            lcd.print("No Targets      ");
            lastLcdUpdate = millis();
        }
    }
}

void errorBlinkLED(void) {
    while (true) {
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        delay(ERROR_BLINK_MS);
    }
}

void printBanner(void) {
    Serial.println();
    Serial.println(F("╔════════════════════════════════════════╗"));
    Serial.println(F("║   mmWave Presence - Pulse Mode v2.9.0  ║"));
    Serial.println(F("║      with Gravity LCD1602 RGB          ║"));
    Serial.println(F("╚════════════════════════════════════════╝"));
    Serial.println();
}

void printSeparator(void) {
    Serial.println(F("──────────────────────────────────────────"));
}

void printStateChange(DetectionState state) {
    Serial.println();
    Serial.println(F("┌───────────────────────────────────────┐"));
    if (state == DetectionState::DETECTED) {
        Serial.println(F("│  🟢 HUMAN PULSE TRIGGERED (3s)        │"));
    } else {
        Serial.println(F("│  ⚫ RESET: SCANNNING                  │"));
    }
    Serial.println(F("└───────────────────────────────────────┘"));
    Serial.println();
}

void printVerboseStatus(const SensorReading& reading, DetectionState state) {
    Serial.print(F("T:"));
    Serial.print(reading.targetCount);
    Serial.print(F(" | R:"));
    Serial.print(reading.targetRange, 2);
    Serial.print(F("m | "));
    Serial.print(reading.valid ? F("✓") : F("✗"));
    Serial.print(F(" | "));
    if (state == DetectionState::DETECTED) {
        unsigned long timeSinceDetection = millis() - lastDetectionTime;
        unsigned long latchRemaining = 0;
        if (timeSinceDetection < DETECTION_LATCH_MS &&
            timeSinceDetection < MILLIS_SANITY_CHECK_MS) {
            latchRemaining = (DETECTION_LATCH_MS - timeSinceDetection) / 1000;
        }
        Serial.print(F("[LATCHED: "));
        Serial.print(latchRemaining);
        Serial.println(F("s]"));
    } else {
        Serial.println(F("[SCANNING]"));
    }
}

void printRawUART(void) {
    if (Serial1.available()) {
        String message = "";
        unsigned long startTime = millis();
        while (Serial1.available() && (millis() - startTime < UART_READ_TIMEOUT_MS)) {
            char c = Serial1.read();
            if (isPrintable(c) && c != '\r' && c != '\n') message += c;
            if (c == '*' || c == '\n' || c == '\r') {
                delay(2);
                while (Serial1.available()) {
                    char trailing = Serial1.read();
                    if (isPrintable(trailing) && trailing != '\r' && trailing != '\n') message += trailing;
                    if (trailing == '\n') break;
                }
                break;
            }
        }
        if (message.startsWith("$DFDMD,")) {
             // Optional: Add full parsing if needed, mostly noise for now
        } else if (message.length() > 0) {
            Serial.print(F("RAW: "));
            Serial.println(message);
        }
    }
}

void checkSensorHealth(const SensorReading& reading) {
    // Detect stuck readings
    if (reading.targetRange == healthMonitor.lastRange &&
        reading.targetEnergy == healthMonitor.lastEnergy &&
        reading.targetCount > 0) {
        healthMonitor.identicalReadings++;

        if (healthMonitor.identicalReadings > MAX_IDENTICAL_READINGS_ALERT) {
            Serial.println(F("⚠️  WARNING: Sensor may be stuck!"));
            lcd.setRGB(255, 165, 0);  // Orange warning
            // Optional: Attempt sensor reset
            // radarSensor.setSensor(eStartSen);
        }
    } else {
        healthMonitor.identicalReadings = 0;
        healthMonitor.lastRange = reading.targetRange;
        healthMonitor.lastEnergy = reading.targetEnergy;
    }
}

int getFreeRAM() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void updateSystemMetrics(void) {
    unsigned long now = millis();
    if (now - metrics.lastMetricUpdate > 60000) {  // Every 60 seconds
        metrics.uptimeSeconds += 60;
        metrics.lastMetricUpdate = now;

        Serial.println();
        Serial.print(F("📊 Uptime: "));
        Serial.print(metrics.uptimeSeconds / 3600);
        Serial.print(F("h "));
        Serial.print((metrics.uptimeSeconds % 3600) / 60);
        Serial.print(F("m | Detections: "));
        Serial.print(metrics.totalDetections);
        Serial.print(F(" | Free RAM: "));
        Serial.print(getFreeRAM());
        Serial.println(F(" bytes"));
    }
}

void printDataQualityReport(void) {
    if (dataQuality.totalReadings == 0) return;
    Serial.println();
    Serial.print(F("Data Quality (30s): Valid="));
    Serial.print(dataQuality.validReadings);
    Serial.print(F(" Corrupt="));
    Serial.print(dataQuality.corruptedEnergy);
    Serial.print(F(" OutRange="));
    Serial.print(dataQuality.outOfRange);
    // [Added] Print Speed Rejections
    Serial.print(F(" HighSpeedRejections="));
    Serial.println(dataQuality.highSpeedRejections);
    
    dataQuality.totalReadings = 0;
    dataQuality.validReadings = 0;
    dataQuality.corruptedEnergy = 0;
    dataQuality.outOfRange = 0;
    dataQuality.highSpeedRejections = 0;
}