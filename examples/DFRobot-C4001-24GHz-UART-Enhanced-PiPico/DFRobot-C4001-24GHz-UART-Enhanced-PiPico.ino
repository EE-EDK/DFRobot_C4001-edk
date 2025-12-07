/*!
 * @file    C4001_PresenceDetector_LCD_Final.ino
 * @brief   Enhanced mmWave Presence Detection - Pulse Mode + LCD
 * @version 2.7.4 - NO COUNTDOWN, DATA ONLY
 */

#include <DFRobot_C4001.h>
#include <Wire.h>                // Required for I2C LCD
#include "DFRobot_RGBLCD1602.h"  // DFRobot Gravity LCD Library

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
// DETECTION RANGE CONFIGURATION (meters)
// ============================================================================

#define MIN_DETECTION_RANGE_M   1.0     
#define MAX_DETECTION_RANGE_M   10.0     

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
#define MAX_VALID_ENERGY        10000000 

// ============================================================================
// OUTPUT & DISPLAY CONFIGURATION
// ============================================================================

#define ENABLE_VERBOSE_OUTPUT   false   
#define ENABLE_STATE_CHANGES    true    
#define ENABLE_DETAILED_STARTUP true    
#define ENABLE_RAW_UART_DEBUG   true    
#define ENABLE_DATA_QUALITY     true    

#define VERBOSE_PRINT_EVERY_N   10      
#define RAW_UART_PRINT_EVERY_N  5       
#define LCD_UPDATE_MS           250     

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

#define LOOP_DELAY_MS           100     
#define ERROR_BLINK_MS          200     
#define SENSOR_STARTUP_DELAY_MS 2000    

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
};

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
unsigned long lastLcdUpdate = 0;
uint8_t consecutiveDetections = 0;
uint8_t consecutiveClears = 0;
uint32_t loopCounter = 0;
uint32_t rawUartCounter = 0;
unsigned long lastMotionTime = 0;

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
        if (now - dataQuality.lastReportTime > 30000) { 
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

        // Flash "LOADING..." logic (every 500ms)
        if (millis() - lastFlash > 500) {
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

    // Setup sensor UART
    Serial1.begin(BAUD_SENSOR);
    delay(200);
    Serial.println(F("  ✓ UART initialized"));

    flushUARTBuffer();
    Serial.println(F("  ✓ UART buffer flushed"));

    Serial.println();
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
    Serial.print(F("│ Startup Buffer:  "));
    Serial.print(STARTUP_BUFFER_MS / 1000);
    Serial.println(F(" sec                 │"));
    Serial.println(F("└─────────────────────────────────────────┘"));
}

SensorReading readSensor(void) {
    SensorReading reading;
    reading.targetCount = radarSensor.getTargetNumber();
    reading.targetRange = radarSensor.getTargetRange();
    reading.targetSpeed = radarSensor.getTargetSpeed();
    reading.targetEnergy = radarSensor.getTargetEnergy();

    dataQuality.totalReadings++;
    reading.energyCorrupted = (reading.targetEnergy > MAX_VALID_ENERGY);
    if (reading.energyCorrupted) dataQuality.corruptedEnergy++;

    reading.valid = (reading.targetRange >= MIN_DETECTION_RANGE_M &&
                     reading.targetRange <= MAX_DETECTION_RANGE_M);
    
    if (!reading.valid && reading.targetCount > 0) dataQuality.outOfRange++;
    if (reading.valid && reading.targetCount > 0 && !reading.energyCorrupted) dataQuality.validReadings++;

    return reading;
}

bool verifyDetectionUART(void) {
    uint8_t verifiedCount = 0;
    for (uint8_t i = 0; i < UART_VERIFY_SAMPLES; i++) {
        delay(10);
        uint8_t targetCount = radarSensor.getTargetNumber();
        float targetRange = radarSensor.getTargetRange();
        bool validRange = (targetRange >= MIN_DETECTION_RANGE_M &&
                          targetRange <= MAX_DETECTION_RANGE_M);
        if (targetCount > 0 && validRange) {
            verifiedCount++;
        }
    }
    return (verifiedCount >= (UART_VERIFY_SAMPLES / 2 + 1));
}

DetectionState evaluateDetection(const SensorReading& reading) {
    
    unsigned long timeSinceDetection = millis() - lastDetectionTime;
    
    // --- LATCH ACTIVE PHASE ---
    if (timeSinceDetection < DETECTION_LATCH_MS) {
        return DetectionState::DETECTED;
    }

    // --- SCANNING PHASE ---
    bool hasTarget = (reading.targetCount > 0);
    bool validData = (hasTarget && reading.valid && !reading.energyCorrupted);

    if (validData) {
        consecutiveDetections++;
        consecutiveClears = 0;

        if (consecutiveDetections >= STABLE_READINGS) {
            bool confirmed = true;
            if (ENABLE_UART_VERIFY) {
                confirmed = verifyDetectionUART();
            }

            if (confirmed) {
                lastDetectionTime = millis(); // Trigger the 3s Latch
                dataQuality.detectionCount++;
                return DetectionState::DETECTED;
            } else {
                consecutiveDetections = 0;
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
    bool isMotionActive = (millis() - lastMotionTime < 3000); 

    bool timeToUpdateText = (millis() - lastLcdUpdate > LCD_UPDATE_MS);

    // PRIORITY 1: Human Latch Active (White)
    if (isHumanActive) {
        lcd.setRGB(255, 255, 255); // White
        
        if (timeToUpdateText) {
            lcd.setCursor(0, 0);
            lcd.print("Human Found!    ");
            
            lcd.setCursor(0, 1);
            
            // SHOW DATA (CountDown Removed)
            if (reading.targetCount > 0) {
                 lcd.print("R:");
                 lcd.print(reading.targetRange, 2);
                 lcd.print("m E:"); 
                 lcd.print(reading.targetEnergy);
                 lcd.print("    "); 
            }        
            lastLcdUpdate = millis();
        }
    }
    // PRIORITY 2: Raw Motion Only (Green)
    else if (isMotionActive) {
        lcd.setRGB(0, 255, 0); // Green

        if (timeToUpdateText) {
            lcd.setCursor(0, 0);
            lcd.print("Motion Detected ");
            
            lcd.setCursor(0, 1);
            if (reading.targetCount > 0) {
                 lcd.print("R:");
                 lcd.print(reading.targetRange, 2);
                 lcd.print("m E:"); 
                 lcd.print(reading.targetEnergy);
                 lcd.print("    "); 
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
    Serial.println(F("║   mmWave Presence - Pulse Mode v2.7.4  ║"));
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
        if (timeSinceDetection < DETECTION_LATCH_MS) {
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
        while (Serial1.available() && (millis() - startTime < 100)) {
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

void printDataQualityReport(void) {
    if (dataQuality.totalReadings == 0) return;
    Serial.println();
    Serial.print(F("Data Quality (30s): Valid="));
    Serial.print(dataQuality.validReadings);
    Serial.print(F(" Corrupt="));
    Serial.print(dataQuality.corruptedEnergy);
    Serial.print(F(" OutRange="));
    Serial.println(dataQuality.outOfRange);
    
    dataQuality.totalReadings = 0;
    dataQuality.validReadings = 0;
    dataQuality.corruptedEnergy = 0;
    dataQuality.outOfRange = 0;
}