/*!
 * @file    DFRobot-C4001-24GHz-UART-Enhanced-PiPico.ino
 * @brief   Enhanced mmWave Presence Detection (non-blocking, dual-core, watchdog)
 * @version 3.0.0
 *
 * Architecture:
 *   - Core 0: sensor I/O, detection logic, LED, USB serial diagnostics
 *   - Core 1: LCD rendering (owns Wire/I2C exclusively after init)
 *   - Cores share a DisplaySnapshot guarded by a hardware mutex
 *   - Cooperative scheduler driven by millis() — no blocking delays in loop()
 *   - RP2040 hardware watchdog kicks the device on a stuck loop
 */

#include <DFRobot_C4001.h>
#include <Wire.h>
#include "DFRobot_RGBLCD1602.h"
#include <math.h>
#include "pico/mutex.h"

// ============================================================================
// HARDWARE
// ============================================================================
#define PIN_STATUS_LED          18
#define PIN_UART_RX             1
#define PIN_UART_TX             0
#define PIN_MOTION_INPUT        22

#define INVERT_LED_LOGIC        false

DFRobot_RGBLCD1602 lcd(0x2D, 0x3E);

// ============================================================================
// COMMS
// ============================================================================
#define BAUD_DEBUG              115200
#define BAUD_SENSOR             9600
#define SERIAL_TIMEOUT_MS       3000

// ============================================================================
// DETECTION
// ============================================================================
#define MIN_DETECTION_RANGE_M   2.0f
#define MAX_DETECTION_RANGE_M   10.0f
#define HYSTERESIS_LOWER_M      0.8f
#define HYSTERESIS_UPPER_M      10.2f

#define EMA_ALPHA               0.3f
#define DETECTION_THRESHOLD     9
#define STABLE_READINGS         4
#define ENABLE_UART_VERIFY      true
#define UART_VERIFY_SAMPLES     2
#define UART_VERIFY_GAP_MS      10
#define DETECTION_LATCH_MS      3000UL
#define STARTUP_BUFFER_MS       5000UL
#define ENABLE_FRETTING         eON
#define MIN_VALID_ENERGY        1UL
#define MAX_VALID_ENERGY        100000UL
#define MOTION_HOLDOVER_MS      3000UL

// Adaptive speed limits (range-dependent)
#define SPEED_LIMIT_NEAR_MS     7.0f   // < 3 m  (human sprint)
#define SPEED_LIMIT_MID_MS      10.0f  // 3-6 m  (measurement variance)
#define SPEED_LIMIT_FAR_MS      15.0f  // > 6 m  (greater tolerance)

// ============================================================================
// OUTPUT / DIAGNOSTICS
// ============================================================================
#define ENABLE_VERBOSE_OUTPUT       false
#define ENABLE_STATE_CHANGES        true
#define ENABLE_DETAILED_STARTUP     true
#define ENABLE_RAW_UART_DEBUG       false  // contends with library UART parser
#define ENABLE_DATA_QUALITY         true

#define VERBOSE_PRINT_EVERY_N           10
#define RAW_UART_PRINT_EVERY_N          5
#define LCD_TICK_MS                     250UL
#define DATA_QUALITY_INTERVAL_MS        30000UL
#define METRIC_REPORT_INTERVAL_MS       60000UL
#define LOADING_FLASH_INTERVAL_MS       500UL
#define UART_READ_TIMEOUT_MS            100UL
#define IDENTICAL_READINGS_ALERT        50
#define UART_RAW_BUFFER_SIZE            96

// ============================================================================
// TIMING
// ============================================================================
#define SENSOR_TICK_MS              100UL
#define ERROR_BLINK_MS              200
#define SENSOR_STARTUP_DELAY_MS     2000
#define WATCHDOG_TIMEOUT_MS         8000

#define MIN_DETECTION_RANGE_CM      ((uint16_t)(MIN_DETECTION_RANGE_M * 100))
#define MAX_DETECTION_RANGE_CM      ((uint16_t)(MAX_DETECTION_RANGE_M * 100))

// ============================================================================
// TYPES
// ============================================================================
DFRobot_C4001_UART radarSensor(&Serial1, BAUD_SENSOR, PIN_UART_RX, PIN_UART_TX);

enum class DetectionState : uint8_t { NO_TARGET, DETECTED };

struct SensorReading {
    uint8_t  targetCount;
    float    rawRange;
    float    rawSpeed;
    uint32_t rawEnergy;
    float    smoothedRange;
    float    smoothedSpeed;
    uint32_t smoothedEnergy;
    bool     rangeValid;
    bool     speedValid;
    bool     energyCorrupted;
    bool     valid;
};

struct DataQuality {
    uint32_t totalReadings;
    uint32_t validReadings;
    uint32_t corruptedEnergy;
    uint32_t outOfRange;
    uint32_t highSpeedRejections;
    uint32_t detectionCount;
    unsigned long lastReportTime;
};

struct SensorHealthMonitor {
    uint32_t identicalReadings;
    float    lastRange;
    uint32_t lastEnergy;
    bool     stuck;
};

struct EmaFilter {
    float    range;
    float    speed;
    uint32_t energy;
    bool     initialized;
};

struct SystemMetrics {
    unsigned long totalDetections;
    unsigned long lastReportTime;
};

// Shared between cores; renderer reads under mutex.
struct DisplaySnapshot {
    SensorReading  reading;
    DetectionState state;
    bool           rawMotion;
    bool           sensorStuck;
    unsigned long  lastMotionTime;
    unsigned long  lastDetectionTime;
};

// ============================================================================
// STATE
// ============================================================================
DetectionState currentState  = DetectionState::NO_TARGET;
DetectionState previousState = DetectionState::NO_TARGET;
unsigned long  lastDetectionTime    = 0;
unsigned long  lastSensorTick       = 0;
unsigned long  lastMotionTime       = 0;
uint8_t        consecutiveDetections = 0;
uint32_t       loopCounter           = 0;
uint32_t       rawUartCounter        = 0;

DataQuality          dataQuality   = {};
SensorHealthMonitor  healthMonitor = {};
EmaFilter            ema           = {};
SystemMetrics        metrics       = {};

DisplaySnapshot      _snapshot     = {};
mutex_t              _snapshotMu;
volatile bool        _initComplete = false;

// ============================================================================
// PROTOTYPES
// ============================================================================
void initializeSystem();
void initializeHardware();
void createLCDCustomChars();
void initializeSensor();
void configureSensor();
void flushUARTBuffer();
void performStartupBuffer();
void showConfigSummary();
SensorReading readSensor();
bool verifyDetectionUART();
DetectionState evaluateDetection(const SensorReading& r);
void updateLED(DetectionState s);
void renderLCD(const DisplaySnapshot& snap);
void errorBlinkLED();
void printStateChange(DetectionState s);
void printVerboseStatus(const SensorReading& r, DetectionState s);
void printRawUART();
void printDataQualityReport();
void printBanner();
void printSeparator();
void checkSensorHealth(const SensorReading& r);
float maxSpeedForRange(float range);
void reportSystemMetrics();
int  getFreeRAM();
void publishSnapshot(const SensorReading& r, DetectionState s);

// ============================================================================
// CORE 0 — sensor + logic
// ============================================================================
void setup() {
    mutex_init(&_snapshotMu);
    initializeSystem();
    _initComplete = true;
    rp2040.wdt_begin(WATCHDOG_TIMEOUT_MS);
}

void loop() {
    rp2040.wdt_reset();

    unsigned long now = millis();
    if ((now - lastSensorTick) < SENSOR_TICK_MS) {
        return;  // cooperative yield; no blocking delay
    }
    lastSensorTick = now;

    if (ENABLE_RAW_UART_DEBUG && (rawUartCounter++ % RAW_UART_PRINT_EVERY_N == 0)) {
        printRawUART();
    }

    SensorReading reading = readSensor();
    checkSensorHealth(reading);
    currentState = evaluateDetection(reading);
    updateLED(currentState);
    publishSnapshot(reading, currentState);

    if (ENABLE_VERBOSE_OUTPUT) {
        if (loopCounter % VERBOSE_PRINT_EVERY_N == 0) {
            printVerboseStatus(reading, currentState);
        }
    } else if (ENABLE_STATE_CHANGES && currentState != previousState) {
        printStateChange(currentState);
        previousState = currentState;
    }

    if (ENABLE_DATA_QUALITY &&
        (now - dataQuality.lastReportTime) > DATA_QUALITY_INTERVAL_MS) {
        printDataQualityReport();
        dataQuality.lastReportTime = now;
    }

    if ((now - metrics.lastReportTime) > METRIC_REPORT_INTERVAL_MS) {
        reportSystemMetrics();
        metrics.lastReportTime = now;
    }

    loopCounter++;
}

// ============================================================================
// CORE 1 — LCD renderer (owns Wire/I2C exclusively post-init)
// ============================================================================
void setup1() {
    while (!_initComplete) { delay(10); }
}

void loop1() {
    static unsigned long lastRender = 0;
    unsigned long now = millis();
    if ((now - lastRender) < LCD_TICK_MS) return;
    lastRender = now;

    DisplaySnapshot snap;
    mutex_enter_blocking(&_snapshotMu);
    snap = _snapshot;
    mutex_exit(&_snapshotMu);

    renderLCD(snap);
}

void publishSnapshot(const SensorReading& r, DetectionState s) {
    bool rawMotion = (digitalRead(PIN_MOTION_INPUT) == HIGH);
    if (rawMotion) lastMotionTime = millis();

    mutex_enter_blocking(&_snapshotMu);
    _snapshot.reading           = r;
    _snapshot.state             = s;
    _snapshot.rawMotion         = rawMotion;
    _snapshot.sensorStuck       = healthMonitor.stuck;
    _snapshot.lastMotionTime    = lastMotionTime;
    _snapshot.lastDetectionTime = lastDetectionTime;
    mutex_exit(&_snapshotMu);
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void initializeSystem() {
    Serial.begin(BAUD_DEBUG);
    unsigned long start = millis();
    while (!Serial && (millis() - start < SERIAL_TIMEOUT_MS)) {
        delay(10);
    }

    printBanner();
    initializeHardware();
    initializeSensor();
    configureSensor();

    if (ENABLE_DETAILED_STARTUP) showConfigSummary();
    performStartupBuffer();

    printSeparator();
    Serial.println(F("System ready - monitoring for presence..."));
    printSeparator();
    Serial.println();

    // Ensure first tick reads as "latch expired"
    lastDetectionTime = millis() - DETECTION_LATCH_MS - 1000;
}

void performStartupBuffer() {
    Serial.println();
    Serial.println(F("Startup buffer: stabilizing for 5 seconds..."));

    lcd.setRGB(255, 0, 255);
    lcd.clear();

    unsigned long startBuf = millis();
    unsigned long lastFlash = 0;
    bool flashOn = false;

    while ((millis() - startBuf) < STARTUP_BUFFER_MS) {
        unsigned long elapsed = millis() - startBuf;
        int pct = (elapsed * 100) / STARTUP_BUFFER_MS;

        if ((millis() - lastFlash) > LOADING_FLASH_INTERVAL_MS) {
            flashOn = !flashOn;
            lastFlash = millis();
            lcd.setCursor(3, 0);
            lcd.print(flashOn ? "LOADING..." : "          ");
        }

        int blocks = map(pct, 0, 100, 0, 16);
        lcd.setCursor(0, 1);
        for (int i = 0; i < 16; i++) {
            if (i < blocks) lcd.write(0xFF);
            else            lcd.print(" ");
        }

        // Drain UART chatter so the buffer doesn't overflow
        while (Serial1.available()) Serial1.read();
        delay(20);
    }

    lcd.clear();
    Serial.println(F(" done"));
    flushUARTBuffer();
}

void initializeHardware() {
    Serial.println(F("Initializing hardware..."));

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, INVERT_LED_LOGIC ? HIGH : LOW);
    Serial.println(F("  LED configured"));

    pinMode(PIN_MOTION_INPUT, INPUT);
    Serial.println(F("  Motion input configured"));

    Serial.print(F("  Initializing LCD... "));
    lcd.init();
    lcd.setRGB(0, 0, 255);
    lcd.setCursor(0, 0);
    lcd.print("System Booting..");
    createLCDCustomChars();
    Serial.println(F("ok"));

    Serial1.begin(BAUD_SENSOR);
    delay(200);
    Serial.println(F("  UART initialized"));
    flushUARTBuffer();
    Serial.println(F("  UART buffer flushed"));
    Serial.println();
}

void createLCDCustomChars() {
    byte humanIcon[8] = {
        0b01110, 0b01110, 0b00100, 0b11111,
        0b00100, 0b01010, 0b10001, 0b00000
    };
    lcd.customSymbol(0, humanIcon);

    byte waveIcon[8] = {
        0b00001, 0b00010, 0b00100, 0b01000,
        0b10000, 0b01000, 0b00100, 0b00010
    };
    lcd.customSymbol(1, waveIcon);
}

void initializeSensor() {
    Serial.println(F("Connecting to mmWave sensor..."));
    const uint8_t maxAttempts = 5;
    uint8_t attempts = 0;

    while (!radarSensor.begin()) {
        attempts++;
        Serial.print(F("  attempt "));
        Serial.print(attempts);
        Serial.print(F("/"));
        Serial.println(maxAttempts);

        if (attempts >= maxAttempts) {
            lcd.setRGB(255, 0, 0);
            lcd.setCursor(0, 0);
            lcd.print("SENSOR ERROR!   ");
            Serial.println(F("ERROR: cannot connect to sensor"));
            errorBlinkLED();  // infinite blink; power-cycle required pre-watchdog
        }
        delay(1000);
    }
    Serial.println(F("  sensor connected"));
    Serial.println();
}

void configureSensor() {
    Serial.println(F("Configuring sensor..."));

    if (radarSensor.setSensorMode(eSpeedMode))                   Serial.println(F("  mode set"));
    delay(500);
    if (radarSensor.setDetectThres(MIN_DETECTION_RANGE_CM,
                                   MAX_DETECTION_RANGE_CM,
                                   DETECTION_THRESHOLD))         Serial.println(F("  range/threshold set"));
    delay(500);
    radarSensor.setFrettingDetection(ENABLE_FRETTING);
    Serial.println(F("  fretting configured"));
    delay(500);

    radarSensor.setSensor(eSaveParams);
    delay(SENSOR_STARTUP_DELAY_MS);
    radarSensor.setSensor(eStartSen);
    delay(SENSOR_STARTUP_DELAY_MS);

    Serial.println(F("Configuration complete"));
    Serial.println();
}

void flushUARTBuffer() {
    unsigned long start = millis();
    while (Serial1.available() && (millis() - start < 500)) {
        Serial1.read();
        delay(1);
    }
}

void showConfigSummary() {
    Serial.println(F("Configuration:"));
    Serial.println(F("  Mode:           Pulse (3s one-shot)"));
    Serial.print  (F("  Range:          "));
    Serial.print(MIN_DETECTION_RANGE_M, 1);
    Serial.print(F("-"));
    Serial.print(MAX_DETECTION_RANGE_M, 1);
    Serial.println(F(" m"));
    Serial.print  (F("  Speed limits:   "));
    Serial.print(SPEED_LIMIT_NEAR_MS, 1);  Serial.print(F("/"));
    Serial.print(SPEED_LIMIT_MID_MS,  1);  Serial.print(F("/"));
    Serial.print(SPEED_LIMIT_FAR_MS,  1);  Serial.println(F(" m/s"));
    Serial.print  (F("  Startup buffer: "));
    Serial.print(STARTUP_BUFFER_MS / 1000);
    Serial.println(F(" s"));
    Serial.print  (F("  Watchdog:       "));
    Serial.print(WATCHDOG_TIMEOUT_MS);
    Serial.println(F(" ms"));
}

// ============================================================================
// SENSOR READ + EMA FILTER (raw and smoothed kept separate)
// ============================================================================
float maxSpeedForRange(float range) {
    if (range < 3.0f) return SPEED_LIMIT_NEAR_MS;
    if (range < 6.0f) return SPEED_LIMIT_MID_MS;
    return SPEED_LIMIT_FAR_MS;
}

SensorReading readSensor() {
    SensorReading r = {};
    r.targetCount = radarSensor.getTargetNumber();
    r.rawRange    = radarSensor.getTargetRange();
    r.rawSpeed    = radarSensor.getTargetSpeed();
    r.rawEnergy   = radarSensor.getTargetEnergy();

    // EMA filter — populate smoothed fields without overwriting raw
    if (r.targetCount > 0) {
        if (!ema.initialized) {
            ema.range = r.rawRange;
            ema.speed = r.rawSpeed;
            ema.energy = r.rawEnergy;
            ema.initialized = true;
        } else {
            ema.range  = EMA_ALPHA * r.rawRange  + (1.0f - EMA_ALPHA) * ema.range;
            ema.speed  = EMA_ALPHA * r.rawSpeed  + (1.0f - EMA_ALPHA) * ema.speed;
            ema.energy = (uint32_t)(EMA_ALPHA * r.rawEnergy +
                                    (1.0f - EMA_ALPHA) * ema.energy);
        }
        r.smoothedRange  = ema.range;
        r.smoothedSpeed  = ema.speed;
        r.smoothedEnergy = ema.energy;
    } else {
        ema.initialized = false;
        r.smoothedRange  = r.rawRange;
        r.smoothedSpeed  = r.rawSpeed;
        r.smoothedEnergy = r.rawEnergy;
    }

    dataQuality.totalReadings++;

    r.energyCorrupted = (r.smoothedEnergy < MIN_VALID_ENERGY ||
                         r.smoothedEnergy > MAX_VALID_ENERGY);
    if (r.energyCorrupted) dataQuality.corruptedEnergy++;

    // Hysteresis: looser bounds when already tracking, normal bounds when acquiring.
    static bool wasInRange = false;
    if (wasInRange) {
        r.rangeValid = (r.smoothedRange >= HYSTERESIS_LOWER_M &&
                        r.smoothedRange <= HYSTERESIS_UPPER_M);
    } else {
        r.rangeValid = (r.smoothedRange >= MIN_DETECTION_RANGE_M &&
                        r.smoothedRange <= MAX_DETECTION_RANGE_M);
    }
    wasInRange = r.rangeValid;
    if (!r.rangeValid && r.targetCount > 0) dataQuality.outOfRange++;

    float speedLimit = maxSpeedForRange(r.smoothedRange);
    r.speedValid = (fabsf(r.smoothedSpeed) <= speedLimit);
    if (!r.speedValid && r.targetCount > 0) dataQuality.highSpeedRejections++;

    r.valid = (r.rangeValid && r.speedValid && !r.energyCorrupted);
    if (r.valid && r.targetCount > 0) dataQuality.validReadings++;

    return r;
}

// One-shot synchronous re-read on detection edges.
// Brief blocking is acceptable here: only triggered on transitions, not steady state.
bool verifyDetectionUART() {
    uint8_t verifiedCount = 0;
    for (uint8_t i = 0; i < UART_VERIFY_SAMPLES; i++) {
        delay(UART_VERIFY_GAP_MS);
        rp2040.wdt_reset();

        uint8_t cnt   = radarSensor.getTargetNumber();
        float   range = radarSensor.getTargetRange();
        float   speed = radarSensor.getTargetSpeed();

        bool rOK = (range >= MIN_DETECTION_RANGE_M && range <= MAX_DETECTION_RANGE_M);
        bool sOK = (fabsf(speed) <= maxSpeedForRange(range));

        if (cnt > 0 && rOK && sOK) verifiedCount++;
    }
    return (verifiedCount >= (UART_VERIFY_SAMPLES / 2 + 1));
}

// ============================================================================
// DETECTION STATE MACHINE
// ============================================================================
DetectionState evaluateDetection(const SensorReading& r) {
    // Latch active — unsigned subtraction is rollover-safe; no false ceiling needed.
    if ((millis() - lastDetectionTime) < DETECTION_LATCH_MS) {
        return DetectionState::DETECTED;
    }

    bool hasTarget = (r.targetCount > 0);
    bool validData = (hasTarget && r.valid);

    if (!validData) {
        consecutiveDetections = 0;
        return DetectionState::NO_TARGET;
    }

    consecutiveDetections++;

    if (consecutiveDetections < STABLE_READINGS) {
        return DetectionState::NO_TARGET;
    }

    bool confirmed = ENABLE_UART_VERIFY ? verifyDetectionUART() : true;

    if (confirmed) {
        lastDetectionTime = millis();
        dataQuality.detectionCount++;
        metrics.totalDetections++;
        return DetectionState::DETECTED;
    }

    // Graceful degradation: 2x stability without verification still trips.
    if (consecutiveDetections >= STABLE_READINGS * 2) {
        Serial.println(F("Detection with degraded confidence"));
        lastDetectionTime = millis();
        metrics.totalDetections++;
        return DetectionState::DETECTED;
    }

    return DetectionState::NO_TARGET;
}

// ============================================================================
// LED + LCD
// ============================================================================
void updateLED(DetectionState s) {
    bool on = (s == DetectionState::DETECTED);
    if (INVERT_LED_LOGIC) on = !on;
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

/**
 * @brief LCD renderer (core 1).
 *   Priority: human latch (white) > raw motion (green) > scanning (red).
 *   Sensor-stuck warning overrides with orange.
 */
void renderLCD(const DisplaySnapshot& snap) {
    if (snap.sensorStuck) {
        lcd.setRGB(255, 165, 0);
        lcd.setCursor(0, 0); lcd.print("Sensor Warning! ");
        lcd.setCursor(0, 1); lcd.print("Stuck readings  ");
        return;
    }

    bool human = (snap.state == DetectionState::DETECTED);
    bool motion = ((millis() - snap.lastMotionTime) < MOTION_HOLDOVER_MS);

    if (human) {
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write((byte)0);
        lcd.print(" Human Found!   ");
    } else if (motion) {
        lcd.setRGB(0, 255, 0);
        lcd.setCursor(0, 0);
        lcd.write((byte)1);
        lcd.print(" Motion Detect  ");
    } else {
        lcd.setRGB(255, 0, 0);
        lcd.setCursor(0, 0);
        lcd.print("Scanning...     ");
    }

    lcd.setCursor(0, 1);
    if (!human && !motion) {
        lcd.print("No Targets      ");
        return;
    }
    if (snap.reading.targetCount > 0 && snap.reading.valid) {
        char buf[17];
        snprintf(buf, sizeof(buf), "R:%.2fm E:%lu",
                 snap.reading.smoothedRange,
                 (unsigned long)snap.reading.smoothedEnergy);
        // pad to 16 chars
        size_t len = strlen(buf);
        for (size_t i = len; i < 16; i++) buf[i] = ' ';
        buf[16] = '\0';
        lcd.print(buf);
    } else {
        lcd.print("                ");
    }
}

void errorBlinkLED() {
    while (true) {
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        delay(ERROR_BLINK_MS);
    }
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================
void printBanner() {
    Serial.println();
    Serial.println(F("================================================"));
    Serial.println(F("  mmWave Presence - Pulse Mode v3.0.0"));
    Serial.println(F("  Dual-core, non-blocking, watchdog-protected"));
    Serial.println(F("================================================"));
    Serial.println();
}

void printSeparator() {
    Serial.println(F("------------------------------------------------"));
}

void printStateChange(DetectionState s) {
    Serial.println();
    if (s == DetectionState::DETECTED) {
        Serial.println(F(">>> HUMAN PULSE TRIGGERED (3s)"));
    } else {
        Serial.println(F("<<< RESET: SCANNING"));
    }
    Serial.println();
}

void printVerboseStatus(const SensorReading& r, DetectionState s) {
    Serial.print(F("T:"));   Serial.print(r.targetCount);
    Serial.print(F(" R:"));  Serial.print(r.smoothedRange, 2);
    Serial.print(F("m S:")); Serial.print(r.smoothedSpeed, 2);
    Serial.print(F(" E:"));  Serial.print(r.smoothedEnergy);
    Serial.print(F(" "));    Serial.print(r.valid ? F("ok") : F("rej"));

    if (s == DetectionState::DETECTED) {
        unsigned long elapsed = millis() - lastDetectionTime;
        unsigned long remain  = (elapsed < DETECTION_LATCH_MS)
                                  ? (DETECTION_LATCH_MS - elapsed) / 1000
                                  : 0;
        Serial.print(F(" [LATCH "));
        Serial.print(remain);
        Serial.println(F("s]"));
    } else {
        Serial.println(F(" [SCAN]"));
    }
}

/**
 * @brief Read raw NMEA-style frames from sensor UART.
 * @warning Bytes consumed here are bytes the library cannot parse.
 *          Only enable for diagnostic sessions.
 */
void printRawUART() {
    if (!Serial1.available()) return;

    char buf[UART_RAW_BUFFER_SIZE];
    size_t len = 0;
    unsigned long start = millis();

    while (Serial1.available() &&
           (millis() - start < UART_READ_TIMEOUT_MS) &&
           (len < sizeof(buf) - 1)) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r' || c == '*') {
            if (c == '*') buf[len++] = c;
            break;
        }
        if (isPrintable(c) && len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
    buf[len] = '\0';

    if (len > 0 && strncmp(buf, "$DFDMD,", 7) != 0) {
        Serial.print(F("RAW: "));
        Serial.println(buf);
    }
}

void checkSensorHealth(const SensorReading& r) {
    if (r.targetCount > 0 &&
        r.rawRange  == healthMonitor.lastRange &&
        r.rawEnergy == healthMonitor.lastEnergy) {
        healthMonitor.identicalReadings++;
        if (healthMonitor.identicalReadings > IDENTICAL_READINGS_ALERT) {
            if (!healthMonitor.stuck) {
                Serial.println(F("WARNING: sensor may be stuck"));
            }
            healthMonitor.stuck = true;
        }
    } else {
        healthMonitor.identicalReadings = 0;
        healthMonitor.lastRange  = r.rawRange;
        healthMonitor.lastEnergy = r.rawEnergy;
        healthMonitor.stuck = false;
    }
}

int getFreeRAM() {
    return rp2040.getFreeHeap();
}

void reportSystemMetrics() {
    unsigned long uptime = millis() / 1000;
    Serial.println();
    Serial.print(F("Uptime: "));
    Serial.print(uptime / 3600);
    Serial.print(F("h "));
    Serial.print((uptime % 3600) / 60);
    Serial.print(F("m | Detections: "));
    Serial.print(metrics.totalDetections);
    Serial.print(F(" | Free RAM: "));
    Serial.print(getFreeRAM());
    Serial.println(F(" bytes"));
}

void printDataQualityReport() {
    if (dataQuality.totalReadings == 0) return;

    Serial.println();
    Serial.print(F("Data Quality (30s): Total="));
    Serial.print(dataQuality.totalReadings);
    Serial.print(F(" Valid="));
    Serial.print(dataQuality.validReadings);
    Serial.print(F(" Corrupt="));
    Serial.print(dataQuality.corruptedEnergy);
    Serial.print(F(" OutRange="));
    Serial.print(dataQuality.outOfRange);
    Serial.print(F(" HighSpeed="));
    Serial.println(dataQuality.highSpeedRejections);

    dataQuality.totalReadings        = 0;
    dataQuality.validReadings        = 0;
    dataQuality.corruptedEnergy      = 0;
    dataQuality.outOfRange           = 0;
    dataQuality.highSpeedRejections  = 0;
}
