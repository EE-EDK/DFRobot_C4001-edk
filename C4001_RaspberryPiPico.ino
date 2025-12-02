/*!
 * @file C4001_RaspberryPiPico.ino
 * @brief Unified DFRobot C4001 mmWave Radar UART driver for Raspberry Pi Pico
 * @details This implementation includes:
 *          - Proper output trigger debouncing
 *          - Software UART verification
 *          - Comprehensive debug output
 *          - Combined motion detection and ranging
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @version V2.1
 * @date 2025-12-02
 */

#include "src/DFRobot_C4001.h"

// ============================================================================
// CONFIGURATION DEFINES
// ============================================================================

// UART Communication Configuration
#define UART_BAUD_RATE            115200
#define DEBUG_BAUD_RATE           115200

// UART Pin Configuration for Raspberry Pi Pico
#define UART_RX_PIN               5    // GP5 - Connect to sensor TX
#define UART_TX_PIN               4    // GP4 - Connect to sensor RX

// Debug Configuration
#define ENABLE_DEBUG              1    // 1 = Enable debug output, 0 = Disable
#define ENABLE_VERBOSE_DEBUG      1    // 1 = Extra detailed debug, 0 = Basic debug only

// Sensor Operating Mode
#define SENSOR_MODE               eExitMode  // eExitMode or eSpeedMode

// Detection Range Configuration (in cm)
#define DETECTION_MIN_RANGE       30   // Minimum: 30cm (0.3m)
#define DETECTION_MAX_RANGE       1000 // Maximum: 1000cm (10m)
#define DETECTION_TRIG_RANGE      1000 // Trigger distance (should be <= MAX_RANGE)

// Sensitivity Configuration (0-9 scale)
#define TRIG_SENSITIVITY          1    // Trigger sensitivity: 0=lowest, 9=highest
#define KEEP_SENSITIVITY          2    // Keep sensitivity: 0=lowest, 9=highest

// Timing Configuration
#define TRIG_DELAY_UNITS          100  // Trigger delay: 0-200 units (0.01s each) = 1.00s
#define KEEP_TIMEOUT_UNITS        4    // Keep timeout: 4-3000 units (0.5s each) = 2.0s

// PWM Output Configuration
#define PWM_NO_TARGET_DUTY        50   // PWM duty cycle when no target (0-100%)
#define PWM_TARGET_DUTY           0    // PWM duty cycle when target detected (0-100%)
#define PWM_TRANSITION_TIMER      10   // Transition time in units (timer * 64ms)

// IO Polarity
#define IO_POLARITY_ACTIVE_HIGH   1    // 1 = high when target, 0 = low when target

// Debouncing Configuration
#define DEBOUNCE_TIME_MS          500  // Debounce time in milliseconds
#define DEBOUNCE_SAMPLES          5    // Number of consistent samples required
#define UART_VERIFY_SAMPLES       3    // Number of UART verifications needed

// Speed Mode Configuration (only used if SENSOR_MODE = eSpeedMode)
#define SPEED_MIN_RANGE           11   // Minimum range in cm
#define SPEED_MAX_RANGE           1200 // Maximum range in cm
#define SPEED_THRESHOLD           10   // Detection threshold (dimensionless, 0.1 units)
#define FRETTING_DETECTION        eON  // eON or eOFF for micro-motion detection

// Loop timing
#define MAIN_LOOP_DELAY_MS        100  // Main loop delay in milliseconds

// ============================================================================
// DEBUG MACROS
// ============================================================================

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x)          Serial.print(x)
  #define DEBUG_PRINTLN(x)        Serial.println(x)
  #define DEBUG_PRINTF(...)       Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

#if ENABLE_VERBOSE_DEBUG && ENABLE_DEBUG
  #define VERBOSE_PRINT(x)        Serial.print(x)
  #define VERBOSE_PRINTLN(x)      Serial.println(x)
  #define VERBOSE_PRINTF(...)     Serial.printf(__VA_ARGS__)
#else
  #define VERBOSE_PRINT(x)
  #define VERBOSE_PRINTLN(x)
  #define VERBOSE_PRINTF(...)
#endif

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

DFRobot_C4001_UART radar(&Serial1, UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

// ============================================================================
// DEBOUNCING STATE MACHINE
// ============================================================================

typedef enum {
  TARGET_STATE_NONE = 0,
  TARGET_STATE_DETECTED,
  TARGET_STATE_VERIFIED
} eTargetState_t;

typedef struct {
  eTargetState_t currentState;
  eTargetState_t previousState;
  unsigned long stateEntryTime;
  unsigned long lastChangeTime;
  uint8_t consecutiveCount;
  uint8_t uartVerifyCount;
  bool debounced;
} sDebounceState_t;

sDebounceState_t debounceState = {
  .currentState = TARGET_STATE_NONE,
  .previousState = TARGET_STATE_NONE,
  .stateEntryTime = 0,
  .lastChangeTime = 0,
  .consecutiveCount = 0,
  .uartVerifyCount = 0,
  .debounced = false
};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSensor(void);
void configureSensorParameters(void);
void printSensorStatus(void);
void printSensorConfiguration(void);
bool readDebouncedMotion(void);
void updateTargetState(bool rawDetection);
void printTargetData(void);
void processExitMode(void);
void processSpeedMode(void);
bool verifyUARTData(void);

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  // Initialize debug serial port
  Serial.begin(DEBUG_BAUD_RATE);
  while (!Serial) {
    delay(10);
  }

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("========================================"));
  DEBUG_PRINTLN(F("  DFRobot C4001 mmWave Radar - Pico"));
  DEBUG_PRINTLN(F("       UART Interface Mode"));
  DEBUG_PRINTLN(F("========================================"));
  DEBUG_PRINTLN();

  // Print configuration
  DEBUG_PRINT(F("UART Baud: "));
  DEBUG_PRINTLN(UART_BAUD_RATE);
  DEBUG_PRINT(F("UART RX Pin: GP"));
  DEBUG_PRINTLN(UART_RX_PIN);
  DEBUG_PRINT(F("UART TX Pin: GP"));
  DEBUG_PRINTLN(UART_TX_PIN);

  DEBUG_PRINT(F("Sensor Mode: "));
  DEBUG_PRINTLN((SENSOR_MODE == eExitMode) ? F("Exit/Motion Detection") : F("Speed/Ranging"));
  DEBUG_PRINTLN();

  // Initialize sensor
  initializeSensor();

  // Configure sensor parameters
  configureSensorParameters();

  // Print status and configuration
  printSensorStatus();
  printSensorConfiguration();

  DEBUG_PRINTLN(F("========================================"));
  DEBUG_PRINTLN(F("  Initialization Complete"));
  DEBUG_PRINTLN(F("========================================"));
  DEBUG_PRINTLN();

  // Initialize debounce state
  debounceState.stateEntryTime = millis();
  debounceState.lastChangeTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  if (SENSOR_MODE == eExitMode) {
    processExitMode();
  } else {
    processSpeedMode();
  }

  delay(MAIN_LOOP_DELAY_MS);
}

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================

void initializeSensor(void) {
  DEBUG_PRINTLN(F("Initializing UART sensor..."));

  uint8_t attempts = 0;
  const uint8_t maxAttempts = 10;

  while (!radar.begin()) {
    attempts++;
    DEBUG_PRINT(F("Connection attempt "));
    DEBUG_PRINT(attempts);
    DEBUG_PRINT(F("/"));
    DEBUG_PRINTLN(maxAttempts);

    if (attempts >= maxAttempts) {
      DEBUG_PRINTLN(F("ERROR: Failed to connect to sensor!"));
      DEBUG_PRINTLN(F("Check wiring and power supply."));
      DEBUG_PRINTLN(F("Verify UART TX/RX pins are correct."));
      while (1) {
        delay(1000);
      }
    }
    delay(1000);
  }

  DEBUG_PRINTLN(F("Sensor connected successfully!"));
  DEBUG_PRINTLN();
}

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

void configureSensorParameters(void) {
  DEBUG_PRINTLN(F("Configuring sensor parameters..."));

  // Set sensor mode
  DEBUG_PRINT(F("Setting mode to "));
  DEBUG_PRINTLN((SENSOR_MODE == eExitMode) ? F("Exit Mode...") : F("Speed Mode..."));
  if (radar.setSensorMode(SENSOR_MODE)) {
    DEBUG_PRINTLN(F("  ✓ Mode set successfully"));
  } else {
    DEBUG_PRINTLN(F("  ✗ Failed to set mode"));
  }

  if (SENSOR_MODE == eExitMode) {
    // Configure Exit/Motion Detection Mode

    // Set detection range
    DEBUG_PRINTLN(F("Setting detection range..."));
    if (radar.setDetectionRange(DETECTION_MIN_RANGE, DETECTION_MAX_RANGE, DETECTION_TRIG_RANGE)) {
      DEBUG_PRINTLN(F("  ✓ Detection range set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set detection range"));
    }

    // Set trigger sensitivity
    DEBUG_PRINTLN(F("Setting trigger sensitivity..."));
    if (radar.setTrigSensitivity(TRIG_SENSITIVITY)) {
      DEBUG_PRINTLN(F("  ✓ Trigger sensitivity set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set trigger sensitivity"));
    }

    // Set keep sensitivity
    DEBUG_PRINTLN(F("Setting keep sensitivity..."));
    if (radar.setKeepSensitivity(KEEP_SENSITIVITY)) {
      DEBUG_PRINTLN(F("  ✓ Keep sensitivity set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set keep sensitivity"));
    }

    // Set delay parameters
    DEBUG_PRINTLN(F("Setting delay parameters..."));
    if (radar.setDelay(TRIG_DELAY_UNITS, KEEP_TIMEOUT_UNITS)) {
      DEBUG_PRINTLN(F("  ✓ Delay parameters set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set delay parameters"));
    }

    // Set PWM output
    DEBUG_PRINTLN(F("Setting PWM output..."));
    if (radar.setPwm(PWM_NO_TARGET_DUTY, PWM_TARGET_DUTY, PWM_TRANSITION_TIMER)) {
      DEBUG_PRINTLN(F("  ✓ PWM output set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set PWM"));
    }

    // Set IO polarity
    DEBUG_PRINTLN(F("Setting IO polarity..."));
    if (radar.setIoPolaity(IO_POLARITY_ACTIVE_HIGH)) {
      DEBUG_PRINTLN(F("  ✓ IO polarity set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set IO polarity"));
    }

  } else {
    // Configure Speed/Ranging Mode

    // Set detection threshold
    DEBUG_PRINTLN(F("Setting detection threshold..."));
    if (radar.setDetectThres(SPEED_MIN_RANGE, SPEED_MAX_RANGE, SPEED_THRESHOLD)) {
      DEBUG_PRINTLN(F("  ✓ Detection threshold set"));
    } else {
      DEBUG_PRINTLN(F("  ✗ Failed to set detection threshold"));
    }

    // Set fretting detection
    DEBUG_PRINTLN(F("Setting fretting detection..."));
    radar.setFrettingDetection(FRETTING_DETECTION);
    DEBUG_PRINTLN(F("  ✓ Fretting detection set"));
  }

  DEBUG_PRINTLN();
}

// ============================================================================
// STATUS REPORTING
// ============================================================================

void printSensorStatus(void) {
  DEBUG_PRINTLN(F("Sensor Status:"));
  DEBUG_PRINTLN(F("----------------------------------------"));

  sSensorStatus_t status = radar.getStatus();

  DEBUG_PRINT(F("  Work Status: "));
  DEBUG_PRINTLN(status.workStatus ? F("RUNNING") : F("STOPPED"));

  DEBUG_PRINT(F("  Work Mode: "));
  if (status.workMode == eExitMode) {
    DEBUG_PRINTLN(F("EXIT/MOTION DETECTION"));
  } else {
    DEBUG_PRINTLN(F("SPEED/RANGING"));
  }

  DEBUG_PRINT(F("  Init Status: "));
  DEBUG_PRINTLN(status.initStatus ? F("INITIALIZED") : F("NOT INITIALIZED"));

  DEBUG_PRINTLN();
}

void printSensorConfiguration(void) {
  DEBUG_PRINTLN(F("Sensor Configuration:"));
  DEBUG_PRINTLN(F("----------------------------------------"));

  if (SENSOR_MODE == eExitMode) {
    // Exit Mode Configuration

    DEBUG_PRINT(F("  Min Range: "));
    DEBUG_PRINT(radar.getMinRange());
    DEBUG_PRINTLN(F(" cm"));

    DEBUG_PRINT(F("  Max Range: "));
    DEBUG_PRINT(radar.getMaxRange());
    DEBUG_PRINTLN(F(" cm"));

    DEBUG_PRINT(F("  Trig Range: "));
    DEBUG_PRINT(radar.getTrigRange());
    DEBUG_PRINTLN(F(" cm"));

    DEBUG_PRINT(F("  Trig Sensitivity: "));
    DEBUG_PRINTLN(radar.getTrigSensitivity());

    DEBUG_PRINT(F("  Keep Sensitivity: "));
    DEBUG_PRINTLN(radar.getKeepSensitivity());

    DEBUG_PRINT(F("  Trig Delay: "));
    DEBUG_PRINT(radar.getTrigDelay());
    DEBUG_PRINT(F(" units ("));
    DEBUG_PRINT(radar.getTrigDelay() * 0.01);
    DEBUG_PRINTLN(F(" s)"));

    DEBUG_PRINT(F("  Keep Timeout: "));
    DEBUG_PRINT(radar.getKeepTimerout());
    DEBUG_PRINT(F(" units ("));
    DEBUG_PRINT(radar.getKeepTimerout() * 0.5);
    DEBUG_PRINTLN(F(" s)"));

    DEBUG_PRINT(F("  IO Polarity: "));
    DEBUG_PRINTLN(radar.getIoPolaity() ? F("ACTIVE HIGH") : F("ACTIVE LOW"));

    sPwmData_t pwmData = radar.getPwm();
    DEBUG_PRINT(F("  PWM No Target: "));
    DEBUG_PRINT(pwmData.pwm1);
    DEBUG_PRINTLN(F("%"));

    DEBUG_PRINT(F("  PWM Target: "));
    DEBUG_PRINT(pwmData.pwm2);
    DEBUG_PRINTLN(F("%"));

    DEBUG_PRINT(F("  PWM Timer: "));
    DEBUG_PRINT(pwmData.timer);
    DEBUG_PRINT(F(" ("));
    DEBUG_PRINT(pwmData.timer * 64);
    DEBUG_PRINTLN(F(" ms)"));

  } else {
    // Speed Mode Configuration

    DEBUG_PRINT(F("  Min Range: "));
    DEBUG_PRINT(radar.getTMinRange());
    DEBUG_PRINTLN(F(" cm"));

    DEBUG_PRINT(F("  Max Range: "));
    DEBUG_PRINT(radar.getTMaxRange());
    DEBUG_PRINTLN(F(" cm"));

    DEBUG_PRINT(F("  Threshold: "));
    DEBUG_PRINTLN(radar.getThresRange());

    DEBUG_PRINT(F("  Fretting Detection: "));
    DEBUG_PRINTLN(radar.getFrettingDetection() == eON ? F("ON") : F("OFF"));
  }

  DEBUG_PRINTLN();
}

// ============================================================================
// DEBOUNCING LOGIC
// ============================================================================

bool readDebouncedMotion(void) {
  bool rawDetection = radar.motionDetection();
  updateTargetState(rawDetection);
  return debounceState.debounced;
}

void updateTargetState(bool rawDetection) {
  unsigned long currentTime = millis();
  eTargetState_t newState = rawDetection ? TARGET_STATE_DETECTED : TARGET_STATE_NONE;

  // Check if raw detection changed
  if (newState != debounceState.currentState) {
    VERBOSE_PRINT(F("  [DEBOUNCE] Raw state change: "));
    VERBOSE_PRINT(debounceState.currentState);
    VERBOSE_PRINT(F(" -> "));
    VERBOSE_PRINTLN(newState);

    debounceState.currentState = newState;
    debounceState.consecutiveCount = 1;
    debounceState.lastChangeTime = currentTime;
    debounceState.uartVerifyCount = 0;
  } else {
    // State is stable, increment counter
    debounceState.consecutiveCount++;
  }

  // Check if state has been stable for debounce period
  unsigned long timeInState = currentTime - debounceState.lastChangeTime;

  if (timeInState >= DEBOUNCE_TIME_MS &&
      debounceState.consecutiveCount >= DEBOUNCE_SAMPLES) {

    // For UART, verify with additional data reads
    if (debounceState.currentState == TARGET_STATE_DETECTED) {
      if (verifyUARTData()) {
        debounceState.uartVerifyCount++;

        VERBOSE_PRINT(F("  [UART VERIFY] Count: "));
        VERBOSE_PRINT(debounceState.uartVerifyCount);
        VERBOSE_PRINT(F("/"));
        VERBOSE_PRINTLN(UART_VERIFY_SAMPLES);

        if (debounceState.uartVerifyCount >= UART_VERIFY_SAMPLES) {
          // Verified detection
          if (!debounceState.debounced || debounceState.previousState != TARGET_STATE_DETECTED) {
            DEBUG_PRINTLN(F("✓ TARGET VERIFIED (UART + Debounce)"));
            debounceState.debounced = true;
            debounceState.previousState = TARGET_STATE_DETECTED;
          }
        }
      } else {
        // Failed verification, reset
        VERBOSE_PRINTLN(F("  [UART VERIFY] Failed, resetting"));
        debounceState.uartVerifyCount = 0;
        debounceState.consecutiveCount = 0;
      }
    } else {
      // No target state
      if (debounceState.debounced || debounceState.previousState != TARGET_STATE_NONE) {
        DEBUG_PRINTLN(F("✓ NO TARGET (Debounced)"));
        debounceState.debounced = false;
        debounceState.previousState = TARGET_STATE_NONE;
      }
      debounceState.uartVerifyCount = 0;
    }
  }
}

bool verifyUARTData(void) {
  // Read motion detection again to verify
  bool verification1 = radar.motionDetection();
  delay(10);
  bool verification2 = radar.motionDetection();

  VERBOSE_PRINT(F("  [VERIFY] v1="));
  VERBOSE_PRINT(verification1);
  VERBOSE_PRINT(F(", v2="));
  VERBOSE_PRINTLN(verification2);

  return (verification1 && verification2);
}

// ============================================================================
// MODE PROCESSING
// ============================================================================

void processExitMode(void) {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  // Read debounced motion
  bool targetPresent = readDebouncedMotion();

  // Print status periodically or on state change
  static bool lastTargetPresent = false;
  bool stateChanged = (targetPresent != lastTargetPresent);

  if (stateChanged || (currentTime - lastPrintTime >= 5000)) {
    if (targetPresent) {
      DEBUG_PRINTLN(F(">>> MOTION DETECTED <<<"));
    } else {
      VERBOSE_PRINTLN(F("No motion"));
    }
    lastPrintTime = currentTime;
  }

  lastTargetPresent = targetPresent;
}

void processSpeedMode(void) {
  uint8_t targetNumber = radar.getTargetNumber();

  if (targetNumber > 0) {
    DEBUG_PRINTLN(F("========================================"));
    DEBUG_PRINT(F("Target Number: "));
    DEBUG_PRINTLN(targetNumber);

    DEBUG_PRINT(F("Target Speed:  "));
    DEBUG_PRINT(radar.getTargetSpeed());
    DEBUG_PRINTLN(F(" m/s"));

    DEBUG_PRINT(F("Target Range:  "));
    DEBUG_PRINT(radar.getTargetRange());
    DEBUG_PRINTLN(F(" m"));

    DEBUG_PRINT(F("Target Energy: "));
    DEBUG_PRINTLN(radar.getTargetEnergy());

    DEBUG_PRINTLN(F("========================================"));
  } else {
    VERBOSE_PRINTLN(F("No target detected"));
  }
}

// ============================================================================
// END OF FILE
// ============================================================================
