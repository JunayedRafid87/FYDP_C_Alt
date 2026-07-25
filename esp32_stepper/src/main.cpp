#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================
#define I2C_SDA     8   // ESP32-S3 SDA
#define I2C_SCL     9   // ESP32-S3 SCL
#define STEP_PIN    4   // TMC2209 STEP
#define DIR_PIN     5   // TMC2209 DIR
#define EN_PIN      6   // TMC2209 EN (LOW = enabled)

// ============================================================
// MOTOR CONFIGURATION
// ============================================================
#define STEPS_PER_REV       1600
#define DEGREES_PER_STEP    (360.0 / STEPS_PER_REV)
#define TILT_MIN_DEG       -45.0
#define TILT_MAX_DEG        45.0
#define TILT_MIN_STEPS     ((long)(TILT_MIN_DEG / DEGREES_PER_STEP))
#define TILT_MAX_STEPS     ((long)(TILT_MAX_DEG / DEGREES_PER_STEP))

// Slower, smoother scanning speed (adjusted slightly faster)
#define MAX_SPEED           800     
#define ACCELERATION        400     
#define REPORT_INTERVAL_MS  20

// ============================================================
// STATE MACHINE DEFINITIONS
// ============================================================
enum ScanState {
    STATE_MOVING,           // Rover is moving, stepper locked at 0
    STATE_STABILIZING,      // Motion stopped, waiting to settle
    STATE_SCANNING,         // Stable, actively sweeping for 30s
    STATE_SCAN_COMPLETE     // 30s sweep complete, stepper sleeping at 0
};

// ============================================================
// GLOBALS
// ============================================================
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool bnoFound = false;

// State tracking
ScanState currentState = STATE_MOVING;
unsigned long stateTimerStart = 0;
unsigned long lastMotionTime = 0;
bool sweepForward = true;
unsigned long lastAngleReport = 0;

// Print helper functions to write to both Native USB (Serial) and UART (Serial0)
template<typename T>
void printBoth(T val) {
    Serial.print(val);
    Serial0.print(val);
}

template<typename T>
void printlnBoth(T val) {
    Serial.println(val);
    Serial0.println(val);
}

void setup() {
    Serial.begin(115200);
    Serial0.begin(115200);
    delay(2000);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (bno.begin(OPERATION_MODE_IMUPLUS)) {
        bnoFound = true;
        bno.setExtCrystalUse(true);
        printlnBoth("# BNO055 Initialized in IMUPLUS mode.");
    } else {
        printlnBoth("# WARNING: BNO055 not found! Falling back to stepper calculations.");
    }

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable driver

    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);
    stepper.moveTo(0);

    lastMotionTime = millis();
}

void loop() {
    unsigned long now = millis();
    bool motionDetected = false;
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

    if (bnoFound) {
        // Read IMU Orientation
        imu::Quaternion quat = bno.getQuat();
        qw = quat.w();
        qx = quat.x();
        qy = quat.y();
        qz = quat.z();

        // Read sensor vectors to check for physical base movement
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> linear_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

        double gyro_mag = sqrt(gyro.x()*gyro.x() + gyro.y()*gyro.y() + gyro.z()*gyro.z());
        double acc_mag = sqrt(linear_acc.x()*linear_acc.x() + linear_acc.y()*linear_acc.y() + linear_acc.z()*linear_acc.z());

        // Robust thresholds to avoid false motion reset from baseline sensor noise:
        // Gyro > 8.0 deg/s OR linear accel > 1.2 m/s²
        motionDetected = (gyro_mag > 8.0) || (acc_mag > 1.2);
    }

    if (motionDetected) {
        lastMotionTime = now;
    }

    // ============================================================
    // STATE MACHINE TRANSITIONS & EXECUTION
    // ============================================================
    switch (currentState) {
        case STATE_MOVING:
            // If movement stopped, begin stabilizing phase
            if (now - lastMotionTime >= 1000) {
                currentState = STATE_STABILIZING;
                stateTimerStart = now;
            }
            
            // Return stepper to horizontal and disable driver holding current
            digitalWrite(EN_PIN, LOW);
            stepper.moveTo(0);
            stepper.run();
            if (stepper.distanceToGo() == 0) {
                digitalWrite(EN_PIN, HIGH); // Disable holding current to prevent heat
            }
            break;

        case STATE_STABILIZING:
            // If new motion detected, reset to moving state
            if (now - lastMotionTime < 1000) {
                currentState = STATE_MOVING;
                break;
            }
            
            // Wait 7.0 seconds to allow 2D SLAM position tracking to settle
            if (now - stateTimerStart >= 7000) {
                currentState = STATE_SCANNING;
                stateTimerStart = now;
                sweepForward = true;
                digitalWrite(EN_PIN, LOW);
                stepper.moveTo(TILT_MAX_STEPS);
            }
            
            // Keep stepper flat during stabilizing
            digitalWrite(EN_PIN, LOW);
            stepper.moveTo(0);
            stepper.run();
            break;

        case STATE_SCANNING:
            // If new motion detected, immediately abort scan
            if (now - lastMotionTime < 1000) {
                currentState = STATE_MOVING;
                stepper.moveTo(0);
                break;
            }

            // Active scanning for exactly 30 seconds
            if (now - stateTimerStart >= 30000) {
                currentState = STATE_SCAN_COMPLETE;
                stepper.moveTo(0);
                break;
            }

            // Sweep stepper motor back and forth
            digitalWrite(EN_PIN, LOW);
            if (stepper.distanceToGo() == 0) {
                sweepForward = !sweepForward;
                stepper.moveTo(sweepForward ? TILT_MAX_STEPS : TILT_MIN_STEPS);
            }
            stepper.run();
            break;

        case STATE_SCAN_COMPLETE:
            // If new motion detected, reset state machine
            if (now - lastMotionTime < 1000) {
                currentState = STATE_MOVING;
                break;
            }

            // Keep stepper locked at 0 and disable driver to sleep
            digitalWrite(EN_PIN, LOW);
            stepper.moveTo(0);
            stepper.run();
            if (stepper.distanceToGo() == 0) {
                digitalWrite(EN_PIN, HIGH); // Sleep driver until next physical movement
            }
            break;
    }

    // ============================================================
    // TELEMETRY REPORTING (50 Hz)
    // ============================================================
    if (now - lastAngleReport >= REPORT_INTERVAL_MS) {
        lastAngleReport = now;
        float stepperAngle = stepper.currentPosition() * DEGREES_PER_STEP;
        bool isMovingReport = (currentState == STATE_MOVING || currentState == STATE_STABILIZING);

        printBoth("IMU:");
        printBoth(qw); printBoth(",");
        printBoth(qx); printBoth(",");
        printBoth(qy); printBoth(",");
        printlnBoth(qz);

        printBoth("STEP:");
        printlnBoth(stepperAngle);

        printBoth("MOVING:");
        printlnBoth(isMovingReport ? 1 : 0);

        printBoth("STATE:");
        switch (currentState) {
            case STATE_MOVING:        printlnBoth("MOVING"); break;
            case STATE_STABILIZING:   printlnBoth("STABILIZING"); break;
            case STATE_SCANNING:      printlnBoth("SCANNING"); break;
            case STATE_SCAN_COMPLETE: printlnBoth("SCAN_COMPLETE"); break;
        }
    }
}
