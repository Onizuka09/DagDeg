#include <Arduino.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>


// Define pins
const int sensorPins[5] = {34, 35, 32, 33, 25}; 
const int motorLeftPin1 = 26; 
const int motorLeftPin2 = 27; 
const int motorRightPin1 = 14; 
const int motorRightPin2 = 12; 
const int pwmFrequency = 5000;
const int pwmResolution = 8; // 8-bit PWM resolution (0-255)
const int pwmMaxValue = 255;

// PID constants
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
double setPoint = 0;
double input, output;

// PID controller
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// Bluetooth serial communication
BluetoothSerial SerialBT;

enum State {
    TEST_STATE,
    START_POINT,
    CURVES_PATH,    
    SPLIT_PATH,
    DISCONTINUED_LINE_1,
    WAIT_POINT,
    CIRCLE_PATH,
    DISCONTINUED_LINE_2,
    INVERSE_PATH,
    ZIGZAG_PATH,
    END_POINT
};

State currentState = START_POINT;

void handleBluetoothData() {
    if (SerialBT.available()) {
        String data = SerialBT.readStringUntil('\n');
        data.trim(); 

        // Process Bluetooth data to update parameters
        // TODO: handle stop, start, refresh, bias value and inverse value
        if (data.startsWith("Kp=")) {
            Kp = data.substring(3).toFloat();
            pid.SetTunings(Kp, Ki, Kd);
            Serial.println("Kp updated to: " + String(Kp));
        } else if (data.startsWith("Ki=")) {
            Ki = data.substring(3).toFloat();
            pid.SetTunings(Kp, Ki, Kd);
            Serial.println("Ki updated to: " + String(Ki));
        } else if (data.startsWith("Kd=")) {
            Kd = data.substring(3).toFloat();
            pid.SetTunings(Kp, Ki, Kd);
            Serial.println("Kd updated to: " + String(Kd));
        } else if (data.startsWith("state=")) {
            int stateValue = data.substring(6).toInt();
            if (stateValue >= TEST_STATE && stateValue <= END_POINT) {
                currentState = static_cast<State>(stateValue);
                Serial.println("State updated to: " + String(stateValue));
            }
        }
    }
}

void moveForward() {
    ledcWrite(0, pwmMaxValue);
    ledcWrite(1, 0);
    ledcWrite(2, pwmMaxValue);
    ledcWrite(3, 0);
}

void stopMotors() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
}

void followLine(int sensorValues[], bool inverse = false, int bias = 0) {
    // TODO: add function description
    int line = (inverse == true ? HIGH : LOW);
    int error = (sensorValues[0] == line ? -1 + bias : 0) + 
                (sensorValues[1] == line ? -0.5 : 0) + 
                (sensorValues[2] == line ? 0 : 0) + 
                (sensorValues[3] == line ? 0.5 : 0) + 
                (sensorValues[4] == line ? 1 + bias : 0);
    
    input = error;
    pid.Compute();

    int leftMotorSpeed = pwmMaxValue - output;
    int rightMotorSpeed = pwmMaxValue + output;

    ledcWrite(0, constrain(leftMotorSpeed, 0, pwmMaxValue));
    ledcWrite(1, 0);
    ledcWrite(2, constrain(rightMotorSpeed, 0, pwmMaxValue));
    ledcWrite(3, 0);
}


void setup() {
    Serial.begin(115200);
    SerialBT.begin("PID_Robot"); // Bluetooth name

    // Initialize sensor pins
    for (int i = 0; i < 5; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    // Initialize motor pins
    pinMode(motorLeftPin1, OUTPUT);
    pinMode(motorLeftPin2, OUTPUT);
    pinMode(motorRightPin1, OUTPUT);
    pinMode(motorRightPin2, OUTPUT);

    // Initialize PWM channels
    ledcSetup(0, pwmFrequency, pwmResolution);
    ledcAttachPin(motorLeftPin1, 0);
    ledcSetup(1, pwmFrequency, pwmResolution);
    ledcAttachPin(motorLeftPin2, 1);
    ledcSetup(2, pwmFrequency, pwmResolution);
    ledcAttachPin(motorRightPin1, 2);
    ledcSetup(3, pwmFrequency, pwmResolution);
    ledcAttachPin(motorRightPin2, 3);

    // Initialize PID
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-pwmMaxValue, pwmMaxValue);
}

void loop() {
    handleBluetoothData(); // Check and handle Bluetooth commands   
    // Read sensor values
    int sensorValues[5];
    for (int i = 0; i < 5; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
    }

    // State machine
    switch (currentState) {
        case START_POINT:
            if (sensorValues[0] == HIGH && sensorValues[4] == HIGH) {
                currentState = CURVES_PATH;
            } else {
                moveForward();
            }
            break;

        case CURVES_PATH:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = SPLIT_PATH;
            } else {
                followLine(sensorValues);
            }
            break;

        case SPLIT_PATH: // TODO: make sure of the sign of the bias
            if (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && sensorValues[3] == HIGH && sensorValues[4] == HIGH) {
                currentState = DISCONTINUED_LINE_1;
            } else {
                followLine(sensorValues, false, -0.25);
            }
            break;

        case DISCONTINUED_LINE_1:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = WAIT_POINT;
            } else {
                followLine(sensorValues);
            }
            break;

        case WAIT_POINT: // TODO: improve the wait point logic or fine tune the wait value so that the robot stops in the middle
            moveForward();
            delay(500);
            stopMotors();
            delay(5000); // Wait for 5 seconds
            moveForward();
            delay(500);
            currentState = CIRCLE_PATH;
            break;

        case CIRCLE_PATH: // TODO: make sure of the sign of the bias
            if (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH && sensorValues[3] == HIGH && sensorValues[4] == HIGH) {
                currentState = DISCONTINUED_LINE_2;
            } else {
                followLine(sensorValues, false, -0.25);
            }
            break;

        case DISCONTINUED_LINE_2:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = INVERSE_PATH;
            } else {
                followLine(sensorValues);
            }
            break;

        case INVERSE_PATH:
            if (sensorValues[0] == HIGH && sensorValues[4] == HIGH) {
                currentState = ZIGZAG_PATH;
            } else {
                followLine(sensorValues, true);
            }
            break;

        case ZIGZAG_PATH:
            if (sensorValues[0] == LOW && sensorValues[4] == LOW) {
                currentState = END_POINT;
            } else {
                followLine(sensorValues);
            }
            break;

        case END_POINT:
            moveForward();
            delay(500);
            stopMotors();
            break;

        case TEST_STATE:
            followLine(sensorValues);
    }
}