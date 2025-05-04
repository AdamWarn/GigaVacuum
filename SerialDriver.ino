#include <Arduino.h>

// Motor control pins for left and right motors
const int ENA_LEFT = 5;  // Left Motor PWM
const int IN1_LEFT = 6;  // Left Motor Direction
const int IN2_LEFT = 7;

const int ENA_RIGHT = 9;  // Right Motor PWM
const int IN1_RIGHT = 10; // Right Motor Direction
const int IN2_RIGHT = 11;

// Encoder pins for left and right motors
const int ENCODER_LEFT_A = 2;  // Left encoder signal A
const int ENCODER_LEFT_B = 4;  // Left encoder signal B
const int ENCODER_RIGHT_A = 3; // Right encoder signal A
const int ENCODER_RIGHT_B = 8; // Right encoder signal B

// Pulse counts (volatile for interrupt access)
volatile long left_pulse_count = 0;
volatile long right_pulse_count = 0;

void setup() {
    Serial.begin(9600);  // Start serial communication

    // Motor pins setup
    pinMode(ENA_LEFT, OUTPUT);
    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(ENA_RIGHT, OUTPUT);
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);

    // Encoder pins setup
    pinMode(ENCODER_LEFT_A, INPUT);
    pinMode(ENCODER_LEFT_B, INPUT);
    pinMode(ENCODER_RIGHT_A, INPUT);
    pinMode(ENCODER_RIGHT_B, INPUT);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countRightEncoder, CHANGE);
}

void loop() {
    // Motor control: Handle cmd_vel from Raspberry Pi
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        // Parse linear and angular velocity
        float linear_speed = cmd.substring(0, cmd.indexOf(',')).toFloat();
        float angular_speed = cmd.substring(cmd.indexOf(',') + 1).toFloat();

        // Calculate individual motor speeds
        float left_speed = linear_speed - angular_speed;
        float right_speed = linear_speed + angular_speed;

        // Control motors
        controlMotor(ENA_LEFT, IN1_LEFT, IN2_LEFT, left_speed);
        controlMotor(ENA_RIGHT, IN1_RIGHT, IN2_RIGHT, right_speed);
    }

    // Periodically send encoder data back to Raspberry Pi
    static unsigned long last_send_time = 0;
    if (millis() - last_send_time > 100) {  // Send every 100 ms
        String encoder_data = String(left_pulse_count) + "," + String(right_pulse_count) + "\n";
        Serial.print(encoder_data);
        last_send_time = millis();
    }
}

// Motor control function
void controlMotor(int pwm_pin, int dir1_pin, int dir2_pin, float speed) {
    int pwm_value = int(constrain(abs(speed) * 255, 0, 255));

    if (speed > 0) {  // Forward
        analogWrite(pwm_pin, pwm_value);
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
    } else if (speed < 0) {  // Reverse
        analogWrite(pwm_pin, pwm_value);
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
    } else {  // Stop
        analogWrite(pwm_pin, 0);
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, LOW);
    }
}

// Encoder interrupt functions
void countLeftEncoder() {
    if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
        left_pulse_count++;
    } else {
        left_pulse_count--;
    }
}

void countRightEncoder() {
    if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) {
        right_pulse_count++;
    } else {
        right_pulse_count--;
    }
}
