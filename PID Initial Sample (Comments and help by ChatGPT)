#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h> // Library for IMU (make sure it's installed)

// Pin assignments
const int servoPin = 9; // Pin to control the servo motor

// PID control variables
double kp = 2.0;  // Proportional gain
double ki = 0.5;  // Integral gain
double kd = 1.0;  // Derivative gain

double setPoint = 0.0;  // Desired angle (target angle for stability, e.g., 0 degrees)
double previousError = 0.0;
double integral = 0.0;
double deltaTime = 0.01; // Loop time in seconds (10 ms)

// MPU6050 object (IMU sensor)
MPU6050 mpu;

// Servo object
Servo finServo;

// Time tracking
unsigned long previousTime = 0;

// Function to read pitch angle from MPU6050
double readIMUPitch() {
    // Placeholder for IMU pitch reading (assuming pitch is the stabilization axis)
    mpu.update(); // Get new sensor readings
    return mpu.getAngleX(); // Assuming getAngleX() returns pitch angle in degrees
}

// PID controller function
double PIDController(double currentAngle) {
    double error = setPoint - currentAngle;
    
    // Proportional term
    double proportional = kp * error;

    // Integral term (accumulated error over time)
    integral += error * deltaTime;

    // Derivative term (rate of change of error)
    double derivative = (error - previousError) / deltaTime;

    // Compute PID output
    double output = proportional + (ki * integral) + (kd * derivative);

    // Save error for the next cycle
    previousError = error;

    // Limit the output (to protect the servo from exceeding its limits)
    if (output > 45) output = 45; // Max angle in degrees
    if (output < -45) output = -45; // Min angle in degrees

    return output;
}

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
    
    // Initialize the servo motor
    finServo.attach(servoPin);

    // Set initial servo position to neutral
    finServo.write(90); // 90 degrees is the neutral position
}

void loop() {
    // Get the current pitch angle from the IMU
    double currentPitch = readIMUPitch();
    
    // Calculate the PID control output
    double pidOutput = PIDController(currentPitch);

    // Convert PID output to servo angle (assuming 90 is neutral)
    double servoAngle = 90 + pidOutput;

    // Command the servo to move to the new angle
    finServo.write(servoAngle);

    // Debugging: Print current angle and PID output
    Serial.print("Current Pitch: ");
    Serial.print(currentPitch);
    Serial.print(" | PID Output: ");
    Serial.println(pidOutput);

    // Delay for next loop cycle (control loop runs every 10 ms)
    delay(10);
}
