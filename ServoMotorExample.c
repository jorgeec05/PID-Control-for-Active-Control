#include <Servo.h> // Servo library

// Pin for the servo motor
const int servoPin = 9; // Connect your servo to pin 9 (can be any PWM-capable pin)

// Servo object
Servo finServo;

// Variables for servo control
int servoAngle = 90; // Neutral starting position (90 degrees is the midpoint)

void setup() {
    // Initialize the servo motor
    finServo.attach(servoPin); // Attach the servo to the pin
    
    // Set the servo to the neutral position at the start
    finServo.write(servoAngle); // 90 degrees = neutral (no fin deflection)

    // Initialize serial communication for debugging
    Serial.begin(115200);
}

void loop() {
    // Example: Control the servo based on some sensor input (e.g., PID output)
    // This can be replaced with your PID or Kalman filter output for real-time control

    // Read the PID output (replace this with actual PID/Kalman filter calculation)
    int pidOutput = calculatePIDorKalmanOutput(); // Placeholder function
    
    // Map the PID output to servo angles (assuming output is in the range of -45 to +45 degrees)
    servoAngle = map(pidOutput, -45, 45, 45, 135); // Map -45 degrees to 45 and +45 degrees to 135
    
    // Send the calculated angle to the servo
    finServo.write(servoAngle);
    
    // Debugging: Print the servo angle
    Serial.print("Servo Angle: ");
    Serial.println(servoAngle);

    delay(10); // Control loop delay (adjust based on your system's needs)
}

// Example function to calculate PID or Kalman filter output
int calculatePIDorKalmanOutput() {
    // Placeholder for actual output from your PID/Kalman filter
    // In your case, this would be the output from your control algorithm
    // For now, let's simulate it with a simple sine wave:
    static float t = 0;
    t += 0.01;
    return int(45 * sin(t)); // Simulate output range between -45 and +45
}
