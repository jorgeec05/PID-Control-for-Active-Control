Kalman Filter Variables:

Q_angle: Process noise covariance for the angle (how much uncertainty we expect in the angle estimate).
Q_bias: Process noise covariance for the gyroscope bias (how much uncertainty we expect in the bias estimate).
R_measure: Measurement noise covariance (estimated noise in the accelerometer's pitch reading).
These values are usually determined through experimentation and tuning.

MPU-6050 Setup:

The MPU-6050 is initialized and connected via I2C using the Wire library.
The getMotion6 function from the MPU-6050 library reads accelerometer and gyroscope data.
Pitch Calculation:

calculatePitch computes the pitch angle from the accelerometer data using the arctangent of the Y and Z axis readings.
This gives the raw pitch reading based solely on accelerometer data.
Kalman Filter:

The KalmanFilter function fuses the accelerometer and gyroscope data to give a more accurate estimate of the pitch.
The accelerometer reading is combined with the gyroscope reading over time, reducing the drift commonly seen with gyroscopes.
Delta Time (dt) Calculation:

The Kalman filter needs a reliable estimate of the time between measurements, which is calculated using the millis() function.
The time difference dt is passed to the Kalman filter to update the state.
Usage:
Tuning: The values for Q_angle, Q_bias, and R_measure should be tuned based on your specific setup and desired responsiveness.
Expanding to Roll and Yaw: This implementation focuses on pitch control, but you can expand it to handle roll and yaw by replicating the Kalman filter logic for those axes.

IMU Data Acquisition:

The MPU6050 sensor is used to read the pitch of the rocket. You can adjust this depending on which axis is most relevant to your stabilization (e.g., roll or yaw).
The function readIMUPitch() collects the pitch angle.
PID Control:

A basic PID control loop is implemented in PIDController(). It calculates the error between the desired angle (set to 0 for stability) and the current angle.
The proportional, integral, and derivative terms are computed, and the output is used to adjust the servo motor.
Servo Control:

The output of the PID controller is used to adjust the servo angle.
A simple mapping between the PID output and servo angle is made, where 90° is neutral (no fin deflection), and the range for correction is ±45°.
Loop Timing:

The loop runs every 10 milliseconds (delay(10)), allowing real-time control of the fins.
How to Use:
Install Libraries: You’ll need the MPU6050 and Servo libraries.
Tune the PID Gains: Adjust Kp, Ki, and Kd​ for optimal performance. You can start with small values and adjust based on how the rocket responds in test runs.
Test on Bench: Before flight, test the system on a bench to ensure that the servos are moving as expected and the IMU data is being processed correctly.
Calibration: Make sure the IMU is calibrated properly before using it in the rocket.

Servo Initialization:

The servo is initialized using the Servo library.
The servo is attached to a specific pin (in this case, servoPin is 9), which is a PWM-capable pin on the Teensy 4.0.
Mapping Control Output to Servo Angles:

The output from your PID controller or Kalman filter will be in a range (e.g., -45 to +45 degrees for fin deflection).
The servo expects input in the range of 0 to 180 degrees, so the output is mapped accordingly using the map() function.
For example, a PID output of -45 would correspond to a servo angle of 45 (fully deflected one way), and +45 would correspond to 135 (fully deflected the other way).
Servo Commands:

The servo angle is updated in real-time using the servo.write() function. This sends the new angle to the servo, which adjusts the fins accordingly.
Simulation:

For demonstration purposes, the calculatePIDorKalmanOutput() function generates a simulated output (a sine wave) to move the servo. In your actual implementation, replace this with the output from your PID controller or Kalman filter.
