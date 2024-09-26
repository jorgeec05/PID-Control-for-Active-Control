#include <Wire.h>
#include <MPU6050.h> // Library for the MPU-6050

MPU6050 mpu;

// Kalman filter variables
float Q_angle = 0.001;  // Process noise variance for the accelerometer
float Q_bias = 0.003;   // Process noise variance for the gyroscope bias
float R_measure = 0.03; // Measurement noise variance (usually obtained through experimentation)

float angle = 0.0;      // The angle calculated by the Kalman filter (the pitch or roll)
float bias = 0.0;       // The gyroscope bias calculated by the Kalman filter
float rate = 0.0;       // Unbiased rate (gyroscope rate - calculated bias)

float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

unsigned long previousTime;

// Function to initialize the MPU6050
void initializeIMU() {
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }
}

// Function to read raw gyro and accelerometer data
void readIMU(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ) {
    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    
    // Convert the raw values to meaningful data (e.g., Gs for accelerometer, degrees/sec for gyroscope)
    accelX /= 16384.0; // Assuming the full range is +/-2g
    accelY /= 16384.0;
    accelZ /= 16384.0;
    gyroX /= 131.0;    // Assuming the full range is +/-250 degrees/sec
    gyroY /= 131.0;
    gyroZ /= 131.0;
}

// Function to compute the pitch angle from accelerometer data
float calculatePitch(float accelX, float accelY, float accelZ) {
    return atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
}

// Kalman filter implementation
float KalmanFilter(float newAngle, float newRate, float dt) {
    // Predict
    rate = newRate - bias;
    angle += dt * rate;

    // Update error covariance matrix
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Compute the Kalman gain
    float S = P[0][0] + R_measure; // Estimate error
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update estimate with measurement z
    float y = newAngle - angle; // Angle difference
    angle += K[0] * y;
    bias += K[1] * y;

    // Update error covariance matrix
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void setup() {
    Serial.begin(115200);
    
    initializeIMU(); // Initialize MPU-6050
    previousTime = millis(); // Initialize time for Kalman filter
}

void loop() {
    // Read raw IMU data
    float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
    readIMU(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    // Calculate the pitch angle from the accelerometer
    float accelPitch = calculatePitch(accelX, accelY, accelZ);

    // Calculate delta time in seconds
    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Apply Kalman filter
    float kalmanPitch = KalmanFilter(accelPitch, gyroY, dt); // Assuming pitch stabilization

    // Output results for debugging
    Serial.print("Accel Pitch: ");
    Serial.print(accelPitch);
    Serial.print(" | Kalman Pitch: ");
    Serial.println(kalmanPitch);

    delay(10); // Small delay for next loop iteration
}
