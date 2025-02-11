package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class KalmanThatVelo {
    //lower covarience - greater confidence in the source
    public static double Q = 0.01;  // Process noise covariance (adjustable)//lower val means less acurate system on its own, more trust in formula
    public static double R = 0.1;   // Measurement noise covariance (adjustable)//lowering this relys more on the odo
    private double P = 1;     // Estimate covariance
    private double K = 0;     // Kalman gain
    private double X = 0;     // State estimate (velocity)

    // Constructor to initialize Kalman filter for velocity
    public KalmanThatVelo(double initialValue) {
        this.X = initialValue;
    }

    // Update the Kalman filter with a new measurement (measurement is the velocity)
    public double update(double measurement) {
        // Prediction step
        P = P + Q;  // Increase the uncertainty based on process noise

        // Kalman gain
        K = P / (P + R);  // Compute Kalman gain

        // Update estimate
        X = X + K * (measurement - X);  // Adjust the velocity estimate based on measurement

        // Update estimate covariance
        P = (1 - K) * P;

        return X;
    }
}