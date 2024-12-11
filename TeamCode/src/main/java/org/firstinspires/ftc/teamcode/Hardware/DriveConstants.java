package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {


    double robotMass = 5.669905;
    double maxAxialVelocity = 70;
    double maxLateralVelocity = 55;

    public static double mass = 13.6078;
    public static double effectiveWheelRadius = 0.096000062; //meters


}


/*double v_x = 0;  // Desired forward/backward speed (m/s)
double v_y = 0;  // Desired sideways speed (m/s)
double omega = 0; // Desired angular velocity (rad/s)
double r = 2.0;  // Wheel radius (inches or meters)
double L = 7.0;  // Distance from center to wheels (inches or meters)
*/
/*double v1 = (v_x - v_y - L * omega) / r;  // Front-left wheel
double v2 = (v_x + v_y + L * omega) / r;  // Front-right wheel
double v3 = (v_x + v_y - L * omega) / r;  // Rear-left wheel
double v4 = (v_x - v_y + L * omega) / r;  // Rear-right wheel
*///this code finds the desired motor velocity for a target vx and y speed


/*motor1.setPower(v1);  // Set the power for the front-left motor
motor2.setPower(v2);  // Set the power for the front-right motor
motor3.setPower(v3);  // Set the power for the rear-left motor
motor4.setPower(v4);  // Set the power for the rear-right motor
*/


/*double maxSpeed = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));
if (maxSpeed > 1.0) {
    v1 /= maxSpeed;
    v2 /= maxSpeed;
    v3 /= maxSpeed;
    v4 /= maxSpeed;
}*/