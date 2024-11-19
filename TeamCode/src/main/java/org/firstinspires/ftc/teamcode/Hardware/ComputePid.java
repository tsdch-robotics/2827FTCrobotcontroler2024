package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ComputePid {

    public static double yawKp = 0.7, yawKi = 0.01, yawKd = 0.001;  // PID gains
    public static double vxKp = 0.1, vxKi = 0, vxKd = 0;
    public static double vyKp = 0.1, vyKi = 0, vyKd = 0;

    public double yawPreviousError = 0;
    public double yawIntegralSum = 0;
    public double yawPreviousTime = 0; // Time from previous iteration

    public double vxPreviousError = 0;
    public double vxIntegralSum = 0;
    public double vxPreviousTime = 0; // Time from previous iteration


    public double vyPreviousError = 0;
    public double vyIntegralSum = 0;
    public double vyPreviousTime = 0; // Time from previous iteration


    public static double maxPower = 1;


    //takes postion, heading, wheel data

    //calculates wheel power using a pid equation, returns the holonomic drive values



    public double vxPID(double currentX, double currentTime,double target){

        // Calculate the error
        double error = target - currentX;

        // Calculate the time difference
        double deltaTime = currentTime - vxPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term
        double proportionalTerm = vxKp * error;

        // Integral term (sum of previous errors)
        vxIntegralSum += error * deltaTime;
        double integralTerm = vxKi * vxIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = vxKd * (error - vxPreviousError) / deltaTime;

        // Combine terms to get the final output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Store current error and time for next iteration
        vxPreviousError = error;
        vxPreviousTime = currentTime;




        //normalize outputs
        if (output > maxPower){
            output = maxPower;
        }
        if (output < -maxPower){
            output = -maxPower;
        }


        // Return the output (e.g., motor power or other control signal)
        return output;

    }

    public double vyPID(double currentY, double currentTime, double target){
        // Calculate the error
        double error = target - currentY;

        // Calculate the time difference
        double deltaTime = currentTime - vyPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term
        double proportionalTerm = vyKp * error;

        // Integral term (sum of previous errors)
        vyIntegralSum += error * deltaTime;
        double integralTerm = vyKi * vyIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = vyKd * (error - vyPreviousError) / deltaTime;

        // Combine terms to get the final output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Store current error and time for next iteration
        vyPreviousError = error;
        vyPreviousTime = currentTime;


        //normalize outputs
        if (output > maxPower){
            output = maxPower;
        }
        if (output < -maxPower){
            output = -maxPower;
        }


        // Return the output (e.g., motor power or other control signal)
        return output;
    }


    public double calculateYaw(double x, double y, double h){





        return 2.0;

    }


    public double YawPID(double currentTheta, double currentTime,double target) {

        // Calculate the error
        double error = target - currentTheta;

        // Calculate the time difference
        double deltaTime = currentTime - yawPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term
        double proportionalTerm = yawKp * error;

        // Integral term (sum of previous errors)
        yawIntegralSum += error * deltaTime;
        double integralTerm = yawKi * yawIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = yawKd * (error - yawPreviousError) / deltaTime;

        // Combine terms to get the final output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Store current error and time for next iteration
        yawPreviousError = error;
        yawPreviousTime = currentTime;

        // Return the output (e.g., motor power or other control signal)

        //normalize outputs
        if (output > maxPower){
            output = maxPower;
        }
        if (output < -maxPower){
            output = -maxPower;
        }

        return output;
    }


}
