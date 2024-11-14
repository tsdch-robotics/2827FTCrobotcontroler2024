package org.firstinspires.ftc.teamcode.Hardware;

public class ComputePid {

    private double yawKp, yawKi, yawKd;  // PID gains
    private double previousError = 0;
    private double integralSum = 0;
    private double target;      // Desired target value
    private double previousTime; // Time from previous iteration



    //takes postion, heading, wheel data

    //calculates wheel power using a pid equation, returns the holonomic drive values



    public double calculateLateral(double x, double y, double h){

        return 2.0;
    }

    public double calculateAxial(double x, double y, double h){
        return 2.0;
    }


    public double calculateYaw(double x, double y, double h){





        return 2.0;

    }


    public double YawPID(double currentTheta, double currentTime, double previousTime,double target) {


        // Calculate the error
        double error = target - currentTheta;

        // Calculate the time difference
        double deltaTime = currentTime - previousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term
        double proportionalTerm = yawKp * error;

        // Integral term (sum of previous errors)
        integralSum += error * deltaTime;
        double integralTerm = yawKi * integralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = yawKd * (error - previousError) / deltaTime;

        // Combine terms to get the final output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Store current error and time for next iteration
        previousError = error;
        previousTime = currentTime;

        // Return the output (e.g., motor power or other control signal)
        //return output;
    }


}
