package org.firstinspires.ftc.teamcode.Hardware;

import android.text.method.Touch;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.sun.tools.javac.tree.DCTree;

@Config
public class ComputePid {

    /*public static */double yawKp = 0.8, yawKi = 0.05, yawKd = 0.001;  // PID gains

    public static double vxKp = 0.06, vxKi = 0.001, vxKd = 0.01;//OLD CONSTANTS
    public static double vxKpV = 0.007, vxKiV = 0, vxKdV = 0;

    //double vyKp = vxKp;
    //double vyKi = vxKi;
    //double vyKd = vxKd;

    public static double vyKp = 0.06, vyKi = 0.001, vyKd = 0.01;//OLD CONSTNATS
    public static double vyKpV = 0.007, vyKiV = 0, vyKdV = 0;

    /*public static */double hsKp = 0.003, hsKi = 0.0, hsKd = 0.0;  // PID gains
    /*public static */double vsKp = 0.003, vsKi = 0.0, vsKd = 0.0;  // PID gains


    public double hsPreviousError = 0;
    public double hsIntegralSum = 0;
    public double hsPreviousTime = 0; // Time from previous iteration


    public double vsPreviousError = 0;
    public double vsIntegralSum = 0;
    public double vsPreviousTime = 0; // Time from previous iteration

    public double yawPreviousError = 0;
    public double yawIntegralSum = 0;
    public double yawPreviousTime = 0; // Time from previous iteration

    public double vxPreviousError = 0;
    public double vxIntegralSum = 0;
    public double vxPreviousTime = 0; // Time from previous iteration

    public double vyPreviousError = 0;
    public double vyIntegralSum = 0;
    public double vyPreviousTime = 0; // Time from previous iteration

    /*public static */double maxPower = 1;

    /*public static */double Kf = 0.5;  // Feedforward constant, tune this value
    /*public static*/ double mass = 13.6078;  // Robot mass (in kg), set to your robot's mass

    //takes postion, heading, wheel data

    //calculates wheel power using a pid equation, returns the holonomic drive values


    public static double maxDecc = -50;


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


        //feed forward kinamatics:

       //double feedforwardTerm = Kf * (v_target + a_target * mass);

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


    public double vxPIDVeloConcious(double currentX, double currentVeloX, double currentTime,double target){


        // Calculate the error
        double errorP = target - currentX;


        double signOfError = errorP/(Math.abs(errorP));

        double hypoVelo = signOfError * Math.sqrt(Math.abs(2 * maxDecc * errorP)) + 0.0001/*so it shows*/;//hypothetically, the should be current velo

        double error = hypoVelo - currentVeloX;



        // Calculate the time difference
        double deltaTime = currentTime - vxPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term

        double newKp = Math.abs(vxKpV * errorP);

        if (newKp >= 1){
            newKp = 1;
        }//add a threshold so it doesn't get too low?

        double proportionalTerm = newKp * error;//vxKpV * error



        // Integral term (sum of previous errors)
        vxIntegralSum += error * deltaTime;
        double integralTerm = vxKiV * vxIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = vxKdV * (error - vxPreviousError) / deltaTime;


        //feed forward kinamatics:

        //double feedforwardTerm = Kf * (v_target + a_target * mass);

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
        double output = /*inverseK * */(proportionalTerm + integralTerm + derivativeTerm);


        /*hypoVelo = Math.sqrt((vInst ** 2) + 2 * maxAcc * error);//hypo velo is the velo upon arrival

        if (hypoVelo >= 0.01){

        }*/

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




    public double vyPIDVeloConcious(double currentY, double currentVeloY, double currentTime, double target){


        // Calculate the error
        double errorP = target - currentY;


        double signOfError = errorP/(Math.abs(errorP));

        double hypoVelo = signOfError * Math.sqrt(Math.abs(2 * maxDecc * errorP)) + 0.0001/*so it shows*/;//hypothetically, the should be current velo

        double error = hypoVelo - currentVeloY;


        // Calculate the time difference
        double deltaTime = currentTime - vyPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term


        double newKp = Math.abs(vyKpV * errorP);

        if (newKp >= 1){
            newKp = 1;
        }//add a threshold so it doesn't get too low?

        double proportionalTerm = newKp * error;//vxKpV * error

        // Integral term (sum of previous errors)
        vyIntegralSum += error * deltaTime;
        double integralTerm = vyKiV * vyIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = vyKdV * (error - vyPreviousError) / deltaTime;

        // Combine terms to get the final output
        double output = /*inverseK * */(proportionalTerm + integralTerm + derivativeTerm);


        /*hypoVelo = Math.sqrt((vInst ** 2) + 2 * maxAcc * error);//hypo velo is the velo upon arrival

        if (hypoVelo >= 0.01){

        }*/

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



    public double hsPID(double currentDist, double currentTime,double target) {

        // Calculate the error
        double error = target - currentDist;

        // Calculate the time difference
        double deltaTime = currentTime - hsPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term
        double proportionalTerm = hsKp * error;

        // Integral term (sum of previous errors)
        hsIntegralSum += error * deltaTime;
        double integralTerm = hsKi * hsIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = hsKd * (error - hsPreviousError) / deltaTime;

        // Combine terms to get the final output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Store current error and time for next iteration
        hsPreviousError = error;
        hsPreviousTime = currentTime;

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




    public double vsPID(double currentDist, double currentTime,double target) {

        // Calculate the error
        double error = target - currentDist;

        // Calculate the time difference
        double deltaTime = currentTime - vsPreviousTime;
        if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

        // Proportional term
        double proportionalTerm = vsKp * error;

        // Integral term (sum of previous errors)
        vsIntegralSum += error * deltaTime;
        double integralTerm = vsKi * vsIntegralSum;

        // Derivative term (rate of change of error)
        double derivativeTerm = vsKd * (error - vsPreviousError) / deltaTime;

        // Combine terms to get the final output
        double output = proportionalTerm + integralTerm + derivativeTerm;

        // Store current error and time for next iteration
        vsPreviousError = error;
        vsPreviousTime = currentTime;

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

    /*public void manageZeroing(DcMotor hsSlides, DcMotor vsSlides, TouchSensor touch){

    }*/

}
