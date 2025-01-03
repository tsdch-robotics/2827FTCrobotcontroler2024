package org.firstinspires.ftc.teamcode.Hardware;

public class currentDoHicky {


    double HSpos;
    double VSpos;
    double intakeLiftPos;
    double shoulderPos;
    double wristLPos;
    double wristRPos;//intake is the front of the robot this year

    public currentDoHicky(double HSpos, double VSpos,double intakeLiftPos, double shoulderPos, double wristLPos, double wristRPos) {
        this.HSpos = HSpos;
        this.VSpos = VSpos;
        this.intakeLiftPos = intakeLiftPos;
        this.shoulderPos = shoulderPos;
        this.wristLPos = wristLPos;
        this.wristRPos = wristRPos;
    }


    public double getHSpos() {
        return HSpos;
    }

    public double getVSpos() {
        return VSpos;
    }

    public double getintakeLiftPos() {
        return intakeLiftPos;
    }

    public double getshoulderPos() {
        return shoulderPos;
    }

    public double getwristLPos() {
        return wristLPos;
    }

    public double getwristRPos() {
        return wristRPos;
    }


}