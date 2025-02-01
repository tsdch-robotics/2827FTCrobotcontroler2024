package org.firstinspires.ftc.teamcode.Hardware;

public class currentDoHicky {


    double HSpos;
    double VSpos;
    double intakeLiftPos;
    double shoulderPos;
    double wristLPos;
    double wristRPos;//intake is the front of the robot this year
    double intakeSpeed; //for autonomous only
    double flickSpeed;//likewise
    boolean clawState;
    boolean PTO;
    boolean hookThat;

    public currentDoHicky(double HSpos, double VSpos,double intakeLiftPos, double shoulderPos, double wristLPos, double wristRPos, double intakeSpeed, double flickSpeed, boolean clawState, boolean PTO, boolean hookThat) {
        this.HSpos = HSpos;
        this.VSpos = VSpos;
        this.intakeLiftPos = intakeLiftPos;
        this.shoulderPos = shoulderPos;
        this.wristLPos = wristLPos;
        this.wristRPos = wristRPos;
        this.intakeSpeed = intakeSpeed;
        this.flickSpeed = flickSpeed;
        this.clawState = clawState;
        this.hookThat = hookThat;
        this.PTO = PTO;
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

    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    public double getFlickSpeed(){return flickSpeed;}

    public boolean getClawState() {
        return clawState;
    }

    public boolean getPTO(){

        return PTO;
    }
    public boolean getHookThat(){

        return hookThat;
    }


}