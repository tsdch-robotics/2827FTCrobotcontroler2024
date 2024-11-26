package org.firstinspires.ftc.teamcode.Hardware;

public class VxVyAxAy {

    double Vx;
    double Vy;
    double Ax;
    double Ay;
    double Vh;
    double Ah;

    public VxVyAxAy(double Vx, double Vy,double Ax, double Ay, double Vh, double Ah) {
        this.Vx = Vx;
        this.Vy = Vy;
        this.Ax = Ax;
        this.Ay = Ay;
        this.Vh = Vh;
        this.Ah = Ah;
    }


    public double getVx() {
        return Vx;
    }

    public double getVy() {
        return Vy;
    }

    public double getAx() {
        return Ax;
    }

    public double getAy() {
        return Ay;
    }


    public double getVh() {
        return Vh;
    }


    public double getAh() {
        return Ah;
    }


}



/*package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.Pose2d;

public class thr {
}

    // Constructor to initialize the action with a Pose2d and wait time
    public VxVyAxAy(double Vx, double Vy,double Ax, double Ay) {
        this.Vx = Vx;
        this.Vy = Vy;
        this.Ax = Ax;
        this.Ay = Ay;
    }

    // Getter for Pose2d
    public Pose2d getPose() {
        return pose;
    }

    // Getter for wait time
    public double getWaitTime() {
        return waitTime;
    }
}
*/