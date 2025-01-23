package org.firstinspires.ftc.teamcode.Hardware;

public class Position2d {

    double x;
    double y;
    double heading;

    public Position2d(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getThisX() {
        return x;
    }

    public double getThisY() {
        return y;
    }

    public double getThisHeading() {
        return heading;
    }

}