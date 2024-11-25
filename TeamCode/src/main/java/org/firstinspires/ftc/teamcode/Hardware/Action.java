package org.firstinspires.ftc.teamcode.Hardware;


import com.acmerobotics.roadrunner.Pose2d;


public class Action {
    Pose2d pose;
    double waitTime;

    // Constructor to initialize the action with a Pose2d and wait time
    public Action(Pose2d pose, double waitTime) {
        this.pose = pose;
        this.waitTime = waitTime;
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
