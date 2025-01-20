package org.firstinspires.ftc.teamcode.Hardware;


import com.acmerobotics.roadrunner.Pose2d;


public class Action {
    Pose2d pose;
    double waitTime;
    currentDoHicky verticalDoHicky;
    currentDoHicky horizontalDoHicky;


    doCoolThingies.targetVerticalIdea verticalTargetAuto = doCoolThingies.targetVerticalIdea.COLLECT_SPECIMIN;
    doCoolThingies.targetHorizontalIdea horizontalTargetAuto = doCoolThingies.targetHorizontalIdea.ZERO_HS_SLIDES;

    // Constructor to initialize the action with a Pose2d and wait time
    public Action(Pose2d pose, double waitTime, currentDoHicky verticalDoHicky, currentDoHicky horizontalDoHicky) {
        this.pose = pose;
        this.waitTime = waitTime;
        this.verticalDoHicky = verticalDoHicky;
        this.horizontalDoHicky = horizontalDoHicky;
    }

    // Getter for Pose2d
    public Pose2d getPose() {
        return pose;
    }

    // Getter for wait time
    public double getWaitTime() {return waitTime;}

    public currentDoHicky getHorizontalDoHicky(){



        return horizontalDoHicky;

    }

    public currentDoHicky getVerticalDoHicky(){return verticalDoHicky;}

}
