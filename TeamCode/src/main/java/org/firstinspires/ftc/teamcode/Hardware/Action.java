package org.firstinspires.ftc.teamcode.Hardware;



import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetHorizontalIdea;

import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetVerticalIdea;
public class Action {

    Position2d pose;
    double waitTime;
    currentDoHicky verticalDoHicky;
    currentDoHicky horizontalDoHicky;

    targetVerticalIdea verticalTargetAuto;// = targetVerticalIdea.COLLECT_SPECIMIN;
    targetHorizontalIdea horizontalTargetAuto;// = targetHorizontalIdea.ZERO_HS_SLIDES;

    // Constructor to initialize the action with a Pose2d and wait time
    public Action(Position2d pose, double waitTime, targetVerticalIdea verticalTargetAuto, targetHorizontalIdea targetHorizontalIdeaAuto) {
        this.pose = pose;
        this.waitTime = waitTime;
        this.verticalTargetAuto = verticalTargetAuto;
        this.horizontalTargetAuto = targetHorizontalIdeaAuto;
    }

    // Getter for Pose2d
    public Position2d getPose() {
        return pose;
    }

    // Getter for wait time
    public double getWaitTime() {return waitTime;}

    public targetHorizontalIdea getHorizontalTargetAuto(){

        return horizontalTargetAuto;

    }

    public targetVerticalIdea getVerticalTargetAuto(){

        return verticalTargetAuto;

    }


}
