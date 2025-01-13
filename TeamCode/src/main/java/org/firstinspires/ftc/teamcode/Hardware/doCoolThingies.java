package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config
public class doCoolThingies {

    currentDoHicky ultimatePositions = new currentDoHicky(0,0,0,0,0,0,false);//change this to your ideal init pos


    public static double scoreHeight = 3400;
    public static double collectHeight = 100;
    public static double prescoreSpeciminHeight = 300;
    public static double scoreSpeciminHeight = 350;
    public static double stalkerHeight = 900;
    public static double snatchHeight = 600;

    public static double wristTestL = 0.5;
    public static double wristTestR = 0.4;

    public static double shoulderTest = 0.2;

    public static double full_HS = 1800;

    public static double intakeLOW = 0.16;

    public static double intakeHigh = 0;

    public static int hoverAcrosPos = 1200;

    public static int safePos = 900;

    public static double scoreSpeciminShoulderPos = 0.6;


    public enum targetVerticalIdea{
        INIT,
        DEPOSIT_POTATO,
        PRE_SCORE_SPECIMEN,//when we under or over the bar right before
        SCORE_SPECIMEN,
        STALKER,
        SNATCH_THAT_FISHY,
        SQUEEZE_THE_CATCH,
        COLLECT_SPECIMIN,
        SAFE_RAISE
        //all zeroing will be automatic

    }

    public enum targetHorizontalIdea{
        ZERO_HS_SLIDES,
        //BRING_IN_A_CATCH,
        READY_HS_POS,
        SAFE_POSITION,//intake doesn't need raise or lower
        HOVER_ACROSS_BARIER,
        FULL_EXTENT_DROP//maybe be like, +/- input?

    }


    public currentDoHicky magicalHorizontalMacro(targetHorizontalIdea thisVerticalIdea, double adjustment1){

        switch (thisVerticalIdea){

            case ZERO_HS_SLIDES:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = -100;//so the power ends up being full//-100

                break;

            case READY_HS_POS:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = 1;

                break;

            case HOVER_ACROSS_BARIER:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = hoverAcrosPos;

                break;
            case FULL_EXTENT_DROP:

                ultimatePositions.intakeLiftPos = intakeLOW;
                ultimatePositions.HSpos = full_HS;

                break;

            case SAFE_POSITION:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = safePos;

                break;


        }



        return(ultimatePositions);
    }


    public currentDoHicky magicalVerticalMacro(targetVerticalIdea thisVerticalIdea){

        switch (thisVerticalIdea){

            case SAFE_RAISE:
                ultimatePositions.VSpos = 3000;
                break;

            case DEPOSIT_POTATO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreHeight;//update
                ultimatePositions.shoulderPos = 0.17;//0.17
                ultimatePositions.wristLPos = .5;//.5
                ultimatePositions.wristRPos = .4;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case PRE_SCORE_SPECIMEN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = prescoreSpeciminHeight;//update
                ultimatePositions.shoulderPos = .5;//0.7
                ultimatePositions.wristLPos = wristTestL;
                ultimatePositions.wristRPos = wristTestR;
                ultimatePositions.clawState = true;//and declaring it prevents a user error
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case SCORE_SPECIMEN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreSpeciminHeight;//update
                ultimatePositions.shoulderPos = scoreSpeciminShoulderPos;
                ultimatePositions.wristLPos = wristTestL;
                ultimatePositions.wristRPos = wristTestR;
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case STALKER:
                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = stalkerHeight;//update//900
                ultimatePositions.shoulderPos = 0.85;//0.8
                ultimatePositions.wristLPos = wristTestL;
                ultimatePositions.wristRPos = wristTestR;
                ultimatePositions.clawState = false;
                break;

            case SNATCH_THAT_FISHY:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = snatchHeight;//update
                ultimatePositions.shoulderPos = 0.85;//0.5
                ultimatePositions.wristLPos = wristTestL;
                ultimatePositions.wristRPos = wristTestR;
                ultimatePositions.clawState = false;

                break;


            case SQUEEZE_THE_CATCH:
                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = snatchHeight;//update
                ultimatePositions.shoulderPos = shoulderTest;//0.5
                ultimatePositions.wristLPos = wristTestL;
                ultimatePositions.wristRPos = wristTestR;
                ultimatePositions.clawState = true;
                //close claw
                break;


            case COLLECT_SPECIMIN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = collectHeight;//update
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = wristTestL;
                ultimatePositions.wristRPos = wristTestR;
                ultimatePositions.clawState = false;

                break;

        }


        return(ultimatePositions);
    }


}
