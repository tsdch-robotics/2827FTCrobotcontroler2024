package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config
public class doCoolThingies {

    currentDoHicky ultimatePositions = new currentDoHicky(0,0,0,0,0,0,0, 0, false);//change this to your ideal init pos


    public static double scoreHeight = 3500;
    public static double collectHeight = 2;
    public static double prescoreSpeciminHeight = 1700;
    public static double scoreSpeciminHeight = 1900;
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

    public static double scoreSpeciminShoulderPos = 0.41;


    public enum targetVerticalIdea{
        PARK,
        PRE_ZERO,
        ZERO_VS_SLIDES,
        READY_VS_POS,
        INIT,
        DEPOSIT_POTATO,
        DEPOSIT_POTATO_AUTO,
        RELEASE_POTATO,
        PRE_SCORE_SPECIMEN,//when we under or over the bar right before
        SCORE_SPECIMEN,
        STALKER,
        SNATCH_THAT_FISHY,
        SQUEEZE_THE_CATCH,
        COLLECT_SPECIMIN,
        COLLECT_SPECIMIN_AUTO,
        SAFE_RAISE,
        RELEASE,

        //all zeroing will be automatic

    }

    public enum targetHorizontalIdea{
        ZERO_HS_SLIDES,
        //BRING_IN_A_CATCH,
        READY_HS_POS,
        SAFE_POSITION,//intake doesn't need raise or lower
        HOVER_ACROSS_BARIER,
        FULL_EXTENT_DROP_WITH_INTAKE, //for autoonomous only
        ZERO_HS_SLIDES_FLICK_ON,//likewised
        READY_HS_POS_FLICK_STILL_ON,
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
                ultimatePositions.intakeSpeed = 0;//put elsewhere?
                ultimatePositions.flickSpeed = 0;

                break;

            case READY_HS_POS_FLICK_STILL_ON:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = 1;
                ultimatePositions.intakeSpeed = 0;//put elsewhere?


                break;

            case HOVER_ACROSS_BARIER:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = hoverAcrosPos;

                break;
            case FULL_EXTENT_DROP:

                ultimatePositions.intakeLiftPos = intakeLOW;
                ultimatePositions.HSpos = full_HS;

                break;


            case FULL_EXTENT_DROP_WITH_INTAKE:

                ultimatePositions.intakeLiftPos = intakeLOW;
                ultimatePositions.HSpos = full_HS;
                ultimatePositions.intakeSpeed = 1;

                break;

            case SAFE_POSITION:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = safePos;

                break;

            case ZERO_HS_SLIDES_FLICK_ON:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = -100;//so the power ends up being full//-100
                ultimatePositions.flickSpeed = -1; // pull in

                break;








        }



        return(ultimatePositions);
    }


    public currentDoHicky magicalVerticalMacro(targetVerticalIdea thisVerticalIdea){

        switch (thisVerticalIdea){


            case PRE_ZERO:

                ultimatePositions.shoulderPos = 0.2;
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;
                ultimatePositions.clawState = false;

                ultimatePositions.VSpos = 2000;//so the power ends up being full//-100

                break;
            case ZERO_VS_SLIDES:

                ultimatePositions.VSpos = -500;//so the power ends up being full//-100
                ultimatePositions.clawState = true;

                break;

            case READY_VS_POS:

                ultimatePositions.VSpos = 300;
/*
                ultimatePositions.shoulderPos = 0.85;
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;
                ultimatePositions.clawState = false;*/

                break;

            case SAFE_RAISE:
                ultimatePositions.VSpos = 3000;
                break;

            case DEPOSIT_POTATO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreHeight;//update
                ultimatePositions.shoulderPos = 0.21;//0.17//makes it go in a bit better
                ultimatePositions.wristLPos = .5;//.5
                ultimatePositions.wristRPos = .4;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;


            case DEPOSIT_POTATO_AUTO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreHeight;//update
                ultimatePositions.shoulderPos = 0.22;//0.17
                ultimatePositions.wristLPos = .5;//.5
                ultimatePositions.wristRPos = .4;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case PRE_SCORE_SPECIMEN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = prescoreSpeciminHeight;//update
                ultimatePositions.shoulderPos = .5;//0.7
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;
                ultimatePositions.clawState = true;//and declaring it prevents a user error
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case SCORE_SPECIMEN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreSpeciminHeight;//update
                ultimatePositions.shoulderPos = scoreSpeciminShoulderPos;//.3
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case STALKER:
                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = stalkerHeight;//update//900
                ultimatePositions.shoulderPos = 0.85;//0.85
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;
                ultimatePositions.clawState = false;
                break;

            case SNATCH_THAT_FISHY:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = snatchHeight;//update
                ultimatePositions.shoulderPos = 0.85;//0.85 - i reduce to .8 for better potential
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;
                ultimatePositions.clawState = false;

                break;


            case SQUEEZE_THE_CATCH:
                //ultimatePositions.HSpos = 0;

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

            case COLLECT_SPECIMIN_AUTO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = collectHeight;//update
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = .45;
                ultimatePositions.wristRPos = .35;
                ultimatePositions.clawState = false;

                break;


            case RELEASE:

                ultimatePositions.clawState = false;

                break;


            case RELEASE_POTATO:


                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreHeight;//update
                ultimatePositions.shoulderPos = 0.16;//0.17
                ultimatePositions.wristLPos = .5;//.5
                ultimatePositions.wristRPos = .4;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                ultimatePositions.clawState = false;

                break;

            case PARK:

                ultimatePositions.VSpos = 1;
                ultimatePositions.shoulderPos = scoreSpeciminShoulderPos;//.3
                ultimatePositions.wristLPos = .5;
                ultimatePositions.wristRPos = .4;

                break;





        }


        return(ultimatePositions);
    }


}
