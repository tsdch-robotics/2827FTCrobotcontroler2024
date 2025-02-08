package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;


@Config
public class doCoolThingies {

    currentDoHicky ultimatePositions = new currentDoHicky(0,0,0,0,0,0,0, 0, false, false, false);//change this to your ideal init pos


    public static double scoreHeight = 3500;
    public static double collectHeight = 2;
    public static double prescoreSpeciminHeight = 700;
    public static double scoreSpeciminHeight = 800;
    public static double stalkerHeight = 600;
    public static double snatchHeight = 350;

    public static double wristTestL = 0.51;//was.5
    public static double wristTestR = 0.41;//was.4

    public static double shoulderTest = 0.2;

    public static double full_HS = 1800;

    public static double intakeLOW = 0.16;

    public static double intakeHigh = 0;

    public static int hoverAcrosPos = 1200;

    public static int safePos = 900;

    public static double scoreSpeciminShoulderPos = 0.41;

    public static double rHang1 = 2000;
    public static double rHang2 = 1900;

    public static double accendPull = 300;

    public static double intakeRest = 75;

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
        ZERO_COLLECT,
        COLLECT_SPECIMIN,
        COLLECT_SPECIMIN_AUTO,
        SAFE_RAISE,
        RELEASE,
        PRE_SCORE_SPECIMEN_AUTO,
        SCORE_SPECIMEN_AUTO,
        READY_TO_HANG1,
        READY_TO_HANG2,
        ENGAGE_PTO,
        ACCEND,
        ENGANGE_SUPPORT,

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
                ultimatePositions.HSpos = intakeRest;
                ultimatePositions.intakeSpeed = 0;//put elsewhere?
                ultimatePositions.flickSpeed = 0;

                break;

            case READY_HS_POS_FLICK_STILL_ON:

                ultimatePositions.intakeLiftPos = intakeHigh;
                ultimatePositions.HSpos = intakeRest;
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
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
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
                ultimatePositions.shoulderPos = 0.21;
                ultimatePositions.wristLPos = .51;//.5
                ultimatePositions.wristRPos = .41;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;


            case DEPOSIT_POTATO_AUTO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreHeight;//update
                ultimatePositions.shoulderPos = 0.21;//0.22 + 0.05;//0.17
                ultimatePositions.wristLPos = .51;//.5
                ultimatePositions.wristRPos = .41;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case PRE_SCORE_SPECIMEN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = prescoreSpeciminHeight - 100;//update
                ultimatePositions.shoulderPos = 0.48;//.48
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.clawState = true;//and declaring it prevents a user error
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;


            case PRE_SCORE_SPECIMEN_AUTO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = prescoreSpeciminHeight - 100;//update
                ultimatePositions.shoulderPos = 0.55;//.48
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.clawState = true;//and declaring it prevents a user error
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case SCORE_SPECIMEN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreSpeciminHeight;//update
                ultimatePositions.shoulderPos = 0.3;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;


            case SCORE_SPECIMEN_AUTO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreSpeciminHeight -100;//update
                ultimatePositions.shoulderPos = 0.3;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case STALKER:
                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = stalkerHeight;//update//900
                ultimatePositions.shoulderPos = 0.935;//0.87 + 0.05;//0.85
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.clawState = false;
                break;

            case SNATCH_THAT_FISHY:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = snatchHeight;//update
                ultimatePositions.shoulderPos = 0.935
                ;//0.87 + 0.05;//0.85 - i reduce to .8 for better potential
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.clawState = false;

                break;


            case SQUEEZE_THE_CATCH:
                //ultimatePositions.HSpos = 0;

                ultimatePositions.clawState = true;
                //close claw
                break;

            case ZERO_COLLECT:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = -1000;//it was -500 on friday 1/24
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = .57;
                ultimatePositions.wristRPos = .47;
                ultimatePositions.clawState = false;

                break;

            case COLLECT_SPECIMIN:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 0;//update
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = .57;
                ultimatePositions.wristRPos = .47;
                ultimatePositions.clawState = false;

                break;

            case COLLECT_SPECIMIN_AUTO:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = collectHeight;//update
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = .57;
                ultimatePositions.wristRPos = .47;
                ultimatePositions.clawState = false;

                break;


            case RELEASE:

                ultimatePositions.clawState = false;

                break;


            case RELEASE_POTATO:


                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = scoreHeight;//update
                ultimatePositions.shoulderPos = 0.21;//0.16 + 0.05;//0.17
                ultimatePositions.wristLPos = .51;//.5
                ultimatePositions.wristRPos = .41;//.4
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                ultimatePositions.clawState = false;

                break;

            case PARK:

                ultimatePositions.VSpos = 1;
                ultimatePositions.shoulderPos = 0.5;//scoreSpeciminShoulderPos + 0.05;//.3
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;

                break;


            case READY_TO_HANG1:

                ultimatePositions.VSpos = rHang1;
                ultimatePositions.shoulderPos = 0.6;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;

                break;

            case READY_TO_HANG2:

                ultimatePositions.VSpos = rHang2;
                ultimatePositions.shoulderPos = 0.6;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;

                break;


            case ENGAGE_PTO:

                ultimatePositions.VSpos = rHang2;
                ultimatePositions.shoulderPos = 0.6;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.PTO = true;

                break;

            case ACCEND:

                ultimatePositions.VSpos = accendPull;
                ultimatePositions.shoulderPos = 0.6;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.PTO = true;

                break;


            case ENGANGE_SUPPORT:

                ultimatePositions.VSpos = accendPull;
                ultimatePositions.shoulderPos = 0.6;
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.PTO = true;
                ultimatePositions.hookThat = true;

                break;

            case INIT:

                //ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = prescoreSpeciminHeight;//update
                ultimatePositions.shoulderPos = 0.5;//.48
                ultimatePositions.wristLPos = .51;
                ultimatePositions.wristRPos = .41;
                ultimatePositions.clawState = true;//and declaring it prevents a user error
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;


        }


        return(ultimatePositions);
    }


}
