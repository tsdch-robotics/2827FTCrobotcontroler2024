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

    public enum targetIdea{
        INIT,
        DEPOSIT_POTATO,
        PRE_SCORE_SPECIMEN,//when we under or over the bar right before
        SCORE_SPECIMEN,
        STALKER,
        SNATCH_THAT_FISHY,
        SQUEEZE_THE_CATCH,
        GET_THE_HELL_OUTA_HERE_FORWARDS,
        GET_THE_HELL_OUTA_HERE_BACKWARDS,
        RAISE_AND_PULL,//when we get a yellow or side color one
        COLLECT_SPECIMIN
    }


    public currentDoHicky magicalMacro(DcMotor HS, DcMotor VS,    targetIdea thisIdea){

        switch (thisIdea){

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
                ultimatePositions.shoulderPos = shoulderTest;//0
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


            case GET_THE_HELL_OUTA_HERE_FORWARDS:
                //ultimatePositions.intakePower = -1;
                //ultimatePositions.littleWheelPower = -1;
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