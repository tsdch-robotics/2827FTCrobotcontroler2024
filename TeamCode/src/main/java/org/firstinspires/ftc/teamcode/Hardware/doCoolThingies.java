package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class doCoolThingies {

    currentDoHicky ultimatePositions = new currentDoHicky(0,0,0,0,0,0);//change this to your ideal init pos

    public enum targetIdea{
        DEPOSIT_POTATO,
        PRE_SCORE_SPECIMEN,//when we under or over the bar right before
        SCORE_SPECIMEN,
        STALKER,
        SNATCH_THAT_FISHY,
        SQUEEZE_THE_CATCH,
        GET_THE_HELL_OUTA_HERE_FORWARDS,
        GET_THE_HELL_OUTA_HERE_BACKWARDS,
        RAISE_AND_PULL//when we get a yellow or side color one

    }


    public currentDoHicky magicalMacro(DcMotor HS, DcMotor VS,    targetIdea thisIdea){


        switch (thisIdea){

            case DEPOSIT_POTATO:

                ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 1000;//update
                ultimatePositions.shoulderPos = 1;
                ultimatePositions.wristLPos = 0.5;
                ultimatePositions.wristRPos = 0.5;
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case PRE_SCORE_SPECIMEN:

                ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 800;//update
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = 0.5;
                ultimatePositions.wristRPos = 0.5;
                ultimatePositions.clawPos = true;//and declaring it prevents a user error
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case SCORE_SPECIMEN:

                ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 880;//update
                ultimatePositions.shoulderPos = 0;
                ultimatePositions.wristLPos = 0.5;
                ultimatePositions.wristRPos = 0.5;
                //ultimatePositions.clawPos = false;
                //basically if a position isn't declared in this scope, it will allow the drvier to specifically control that feature

                break;

            case STALKER:
                ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 700;//update
                ultimatePositions.shoulderPos = 0.5;
                ultimatePositions.wristLPos = 0.5;
                ultimatePositions.wristRPos = 0.5;
                ultimatePositions.clawPos = false;
                break;

            case SNATCH_THAT_FISHY:

                ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 600;//update
                ultimatePositions.shoulderPos = 0.5;
                ultimatePositions.wristLPos = 0.5;
                ultimatePositions.wristRPos = 0.5;
                ultimatePositions.clawPos = false;

                break;


            case SQUEEZE_THE_CATCH:
                ultimatePositions.HSpos = 0;
                ultimatePositions.VSpos = 600;//update
                ultimatePositions.shoulderPos = 0.5;
                ultimatePositions.wristLPos = 0.5;
                ultimatePositions.wristRPos = 0.5;
                ultimatePositions.clawPos = true;
                //close claw
                break;


            case GET_THE_HELL_OUTA_HERE_FORWARDS:
                ultimatePositions.intakePower = -1;
                ultimatePositions.littleWheelPower = -1;
                break;

        }


        return(ultimatePositions);
    }


}
