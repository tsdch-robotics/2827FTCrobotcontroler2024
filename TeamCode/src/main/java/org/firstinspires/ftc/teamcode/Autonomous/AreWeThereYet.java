package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AreWeThereYet {


    double test = 1;
    public static double stopSpeed = 0.01;
    public static double deadStall = 0.001;

    boolean inTargetBox;

    boolean continuous = false;


    public boolean weThere (double xe, double ye, double ae, double vx, double vy, double va, double currentWait){

        if(currentWait < 0){
            continuous = true;
        }else{
            continuous = false;
        }

        xe = Math.abs(xe);
        ye = Math.abs(ye);
        ae = Math.abs(ae);//scalar

        //chekcs to see if we are in target box and if speed is 0
        if ((xe < 2 & vx < stopSpeed) /*|| vx < deadStall*/){
            if ((ye < 2 & vy < stopSpeed) /*|| vy < deadStall*/){
                if (va < stopSpeed) /*|| velocities.getVh() < 0.0001*/{
                    inTargetBox = true;
                }else{
                    inTargetBox = false;
                }
            }else{
                inTargetBox = false;
            }
        }else{
            inTargetBox = false;
        }

        if(continuous){
            if (xe < Math.abs(currentWait) & ye < Math.abs(currentWait)/* & ae < Math.abs(5 * currentWait)*/){//2inch would becom 10 degrees
                //current wait = a distance in this case
                inTargetBox = true;
            }else{
                inTargetBox = false;
            }
        }


        return(inTargetBox);

    }

}
