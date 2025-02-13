package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.DcMotor;

public class GetWheeledLocalization {

    double L = 6.4;//inches between odo 1 and 2
    double B = 3.5;//actually? 9.125;//dist between midpoint of 1&2 with 3
    double R = 0.6856299;// omni wheels radius
    double N = 8192; //ticks per revolution
    double inches_per_tick = 2 * Math.PI * R / N;

    double posX;
    double posY;
    double posH;

    Position2d positionLocalization;

    double curRight = 0;
    double curLeft = 0;
    double curAngle = 0;

    public Position2d wheeledLocalization(DcMotor encoder1, DcMotor encoder2, DcMotor encoder3, double posX, double posY, double posH){

        this.posX = posX;
        this.posY = posY;
        this.posH = posH;

        double enc1val = encoder1.getCurrentPosition();
        double enc2val = encoder2.getCurrentPosition();
        double enc3val = encoder3.getCurrentPosition();

        double oldRight = curRight;
        double oldLeft = curLeft;
        double oldAngle = curAngle;

        curRight = enc2val; //right
        curLeft = enc1val;//left
        curAngle = enc3val;//angle

        //current config, [] = mecanum wheel, () = odometr pod, X = center
        //         []-()---()--[]
        //          ------------
        //          -----x------
        //          ------------
        //         []----()----[]

        double dn1 = curLeft - oldLeft;
        double dn2 = curRight - oldRight;
        double dn3 = curAngle - oldAngle;

        double dtheta = inches_per_tick * (dn2-dn1) / L;
        double dy = inches_per_tick * (dn2-dn1) / 2.0;
        double dx = inches_per_tick * (dn3 - (dn2-dn1) * B / L);

        double ticksPerInchY = 121.698611;

        double ticksPerRadianY = 37215 / (2*Math.PI);

        double ticksPerRadianX = 45119 / (2*Math.PI);

        //dy = (((dn2 + dn1)/2) - ticksPerRadianY * dtheta) * inches_per_tick;
        dy = ((dn2) - ticksPerRadianY * dtheta) * inches_per_tick;
        dx = (dn3 - ticksPerRadianX * dtheta) * inches_per_tick;


        posH += dtheta /2.0;//with average at this point
        posX += dx * Math.cos(posH) - dy * Math.sin(posH);
        posY += dx * Math.sin(posH) + dy * Math.cos(posH);
        posH += dtheta / 2.0;

        positionLocalization = new Position2d(posX,posY,posH);

        return positionLocalization;

    }

}
