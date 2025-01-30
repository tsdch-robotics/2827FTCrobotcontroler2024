package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@Config
public class VelocityAccelertaionSparkFun {


    public static double maxPower = 1;

    double changeTime = 0;

    double lastTime = 0;

    double changeY = 0;
    double lastY = 0;

    double changeX = 0;
    double lastX = 0;

    double changeH =0;
    double lastH = 0;

    double dydt = 0;
    double dxdt = 0;
    double dhdt = 0;

    double daydt = 0; //acceleration
    double daxdt = 0;
    double dahdt = 0;

    double lastDydt = 0;
    double lastDxdt = 0;
    double lastDhdt = 0;

    public static double fixedDeltaTime = 0.02;  // Fixed time step (50 Hz)
    //private static double accelUpdateRate = 0.04;

    public VxVyAxAy getvelocity(double time, double x2, double y2, double h2){

        //now using a fixed refresh rate (50Hz)
        if (time - lastTime >= fixedDeltaTime) {
            //speed calculations
            changeTime = time - lastTime;
            lastTime = time;

            double xpos2 = x2;
            double ypos2 = y2;
            double heading2 = h2;

            //prob convert to other class file
            changeY = ypos2 - lastY;
            lastY = ypos2;

            changeX = xpos2 - lastX;
            lastX = xpos2;

            changeH = heading2 - lastH;
            lastH = heading2;

            dydt = changeY/changeTime;
            dxdt = changeX/changeTime;
            dhdt = changeH/changeTime;

            daydt = (dydt-lastDydt)/changeTime;
            daxdt = (dxdt-lastDxdt)/changeTime;
            dahdt = (dhdt-lastDhdt)/changeTime;


            lastDydt = dydt;
            lastDxdt = dxdt;
            lastDhdt = dhdt;

        }

        VxVyAxAy velocities = new VxVyAxAy(dxdt,dydt,daxdt,daydt, dhdt, dahdt);

        return velocities;

    }

}
