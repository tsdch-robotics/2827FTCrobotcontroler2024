package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.Hardware.KalmanThatVelo;

@Config
public class VelocityAccelertaionSparkFun {


    public static double maxPower = 1;

    public double changeTime = 0;

    public double lastTime = 0;

    public double changeY = 0;
    public double lastY = 0;

    public double changeX = 0;
    public double lastX = 0;

    public double changeH =0;
    public double lastH = 0;

    public double dydt = 0;
    public double dxdt = 0;
    public double dhdt = 0;

    public double daydt = 0; //acceleration
    public double daxdt = 0;
    public double dahdt = 0;

    public double lastDydt = 0;
    public double lastDxdt = 0;
    public double lastDhdt = 0;

    public static double fixedDeltaTime = 0.02;  // Fixed time step (50 Hz)
    //private static double accelUpdateRate = 0.04;
//no longer spark fun

    private KalmanThatVelo kalmanFilterX = new KalmanThatVelo(0);
    private KalmanThatVelo kalmanFilterY = new KalmanThatVelo(0);
    private KalmanThatVelo kalmanFilterH = new KalmanThatVelo(0);


    private KalmanThatAcc kalmanFilterAX = new KalmanThatAcc(0);
    private KalmanThatAcc kalmanFilterAY = new KalmanThatAcc(0);
    private KalmanThatAcc kalmanFilterAH = new KalmanThatAcc(0);

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

            //Kalman filter here
            dxdt = kalmanFilterX.update(dxdt);
            dydt = kalmanFilterY.update(dydt);
            dhdt = kalmanFilterH.update(dhdt);

            daydt = (dydt-lastDydt)/changeTime;//acceleration?
            daxdt = (dxdt-lastDxdt)/changeTime;
            dahdt = (dhdt-lastDhdt)/changeTime;

            daxdt = kalmanFilterAX.update(daxdt);
            daydt = kalmanFilterAY.update(daydt);
            dahdt = kalmanFilterAH.update(dahdt);

            lastDydt = dydt;
            lastDxdt = dxdt;
            lastDhdt = dhdt;

        }

        VxVyAxAy velocities = new VxVyAxAy(dxdt,dydt,daxdt,daydt, dhdt, dahdt);

        return velocities;

    }

}
