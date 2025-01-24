package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class determineColor {

    public String currentColor = "none";
    public double readSpacing = 0.1;//sec
    public double lastColorReading;

    public double totalRed = 0;
    public double totalGreen = 0;
    public double totalBlue = 0;

    public double readings = 0;

    public double avgRed = 0;

    public double avgBlue = 0;

    public double avgGreen = 0;



    public String color(ColorSensor colorSensor, double time){

        int colorGreen = colorSensor.green();
        int colorRed = colorSensor.red();
        int colorBlue = colorSensor.blue();

        totalRed += colorRed;
        totalGreen += colorGreen;
        totalBlue += colorBlue;

        readings += 1;

        if ((time - lastColorReading) > readSpacing) {


            avgRed = totalRed / readings;
            avgBlue = totalBlue / readings;
            avgGreen = totalGreen / readings;

            //then read
            if (avgRed > 600) {
                if (avgGreen > 600) {
                    currentColor = "yellow";
                } else {
                    currentColor = "red";
                }
            } else if (avgBlue > 600) {
                currentColor = "blue";
            }else {
                currentColor = "none";
            }

            totalRed = 0;
            totalGreen = 0;
            totalBlue = 0;

            lastColorReading = time;
            readings = 0;
        }else{

            currentColor = Double.toString(totalBlue);
        }



        /*if (colorRed > 600) {
            if (colorGreen > 600) {
                currentColor = "yellow";
            } else {
                currentColor = "red";
            }
        } else if (colorBlue > 600) {
            currentColor = "blue";
        } else {
            currentColor = "none";
        }
*/
        return(currentColor);
    }

}
