package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Action;
import org.firstinspires.ftc.teamcode.Hardware.ComputePid;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Mat;
//import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Hardware.Action;



import java.util.List;
import java.util.ArrayList;


/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@Config
@Autonomous(name = "BlueLeftSide", group = "Autonomous")
public class BlueLeftSide extends LinearOpMode {


    public boolean inTargetBox = false;
    public int actionNumber = 0;

    // Create an instance of the sensor

    SparkFunOTOS myOtos;

    Action act1 = new Action(new Pose2d(-7,-61,Math.toRadians(0)), 0);
    Action act2 = new Action(new Pose2d(-5, -35, Math.toRadians(0)), 0);
    Action act3 = new Action(new Pose2d(-5.1, -35, Math.toRadians(0)), 3);
    Action act4 = new Action(new Pose2d(-10, -40, Math.toRadians(-90)), .2);
    Action act5 = new Action(new Pose2d(-50, -40, Math.toRadians(-90)), 0);
    Action act6 = new Action(new Pose2d(-50, -40, Math.toRadians(0)), 1);

    List<Action> actions = new ArrayList<>();

    double act1pause = 1;
    double act2pause = 1;
    double act3pause = 1;
    double act4pause = 1;
    double act5pause = 1;
    double act6pause = 1;


    List<Pose2d> pauses = new ArrayList<>();


    public static double testttt = 1; // Example of FTC dashboard variable

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo servo = null;

    double finalX = 0;
    double finalY = 0;
    double originY = -61;
    double originX = -7;

    double normalHeading = 0;
    double currentDriveX = 0;
    double currentDriveY = 0;


    public static double yawKp = 0.7, yawKi = 0.01, yawKd = 0.001;  // PID gains

    private double previousError = 0;
    private double integralSum = 0;
    public static double yawTarget = (0);   // Desired target value
    public static double targetX = -50;
    public static double targetY = -50;

    double localXTarget = 0;
    double localYTarget = 0;

    private double previousTime = 0; // Time from previous iteration
    private double yawOutput = 0;
    private double vxOutput = 0;
    private double vyOutput = 0;

    public double heading = 0;

    ComputePid PID = new ComputePid();


    double endTime = 0;
    boolean AlreadyPausing = false;


    //speed calculation params
    double changeTime = 0;
    double lastTime = 0;

    double changeY = 0;
    double lastY = 0;

    double changeX = 0;
    double lastX = 0;

    double changeH = 0;
    double lastH = 0;

    double dydt = 0;
    double dxdt = 0;
    double dhdt = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        actions.add(act1);
        actions.add(act2);
        actions.add(act3);
        actions.add(act4);
        actions.add(act5);
        actions.add(act6);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        servo = hardwareMap.get(Servo.class, "servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        configureOtos();

        // Wait for the start button to be pressed
        waitForStart();
        resetRuntime();



        // Loop until the OpMode ends
        while (opModeIsActive()) {

            double stopSpeed = 0.0001;
            double noPauseLeft;

            Action currentAction = actions.get(actionNumber); // Get the current action
            Pose2d targetPose = currentAction.getPose(); // Get the Pose2d from the current action
            double waitTime = currentAction.getWaitTime(); // Get the wait time from the current action

            //chekcs to see if we are in target box and if speed is 0
            if (Math.abs(finalX - targetX) < 1 & dxdt < stopSpeed){
                if (Math.abs(finalY - targetY) < 1 & dydt < stopSpeed){
                    if (/*Math.abs(heading - yawTarget) < 0.0872665 & */dhdt < 0.001){
                        inTargetBox = true;
                        gamepad1.rumble(3);
                    }
                }
            }
            //next we must complete the pause

            //once in targetBox, it starts the pause
            if(inTargetBox & !AlreadyPausing){
                endTime = getRuntime() + waitTime;
                AlreadyPausing = true;
            }

            if (inTargetBox & actionNumber < 5 & getRuntime() > endTime){
                AlreadyPausing = false;
                actionNumber += 1;
                inTargetBox = false;
            }


            targetX = targetPose.position.x;
            targetY = targetPose.position.y;
            yawTarget = Math.toDegrees(targetPose.heading.log());
            telemetry.addData("yawTarget", yawTarget);

            double TelemX = -finalX;
            double TelemY = -finalY;//bc for some reason this works

            double adjH = normalHeading - (3.14159265/2);
            double[] xcordHead = {TelemX - 3, TelemX + 3, TelemX + 15 * Math.cos(normalHeading), TelemX + 15 * Math.cos(normalHeading)};
            double[] ycordHead = {TelemY, TelemY, TelemY + 15 * Math.sin(normalHeading), TelemY + 15 * Math.sin(normalHeading)};

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("cyan")
                    .fillCircle(TelemX, TelemY, 6)
                    .fillCircle(TelemX + 15 * Math.cos(adjH), TelemY + 15 * Math.sin(adjH), 5)
                    .setFill("red")
                    .fillCircle(-targetX, -targetY, 5);
            //.setFill("green")//to show power outputs
            //.fillCircle(TelemX + 15 * Math.cos(adjH), TelemY + 15 * Math.sin(adjH), 2);

            dashboard.sendTelemetryPacket(packet);

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            double xpos = pos.x;
            double ypos = pos.y;
            heading = pos.h;
            //potential problem: taking more than one reading throughout the program

            normalHeading = heading;
            if (normalHeading != Math.abs(normalHeading)){//negatinve
                normalHeading = normalHeading + 2*3.14159265;
            }

            if (gamepad1.y) {
                myOtos.resetTracking();
            }

            if (gamepad1.x) {
                myOtos.calibrateImu();
            }

            double max;
            //double totalMovement = Math.sqrt(Math.pow(vxOutput, 2) + Math.pow(vyOutput, 2));
            //double axial = -vyOutput * Math.sin(normalHeading) + -vyOutput * Math.cos(normalHeading);;
            //double lateral = -vxOutput * Math.cos(normalHeading) - -vyOutput * Math.sin(normalHeading);

            double axial = -vyOutput;//it must be negative
            double lateral = -vxOutput;
//-1 *0 + 0 *

            //cos90 = 0
            //
            //so lateral is working but axial is not?


            lateral = /*+*/(-vxOutput) * Math.cos(normalHeading) + (-vyOutput) * Math.sin(normalHeading);//rotate counter clockwise or clockwise???//x
            axial = -(-vxOutput) * Math.sin(normalHeading) + /*+*/(-vyOutput) * Math.cos(normalHeading);//y

            double yaw = yawOutput;

            if(gamepad1.left_trigger > .1){
                yaw = 0;
            }

            if (gamepad1.right_trigger > .1){
                lateral = 0;
                axial = 0;
            }

          /*  if (totalMovement > 1.0) {
                double scalingFactor = 1.0 / totalMovement;
                axial *= scalingFactor;
                axial *= scalingFactor;
            }
*/
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

           /* pos = myOtos.getPosition();
            double deltaX = pos.x - oldx;
            double deltaY = pos.y - oldy;
            double newHeading = pos.h;
*//*
            if (newHeading != Math.abs(newHeading)){//negatinve
                newHeading = newHeading + 2*3.14159265;
            }*/

            //  normalHeading = (newHeading + oldheading) / 2;

            // Adjusting position calculation to maintain accuracy when heading changes drastically
            //double adjustedHeading = Math.abs(pos.h) > Math.PI ? pos.h - (Math.signum(pos.h) * 2 * Math.PI) : pos.h;

            /*double actualXchange = deltaX * Math.cos(normalHeading) - deltaY * Math.sin(normalHeading);
            double actualYchange = deltaX * Math.sin(normalHeading) + deltaY * Math.cos(normalHeading);*/

            //finalX = finalX + deltaX;
            // finalY = finalY + deltaY;
            finalX = xpos + originX;
            finalY = ypos + originY;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addData("X coordinate", finalX);
            telemetry.addData("Y coordinate", finalY);
            telemetry.addData("yawOutput", yawOutput);
            telemetry.addData("vxOut", vxOutput);
            telemetry.addData("vyOut", vyOutput);
            telemetry.addData("axial", axial);
            telemetry.addData("lateral", lateral);
            telemetry.addData("Heading radians", normalHeading);
            telemetry.addData("xspeed inches/sec", dxdt);
            telemetry.addData("yspeed inches/second", dydt);
            telemetry.addData("angular velocity radians/second", dhdt);
            telemetry.update();

            localXTarget = targetX * Math.cos(normalHeading) - targetY * Math.sin(normalHeading);//rotate counter clockwise or clockwise???
            localYTarget = -targetX * Math.sin(normalHeading) + targetY * Math.cos(normalHeading);//currently, clockwise

            yawOutput = PID.YawPID(pos.h, getRuntime(), Math.toRadians(yawTarget));
            vxOutput = PID.vxPID(finalX, getRuntime(), targetX);
            vyOutput = PID.vyPID(finalY, getRuntime(), targetY);


            //speed calculations
            changeTime = getRuntime() - lastTime;
            lastTime = getRuntime();

            SparkFunOTOS.Pose2D pos2 = myOtos.getPosition();
            double xpos2 = pos2.x;
            double ypos2 = pos2.y;
            double heading2 = pos2.h;

            changeY = pos2.y - lastY;
            lastY = pos2.y;

            changeX = pos2.x - lastX;
            lastX = pos2.x;

            changeH = pos2.h - lastH;
            lastH = pos2.h;

            dydt = changeY/changeTime;
            dxdt = changeX/changeTime;
            dhdt = changeH/changeTime;

        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1);
        myOtos.setAngularScalar(0.988319);

        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);


        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}