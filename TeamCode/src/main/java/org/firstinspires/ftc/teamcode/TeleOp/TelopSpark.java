package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ComputePid;
import org.firstinspires.ftc.teamcode.Hardware.VelocityAccelertaionSparkFun;
import org.firstinspires.ftc.teamcode.Hardware.VxVyAxAy;
import org.firstinspires.ftc.teamcode.R;

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
@TeleOp(name = "TeleopSpark", group = "Sensor")
public class TelopSpark extends LinearOpMode {

    VelocityAccelertaionSparkFun vectorSystem = new VelocityAccelertaionSparkFun();

    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    public static double test1 = .55; // Example of FTC dashboard variable

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor horizontalSlides = null;
    private DcMotor verticalSlides = null;


    private DcMotor intake = null;


    private Servo ptoL = null;
    private Servo ptoR = null;
    private Servo hookL = null;
    private Servo hookR = null;

    private Servo armL = null;
    private Servo armR = null;


    private Servo liftL = null;
    private Servo liftR = null;



    double finalX = 0;
    double finalY = 0;
    double originY = -50;
    double originX = -50;

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

    ComputePid PID = new ComputePid();


    boolean hanging = false;

    double vsPower = 0;
    double hsPower = 0;

    double yaw = 0;


    double hsOutput = 0;
    public static double hsTarget = 50;

    double vsOutput = 0;
    public static double vsTarget = 50;

    @Override
    public void runOpMode() throws InterruptedException {



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");


        horizontalSlides = hardwareMap.get(DcMotor.class, "HS");
        verticalSlides = hardwareMap.get(DcMotor.class, "VS");


        intake = hardwareMap.get(DcMotor.class, "intake");



        ptoL = hardwareMap.get(Servo.class, "ptoL");
        ptoR = hardwareMap.get(Servo.class, "ptoR");
        hookL = hardwareMap.get(Servo.class, "hookL");
        hookR = hardwareMap.get(Servo.class, "hookR");



        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");

        armL.setDirection(Servo.Direction.REVERSE);
        armR.setDirection(Servo.Direction.FORWARD);



        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");

        liftL.setDirection(Servo.Direction.REVERSE);
        liftR.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//important
        verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//(we can still take the reading though)

        ptoL.setPosition(0.48);
        ptoR.setPosition(0.48);




        configureOtos();

        // Wait for the start button to be pressed
        waitForStart();
        resetRuntime();

        // Loop until the OpMode ends
        while (opModeIsActive()) {


            intake.setPower(gamepad1.left_trigger);

            /*if(gamepad1.a){
                ptoL.setPosition(0.48);
                ptoR.setPosition(0.48);
                hanging = false;
            }

            if(gamepad1.b){
                ptoL.setPosition(0.59);
                ptoR.setPosition(0.61);
                hanging = true;
            }*/

            if(gamepad1.b){
                hsTarget = 1700;
            }

            if(gamepad1.a){
                hsTarget = 0;
            }

            if(gamepad1.left_bumper){
                hsTarget = 800;
            }

            if(gamepad1.right_bumper){
                vsTarget = 600;
            }

            if (gamepad1.y){
                vsTarget = 1000;
            }

            if (gamepad1.x){
                vsTarget = 0;
            }




            if(gamepad2.b){
                armL.setPosition(test1);
                armR.setPosition(test1);
            }

            if(gamepad2.a){
                armL.setPosition(0);
                armR.setPosition(0);
            }



            if(gamepad2.y){
                liftL.setPosition(1);
                liftR.setPosition(1);
            }

            if(gamepad2.x){
                liftL.setPosition(0);
                liftR.setPosition(0);
            }

/*
            double signSlides = 1;
            if (gamepad1.left_bumper){
                signSlides = -1;
            }*/

            VxVyAxAy velocities = vectorSystem.getvelocity(getRuntime(), myOtos);

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
            double heading = pos.h;
            //potential problem: taking more than one reading throughout the program

            normalHeading = heading;
            if (normalHeading != Math.abs(normalHeading)){//negatinve
                normalHeading = normalHeading + 2*3.14159265;
            }
//POSSIBLY KEEP INCASE A RESET IS NEEDED

            /*if (gamepad1.y) {
                myOtos.resetTracking();
            }

            if (gamepad1.x) {
                myOtos.calibrateImu();
            }*/

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

// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.


            if(!hanging){
                axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                lateral =  -gamepad1.left_stick_x;
                yaw     =  -gamepad1.right_stick_x;
            }else{
                axial = vsPower;
                lateral = 0;
                yaw     = 0;
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

            double hspos = -horizontalSlides.getCurrentPosition();
            double vspos = verticalSlides.getCurrentPosition();

            //horizontalSlides.setPower(-gamepad1.left_trigger * signSlides);


            /*
            hsPower = -gamepad1.left_trigger * signSlides;
            vsPower = gamepad1.right_trigger * signSlides;


            if (hspos < 1700 & hsPower < 0){
                horizontalSlides.setPower(hsPower);
            }
            else if (hspos > 40 & hsPower > 0){
                horizontalSlides.setPower(hsPower);
            }else{
                horizontalSlides.setPower(0);
            }*/

            verticalSlides.setPower(vsOutput);
            horizontalSlides.setPower(-hsOutput);//bc was negativde when usi9g the gamepad input


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


            telemetry.addData("xspeed inches/sec", velocities.getVx());
            telemetry.addData("yspeed inches/second", velocities.getVy());
            telemetry.addData("angular velocity radians/second", velocities.getVh());

            telemetry.addData("xacceleration inches/sec2", velocities.getAx());
            telemetry.addData("yacceleration inches/second2", velocities.getAy());
            telemetry.addData("angular acceleration radians/second2", velocities.getAh());

            telemetry.addData("hs pos", hspos);
            telemetry.addData("vs pos", vspos);


            telemetry.update();

            localXTarget = targetX * Math.cos(normalHeading) - targetY * Math.sin(normalHeading);//rotate counter clockwise or clockwise???
            localYTarget = -targetX * Math.sin(normalHeading) + targetY * Math.cos(normalHeading);//currently, clockwise

            yawOutput = PID.YawPID(pos.h, getRuntime(), Math.toRadians(yawTarget));
            vxOutput = PID.vxPID(finalX, getRuntime(), targetX);
            vyOutput = PID.vyPID(finalY, getRuntime(), targetY);

            hsOutput = PID.hsPID(hspos, getRuntime(), hsTarget);
            vsOutput = PID.vsPID(vspos, getRuntime(), vsTarget);

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