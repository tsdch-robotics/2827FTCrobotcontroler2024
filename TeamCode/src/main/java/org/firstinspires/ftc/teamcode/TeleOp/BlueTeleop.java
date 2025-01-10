package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ComputePid;
import org.firstinspires.ftc.teamcode.Hardware.VelocityAccelertaionSparkFun;
import org.firstinspires.ftc.teamcode.Hardware.VxVyAxAy;
import org.firstinspires.ftc.teamcode.Hardware.currentDoHicky;
import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies;
import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetIdea;

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
@TeleOp(name = "BlueTeleop", group = "Sensor")
public class BlueTeleop extends LinearOpMode {

    doCoolThingies doCoolThingies = new doCoolThingies();//rename?
    currentDoHicky hearMeOutLetsDoThis = new currentDoHicky(0,0,0,0,0,0,false);//the init params
    targetIdea myEpicTarget = targetIdea.INIT;

    VelocityAccelertaionSparkFun vectorSystem = new VelocityAccelertaionSparkFun();

    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    ColorSensor intakeColor;
    String currentColor = "none";
    double waitUntil = 0;

    public static double test1 = .2; // Example of FTC dashboard variable

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

    private Servo wristL = null;
    private Servo wristR = null;
    private Servo claw = null;


    private Servo liftL = null;
    private Servo liftR = null;
    boolean raising = false;

    private CRServo flick = null;



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

    public static double clawTest = 0.5;

    boolean Ydelay = false;

    boolean Bdelay = false;

    @Override
    public void runOpMode() throws InterruptedException {

        hearMeOutLetsDoThis = doCoolThingies.magicalMacro(horizontalSlides,verticalSlides, myEpicTarget);//testing our ability to update

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        intakeColor = hardwareMap.get(ColorSensor.class, "intakeColor");

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

        flick = hardwareMap.get(CRServo.class, "flick");//Continuous rotation servo

        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");

        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");

        claw = hardwareMap.get(Servo.class, "claw");


/*
        private Servo wristL = null;
        private Servo wristR = null;
        private Servo claw = null;
*/


        armL.setDirection(Servo.Direction.REVERSE);
        armR.setDirection(Servo.Direction.FORWARD);

        wristL.setDirection(Servo.Direction.REVERSE);
        wristR.setDirection(Servo.Direction.FORWARD);

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

            telemetry.addData("STATE", myEpicTarget);
/*
            if(gamepad1.dpad_right){
                armL.setPosition(0);
            }else{
                armL.setPosition(1);
            }

            if(gamepad1.dpad_left){
                armR.setPosition(0);
            }else{
                armR.setPosition(1);
            }*/


            //button a ->
            //button y -> deposit potato

            //official controls buttons
            /*if(gamepad1.y){
                myEpicTarget = targetIdea.DEPOSIT_POTATO;
            }*/

            if(gamepad1.y && myEpicTarget != targetIdea.DEPOSIT_POTATO && myEpicTarget != targetIdea.STALKER &&!Ydelay){
                myEpicTarget = targetIdea.DEPOSIT_POTATO;
                Ydelay = true;
            }else if(gamepad1.y && myEpicTarget != targetIdea.STALKER && !Ydelay){
                myEpicTarget = targetIdea.STALKER;
                Ydelay = true;
            }else if(gamepad1.y && myEpicTarget != targetIdea.SNATCH_THAT_FISHY && !Ydelay){
                myEpicTarget = targetIdea.SNATCH_THAT_FISHY;
                Ydelay = true;
            } else if (!gamepad1.y){
                Ydelay = false;
            }


            if(gamepad1.a){
                myEpicTarget = targetIdea.COLLECT_SPECIMIN;
            }

            if(gamepad1.b && myEpicTarget != targetIdea.PRE_SCORE_SPECIMEN && !Bdelay){
                myEpicTarget = targetIdea.PRE_SCORE_SPECIMEN;
                Bdelay = true;
            }else if (gamepad1.b && myEpicTarget != targetIdea.SCORE_SPECIMEN && !Bdelay){
                myEpicTarget = targetIdea.SCORE_SPECIMEN;
                Bdelay = true;
            }else if (!gamepad1.b){
                Bdelay = false;
            }


            hearMeOutLetsDoThis = doCoolThingies.magicalMacro(horizontalSlides,verticalSlides, myEpicTarget);

            //hsTarget = hearMeOutLetsDoThis.getHSpos();
            vsTarget = hearMeOutLetsDoThis.getVSpos();
            armL.setPosition(hearMeOutLetsDoThis.getshoulderPos());
            armR.setPosition(hearMeOutLetsDoThis.getshoulderPos());
            wristL.setPosition(hearMeOutLetsDoThis.getwristLPos());
            wristR.setPosition(hearMeOutLetsDoThis.getwristRPos());

            if(gamepad1.x){
                hsTarget = 1500;
            }


            if(gamepad1.dpad_down){
                claw.setPosition(.3);
            }

            if(gamepad1.dpad_up){
                claw.setPosition(.6);
            }



            int colorGreen = intakeColor.green();
            int colorRed = intakeColor.red();
            int colorBlue = intakeColor.blue();

            //rgb I think
            //telemetry.addData("colorRed", colorRed);
            //telemetry.addData("colorGreen", colorGreen);
            //telemetry.addData("colorBlue", colorBlue);

            if (colorRed > 600){
                if(colorGreen > 600){
                    currentColor = "yellow";
                }
                else{
                    currentColor = "red";
                }
            }else if(colorBlue > 600){
                currentColor = "blue";
            }else{
                currentColor = "none";
            }

            telemetry.addData("currentColor is", currentColor);
            double colorChangeDelayServo = 0.9;

            if(gamepad1.right_trigger > 0.1){
                intake.setPower(-gamepad1.right_trigger);

            }else{
                intake.setPower(gamepad1.left_trigger);
            }


            if(gamepad1.left_bumper || currentColor == "red"){
                liftL.setPosition(test1);//down
                liftR.setPosition(test1);
                raising = false;

            }

            if(gamepad1.right_bumper || currentColor  == "yellow" || currentColor == "blue"){
                liftL.setPosition(0);
                liftR.setPosition(0);
                raising = true;
                hsTarget = 50;//eventually remove
                //hsTarget = 50;
            }else{
                raising = false;
            }

            if(currentColor == "red" || raising){
                waitUntil = getRuntime() + colorChangeDelayServo;
            }
            if(gamepad2.y || getRuntime() < waitUntil){
                flick.setPower(-1);
            }else if (gamepad1.right_trigger > 0.1){
                raising = false;//unecessary?
                flick.setPower(1);
            }else{
                raising = false;
                flick.setPower(0);
            }


            /*if(gamepad2.x){
                flick.setPower(1);
                armL.setPosition(1);
                armR.setPosition(1);
                hookL.setPosition(1);
                hookR.setPosition(1);
                ptoL.setPosition(1);
                ptoR.setPosition(1);
                liftL.setPosition(1);
                liftR.setPosition(1);
            }
            if(gamepad2.b){
                flick.setPower(0);
                armL.setPosition(0);
                armR.setPosition(0);
                hookL.setPosition(0);
                hookR.setPosition(0);
                ptoL.setPosition(0);
                ptoR.setPosition(0);
                liftL.setPosition(0);
                liftR.setPosition(0);
            }*/


            /*if(gamepad2.y){
                liftL.setPosition(1);
                liftR.setPosition(1);
            }

            if(gamepad2.x){
                liftL.setPosition(0);
                liftR.setPosition(0);
            }*/

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