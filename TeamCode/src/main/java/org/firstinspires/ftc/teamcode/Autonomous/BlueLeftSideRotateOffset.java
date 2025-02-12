package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Action;
import org.firstinspires.ftc.teamcode.Hardware.ComputePid;
import org.firstinspires.ftc.teamcode.Hardware.Position2d;
import org.firstinspires.ftc.teamcode.Hardware.VelocityAccelertaionSparkFun;
import org.firstinspires.ftc.teamcode.Hardware.VxVyAxAy;
import org.firstinspires.ftc.teamcode.Hardware.currentDoHicky;
import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies;

import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetVerticalIdea;

import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetHorizontalIdea;


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
@Disabled
@Autonomous(name = "BlueLeftSideRotateOffset", group = "Autonomous", preselectTeleOp = "BlueTeleop")
public class BlueLeftSideRotateOffset extends LinearOpMode {



    targetVerticalIdea verticalTargetAuto = targetVerticalIdea.COLLECT_SPECIMIN;
    targetHorizontalIdea horizontalTargetAuto = targetHorizontalIdea.ZERO_HS_SLIDES;


    doCoolThingies doCoolThingies = new doCoolThingies();//rename?

    VelocityAccelertaionSparkFun vectorSystem = new VelocityAccelertaionSparkFun();

    ColorSensor intakeColor;
    TouchSensor hsTouch;
    TouchSensor vsTouch;

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

    private CRServo flick = null;


    public static double scale = 0.8; //I think this is a speed scaler?

    public boolean inTargetBox = false;

    public int actionNumber = 0;

    // Create an instance of the sensor

    SparkFunOTOS myOtos;


    Action act1 = new Action(new Position2d(-30,-55,Math.toRadians(-90)), 1, targetVerticalIdea.SQUEEZE_THE_CATCH, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act2 = new Action(new Position2d(-30, -55, Math.toRadians(-60)), 1, targetVerticalIdea.SAFE_RAISE, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act3 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 2, targetVerticalIdea.DEPOSIT_POTATO, targetHorizontalIdea.HOVER_ACROSS_BARIER, 0.8);
    Action act4 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.RELEASE, targetHorizontalIdea.HOVER_ACROSS_BARIER, 0.8);
    Action act5 = new Action(new Position2d(-25, -45, Math.toRadians(0)), 1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, 0.8);

    //needs not to continune until it senses
    Action act6 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.RELEASE, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act7 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.SNATCH_THAT_FISHY, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act8 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.SQUEEZE_THE_CATCH, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act9 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.SAFE_RAISE, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act10 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.DEPOSIT_POTATO, targetHorizontalIdea.READY_HS_POS, 0.8);
    Action act11 = new Action(new Position2d(-30, -55, Math.toRadians(-45)), 1, targetVerticalIdea.RELEASE, targetHorizontalIdea.READY_HS_POS, 0.8);


    int numberOfActs = 11;


    // Action act1 = new Action(new Pose2d(-50, -50, Math.toRadians(0)), 2);
   // Action act2 = new Action(new Pose2d(-50, 35, Math.toRadians(0)), 2);

    List<Action> actions = new ArrayList<>();

    double act1pause = 1;
    double act2pause = 1;
    double act3pause = 1;
    double act4pause = 1;
    double act5pause = 1;
    double act6pause = 1;


    List<Position2d> pauses = new ArrayList<>();


    public static double testttt = 1; // Example of FTC dashboard variable

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo servo = null;

    double finalX = 0;
    double finalY = 0;


    //ORIGIN OF BEGINNING

    double yawOrigin = -90;
    double originY = -64;// -61;
    double originX = -14;//-7;

    //Action returnHome = new Action(new Pose2d(originX, originY, Math.toRadians(0)), 1);


    double normalHeading = 0;
    double currentDriveX = 0;
    double currentDriveY = 0;

    public static double yawKp = 0.7, yawKi = 0.01, yawKd = 0.001;  // PIDgains

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


    public static double clawClose = 0.22;//previous .25
    public static double clawOpen = 0.5;


    currentDoHicky horizontalPositions = new currentDoHicky(0,0,0,0,0,0,0,0,false, false, false);
    currentDoHicky verticalPositions = new currentDoHicky(0,0,0,0,0,0,0,0, false, false, false);

    double vsTarget = 0;
    double hsTarget = 0;

    boolean killHorizontal = true;
    boolean killVertical = true;

    double vsOutput = 0;
    double hsOutput = 0;


    @Override
    public void runOpMode() throws InterruptedException {






        /*actions.add(act1);
        actions.add(act1);
        for (int i = 0; i < 20; i++) {
            actions.add(act1);
            actions.add(act2);
        }*/

        int takeNoteOFTHIS = numberOfActs;

        actions.add(act1);
        actions.add(act2);
        actions.add(act3);
        actions.add(act4);
        actions.add(act5);
        actions.add(act6);
        actions.add(act7);
        actions.add(act8);
        actions.add(act9);
        actions.add(act10);
        actions.add(act11);

        //actions.add(returnHome);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
// Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        intakeColor = hardwareMap.get(ColorSensor.class, "intakeColor");

        hsTouch = hardwareMap.get(TouchSensor.class, "hsTouch");
        vsTouch = hardwareMap.get(TouchSensor.class, "vsTouch");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");


        horizontalSlides = hardwareMap.get(DcMotor.class, "HS");
        verticalSlides = hardwareMap.get(DcMotor.class, "VS");

        verticalSlides.setDirection(DcMotorSimple.Direction.REVERSE);

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


        configureOtos();

        // Wait for the start button to be pressed
        waitForStart();
        resetRuntime();



        // Loop until the OpMode ends
        while (opModeIsActive()) {

            VxVyAxAy velocities = vectorSystem.getvelocity(getRuntime(), finalX, finalY, 0);


            double stopSpeed = 0.0001;
            double noPauseLeft;

            Action currentAction = actions.get(actionNumber); // Get the current action
            Position2d targetPose = currentAction.getPose(); // Get the Pose2d from the current action
            double waitTime = currentAction.getWaitTime(); // Get the wait time from the current action

            //chekcs to see if we are in target box and if speed is 0
            if (Math.abs(finalX - targetX) < 3 & velocities.getVx() < stopSpeed){//prevously, 3 was 1
                if (Math.abs(finalY - targetY) < 3 & velocities.getVy() < stopSpeed){
                    if (/*Math.abs(heading - yawTarget) < 0.0872665 & */velocities.getVh() < 0.001){
                        inTargetBox = true;
                        gamepad1.rumble(3);
                    }
                }
            }


            //GETTING VERTICAL AND HORIZONTAL
            verticalTargetAuto = currentAction.getVerticalTargetAuto();
            horizontalTargetAuto = currentAction.getHorizontalTargetAuto();

            verticalPositions = doCoolThingies.magicalVerticalMacro(verticalTargetAuto);
            horizontalPositions = doCoolThingies.magicalHorizontalMacro(horizontalTargetAuto, 1);


            hsTarget = horizontalPositions.getHSpos();

            if(!killHorizontal){
                liftL.setPosition(horizontalPositions.getintakeLiftPos());
                liftR.setPosition(horizontalPositions.getintakeLiftPos());
                intake.setPower(horizontalPositions.getIntakeSpeed());
            }

            vsTarget = verticalPositions.getVSpos();

            if(!killVertical){
                armL.setPosition(verticalPositions.getshoulderPos());
                armR.setPosition(verticalPositions.getshoulderPos());
                wristL.setPosition(verticalPositions.getwristLPos());
                wristR.setPosition(verticalPositions.getwristRPos());
                if(!verticalPositions.getClawState()){//false
                    claw.setPosition(clawOpen);
                }else{
                    claw.setPosition(clawClose);
                }
            }


            /*COLOR SENSING AUTOMATION

            currentColor = colorTest.color(intakeColor, Time);

            if(gamepad1.left_trigger > 0.1){
                intake.setPower(-gamepad1.left_trigger);

            }else{
                intake.setPower(gamepad1.right_trigger);
            }


            telemetry.addData("currentColor is", currentColor);
            double colorChangeDelayServo = 0.9;

            if(currentColor  == "yellow" || currentColor == "blue" && horizontalTarget == targetHorizontalIdea.FULL_EXTENT_DROP){
                horizontalTarget = targetHorizontalIdea.ZERO_HS_SLIDES;
            }


            if(currentColor == "red" || currentColor  == "yellow" || currentColor == "blue"){
                waitUntil = getRuntime() + colorChangeDelayServo;
            }
            if(getRuntime() < waitUntil){
                flick.setPower(-1);
                telemetry.addData("Servo power", -1);
            }else{
                //flick.setPower(0);
                flick.setPower(-gamepad1.right_trigger);
            }

            */



            //CONTINUE DRIVETRAIN AUTO
            //next we must complete the pause

            //once in targetBox, it starts the pause
            if(inTargetBox & !AlreadyPausing){
                endTime = getRuntime() + waitTime;
                AlreadyPausing = true;
            }

            if (inTargetBox & actionNumber < (actions.size() -1) & getRuntime() > endTime){
                AlreadyPausing = false;
                actionNumber += 1;
                inTargetBox = false;
            }


            targetX = targetPose.getThisX();
            targetY = targetPose.getThisY();
            yawTarget = Math.toDegrees(targetPose.getThisHeading());//I think
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

            //testing new feature
            double unOriginedxpos = pos.x;
            double unOriginedypos = pos.y;

            double xpos = unOriginedxpos * Math.cos(Math.toRadians(yawOrigin)) - unOriginedypos * Math.sin(Math.toRadians(yawOrigin));
            double ypos = unOriginedxpos * Math.sin(Math.toRadians(yawOrigin)) + unOriginedypos * Math.sin(Math.toRadians(yawOrigin));
            heading = pos.h + Math.toRadians(yawOrigin);
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

            if (!gamepad1.a){
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }else{
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }

            //MORE SLIDES STUFFS
            double hspos = -horizontalSlides.getCurrentPosition();
            double vspos = verticalSlides.getCurrentPosition();

            if(!killVertical){
                verticalSlides.setPower(vsOutput);
            }else{
                verticalSlides.setPower(0);
            }
            if(!killHorizontal){
                horizontalSlides.setPower(-hsOutput);//bc was negativde when usi9g the gamepad input
            }else{
                horizontalSlides.setPower(0);
            }

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
            //see below for the use of originYaw

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addData("X coordinate", finalX);
            telemetry.addData("Y coordinate", finalY);
            telemetry.addData("X coordinate", targetX);
            telemetry.addData("Y coordinate", targetY);

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
            telemetry.update();

            localXTarget = targetX * Math.cos(normalHeading) - targetY * Math.sin(normalHeading);//rotate counter clockwise or clockwise???
            localYTarget = -targetX * Math.sin(normalHeading) + targetY * Math.cos(normalHeading);//currently, clockwise

            yawOutput = PID.YawPID(/*pos.h*/pos.h + Math.toRadians(yawOrigin), getRuntime(), Math.toRadians(yawTarget));
            vxOutput = PID.vxPID(finalX, getRuntime(), targetX);
            vyOutput = PID.vyPID(finalY, getRuntime(), targetY);


            yawOutput = yawOutput * scale;
            vxOutput = vxOutput * scale;
            vyOutput = vyOutput * scale;

            vsOutput = PID.vsPID(vspos, getRuntime(), vsTarget);
            hsOutput = PID.hsPID(hspos, getRuntime(), hsTarget);

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