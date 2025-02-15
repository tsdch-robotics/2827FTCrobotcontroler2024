package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Hardware.GetWheeledLocalization;
import org.firstinspires.ftc.teamcode.Hardware.Position2d;
import org.firstinspires.ftc.teamcode.Hardware.VelocityAccelertaionSparkFun;
import org.firstinspires.ftc.teamcode.Hardware.VxVyAxAy;
import org.firstinspires.ftc.teamcode.Hardware.currentDoHicky;
import org.firstinspires.ftc.teamcode.Hardware.determineColor;
import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies;

import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetVerticalIdea;

import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetHorizontalIdea;


import java.util.List;
import java.util.ArrayList;

@Config
@Autonomous(name = "AllLeft", group = "Autonomous", preselectTeleOp = "BlueTeleop")
public class AllLeft extends LinearOpMode {

    AreWeThereYet AreWeThereYet = new AreWeThereYet();


    targetVerticalIdea verticalTargetAuto = targetVerticalIdea.INIT;
    targetHorizontalIdea horizontalTargetAuto = targetHorizontalIdea.ZERO_HS_SLIDES;

    GetWheeledLocalization getWheeledLocalization = new GetWheeledLocalization();

    doCoolThingies doCoolThingies = new doCoolThingies();//rename?

    VelocityAccelertaionSparkFun vectorSystem = new VelocityAccelertaionSparkFun();

    ColorSensor intakeColor;

    String currentColor = "none";
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

    double maxSpeed = 1;
    // Create an instance of the sensor

    public static double cappedSpeed = 1;

    //gohere
    //gethere

    SparkFunOTOS myOtos;
    //use mr hicks robt squaring specimin advice
    Action act1 = new Action(new Position2d(-50,-57,Math.toRadians(-45)), -3, targetVerticalIdea.SAFE_RAISE, targetHorizontalIdea.ZERO_HS_SLIDES, cappedSpeed);
    Action meetThebasket = new Action(new Position2d(-54,-60,Math.toRadians(-45)), 1, targetVerticalIdea.DEPOSIT_POTATO, targetHorizontalIdea.ZERO_HS_SLIDES, cappedSpeed);
    Action dropSample = new Action(new Position2d(-54, -60, Math.toRadians(-45)), 1, targetVerticalIdea.RELEASE, targetHorizontalIdea.HOVER_ACROSS_BARIER, cappedSpeed);

    //List<Action> collectRight = makeFigure8(-45, -56, 0);


    Action collectSampleRight = new Action(new Position2d(/*do not mess*/-45, -54, Math.toRadians(5)), 2, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, 0.7);


    Action bringBack = new Action(new Position2d(-54, -55, Math.toRadians(-45)), 2, targetVerticalIdea.STALKER/*add the drop it aspect*/, targetHorizontalIdea.ZERO_HS_SLIDES_FLICK_ON, cappedSpeed);
    Action grabIt = new Action(new Position2d(-54, -55, Math.toRadians(-45)), 1, targetVerticalIdea.SNATCH_THAT_FISHY, targetHorizontalIdea.READY_HS_POS_FLICK_STILL_ON, cappedSpeed);
    Action squeeze = new Action(new Position2d(-54, -55, Math.toRadians(-45)), 1, targetVerticalIdea.SQUEEZE_THE_CATCH, targetHorizontalIdea.ZERO_HS_SLIDES, cappedSpeed);
    Action safeRaise = new Action(new Position2d(-54, -55, Math.toRadians(-45)), 1, targetVerticalIdea.SAFE_RAISE, targetHorizontalIdea.READY_HS_POS, cappedSpeed);

    Action deposit = new Action(new Position2d(-54, -57, Math.toRadians(-45)), 1, targetVerticalIdea.DEPOSIT_POTATO_AUTO, targetHorizontalIdea.HOVER_ACROSS_BARIER, cappedSpeed);

    //instead add another dropSample and go

    Action collectSampleMid = new Action(new Position2d(-47, -55, Math.toRadians(20)), 1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);

    Action collectSampleLeft = new Action(new Position2d(-47, -57, Math.toRadians(30)), 2, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);

    Action park = new Action(new Position2d(-45, -18, Math.toRadians(-45)), 1, targetVerticalIdea.PARK, targetHorizontalIdea.ZERO_HS_SLIDES, cappedSpeed);
    Action parkb = new Action(new Position2d(-25, -18, Math.toRadians(-90)), 2, targetVerticalIdea.PARK, targetHorizontalIdea.READY_HS_POS, cappedSpeed);


    //Action act8 = new Action(new Position2d(-45, -45, Math.toRadians(-45)), 1, targetVerticalIdea.RELEASE, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE);


    int numberOfActs = 11;



    double pivLink = 30;//inches

    public List<Action> makeFigure8 (double figureX, double figureY, double figureA){

        Action fig1 = new Action(new Position2d(figureX, figureY, Math.toRadians(figureA)), -1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP, cappedSpeed);

        double fig2Ang = -20;
        double fig2Xbase = figureX + pivLink * Math.sin(-fig2Ang);
        double fig2Ybase = figureY + pivLink * Math.cos(-fig2Ang);

        Action fig2 = new Action(new Position2d(fig2Xbase - 1, fig2Ybase + 1, Math.toRadians(figureA + fig2Ang)),
                -1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);

        Action fig3 = new Action(new Position2d(fig2Xbase + 2, fig2Xbase + 2, Math.toRadians(figureA + fig2Ang)),
                1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);

        double fig4Ang = 20;
        double fig4Xbase = figureX + pivLink * Math.sin(fig4Ang);
        double fig4Ybase = figureY + pivLink * Math.cos(fig4Ang);

        Action fig4 = new Action(new Position2d(fig4Xbase + 2, fig4Xbase + 2, Math.toRadians(figureA + fig4Ang)),
                1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);


        Action fig5 = new Action(new Position2d(fig4Xbase -4, fig4Xbase + 6, Math.toRadians(figureA + fig4Ang)),
                1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);


        List<Action> figureActions = new ArrayList<>();

        figureActions.add(fig1);
        figureActions.add(fig2);
        figureActions.add(fig3);
        figureActions.add(fig4);
        figureActions.add(fig5);

        return(figureActions);

    }



    Action figure8 = new Action(new Position2d(-47, -53, Math.toRadians(15)), 1, targetVerticalIdea.STALKER, targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE, cappedSpeed);




    // Action act1 = new Action(new Pose2d(-50, -50, Math.toRadians(0)), 2);
    // Action act2 = new Action(new Pose2d(-50, 35, Math.toRadians(0)), 2);


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

    double yawOrigin = 0;
    double originY = -64;// -61;
    double originX = -37.5;//place on right side of tile

    //Action returnHome = new Action(new Pose2d(originX, originY, Math.toRadians(0)), 1);


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


    public static double clawClose = 0;//previous .25
    public static double clawOpen = 0.5;


    currentDoHicky horizontalPositions = new currentDoHicky(0,0,0,0,0,0,0,0,false, false, false);
    currentDoHicky verticalPositions = new currentDoHicky(0,0,0,0,0,0,0,0, false, false, false);

    double vsTarget = 0;
    double hsTarget = 0;

    boolean killHorizontal = false;
    boolean killVertical = false;

    double vsOutput = 0;
    double hsOutput = 0;


    boolean noMoreVSzero = false;
    boolean noMoreHSzero = false;

    boolean promptedToContinueNow = false;//to speed things up when needed


    double posX = 0;
    double posY = 0;
    double posH = 0;

    boolean bigBoolean = false;
    double bigBooleanRadius = 0;

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
        actions.add(meetThebasket);
        actions.add(dropSample);


        actions.add(collectSampleRight);

        actions.add(bringBack);
        actions.add(grabIt);
        actions.add(squeeze);

        actions.add(safeRaise);

        actions.add(meetThebasket);
        actions.add(dropSample);

//MID
        actions.add(collectSampleMid);
        actions.add(bringBack);
        actions.add(grabIt);
        actions.add(squeeze);


        actions.add(safeRaise);


        actions.add(deposit);
        actions.add(dropSample);
        //END MID
/*
        actions.add(collectSampleLeft);
        actions.add(bringBack);
        actions.add(grabIt);
        actions.add(squeeze);

        actions.add(safeRaise);

        actions.add(deposit);
        actions.add(dropSample);*/

        actions.add(park);
        actions.add(parkb);





        //actions.add(returnHome);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
// Get a reference to the sensor
        determineColor colorTest = new determineColor();

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


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        configureOtos();

        // Wait for the start button to be pressed



        //so that it can hold in init
        verticalPositions = doCoolThingies.magicalVerticalMacro(verticalTargetAuto);

        armL.setPosition(verticalPositions.getshoulderPos());
        armR.setPosition(verticalPositions.getshoulderPos());
        wristL.setPosition(verticalPositions.getwristLPos());
        wristR.setPosition(verticalPositions.getwristRPos());
        if(!verticalPositions.getClawState()){//false
            claw.setPosition(clawOpen);
        }else{
            claw.setPosition(clawClose);
        }

        telemetry.addData("ACTION NUMBER", actionNumber);

        waitForStart();
        resetRuntime();


        // Loop until the OpMode ends
        while (opModeIsActive()) {

            Position2d odoPos = getWheeledLocalization.wheeledLocalization(leftFrontDrive/*en1,left*/,
                    rightBackDrive/*en2,right*/, rightFrontDrive/*enc3, bakc*/, posX, posY, posH);
            posX = odoPos.getThisX();
            posY = odoPos.getThisY();
            posH = odoPos.getThisHeading();

            telemetry.addData("posX", posX);

            telemetry.addData("posY", posY);

            telemetry.addData("posH", posH);

            telemetry.addData("en1", leftFrontDrive.getCurrentPosition());
            telemetry.addData("en2", rightBackDrive.getCurrentPosition());
            telemetry.addData("en3", rightFrontDrive.getCurrentPosition());

            telemetry.addData("ACTION NUMBER", actionNumber);



            double Time = runtime.time();

            VxVyAxAy velocities = vectorSystem.getvelocity(getRuntime(), finalX, finalY, posH);

            //double stopSpeed = 0.0001;
            double stopSpeed = 0.01;
            double noPauseLeft;

            Action currentAction = actions.get(actionNumber); // Get the current action
            Position2d targetPose = currentAction.getPose(); // Get the Pose2d from the current action
            double waitTime = currentAction.getWaitTime(); // Get the wait time from the current action

            targetX = targetPose.getThisX();
            targetY = targetPose.getThisY();
            yawTarget = Math.toDegrees(targetPose.getThisHeading());
            telemetry.addData("yawTarget", yawTarget);

            //ARE WE THERE YET?
            double errorX = finalX - targetX;
            double errorY = finalY - targetY;
            double errorH = posH - yawTarget;


            inTargetBox = AreWeThereYet.weThere(errorX, errorY, errorH, velocities.getVx(), velocities.getVy(), velocities.getVh(), currentAction.getWaitTime());//the wait time is useed to determine continuity


            //GETTING VERTICAL AND HORIZONTAL
            if(!noMoreVSzero){
                verticalTargetAuto = currentAction.getVerticalTargetAuto();
            }

            if(!noMoreHSzero){
                horizontalTargetAuto = currentAction.getHorizontalTargetAuto();
            }

            maxSpeed = currentAction.getMaxSpeed();


            //AUTO ZERO
            if(verticalTargetAuto == targetVerticalIdea.ZERO_VS_SLIDES && vsTouch.isPressed()){
                verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalTargetAuto = targetVerticalIdea.READY_VS_POS;
                noMoreVSzero = true;
            }else if (verticalTargetAuto != targetVerticalIdea.ZERO_VS_SLIDES){
                noMoreVSzero = false;
            }else{
                verticalTargetAuto = targetVerticalIdea.READY_VS_POS;
            }

            if(horizontalTargetAuto == targetHorizontalIdea.ZERO_HS_SLIDES && hsTouch.isPressed()){
                horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontalTargetAuto = targetHorizontalIdea.READY_HS_POS_FLICK_STILL_ON;
                noMoreHSzero = true;
            }else if (horizontalTargetAuto != targetHorizontalIdea.ZERO_HS_SLIDES){
                noMoreHSzero = false;
            }else{
                horizontalTargetAuto = targetHorizontalIdea.READY_HS_POS_FLICK_STILL_ON;
            }


            //SET POSITIONS ACCORDINGLY
            verticalPositions = doCoolThingies.magicalVerticalMacro(verticalTargetAuto);
            horizontalPositions = doCoolThingies.magicalHorizontalMacro(horizontalTargetAuto, 1);


            hsTarget = horizontalPositions.getHSpos();

            if(!killHorizontal){
                liftL.setPosition(horizontalPositions.getintakeLiftPos());
                liftR.setPosition(horizontalPositions.getintakeLiftPos());
                intake.setPower(horizontalPositions.getIntakeSpeed());
                flick.setPower(horizontalPositions.getFlickSpeed());
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


            //COLOR SENSING AUTOMATION
            currentColor = colorTest.color(intakeColor, Time);

            telemetry.addData("currentColor is", currentColor);
            double colorChangeDelayServo = 0.9;

            if(currentColor  == "yellow" || currentColor == "blue" && horizontalTargetAuto == targetHorizontalIdea.FULL_EXTENT_DROP_WITH_INTAKE){
                promptedToContinueNow = true;
            }

            /*if(currentColor == "red" || currentColor  == "yellow" || currentColor == "blue"){
                waitUntil = getRuntime() + colorChangeDelayServo;
            }
            if(getRuntime() < waitUntil){
                flick.setPower(-1);
                telemetry.addData("Servo power", -1);
            }else{
                //flick.setPower(0);
                flick.setPower(-gamepad1.right_trigger);
            }*/



            //CONTINUE DRIVETRAIN AUTO
            //next we must complete the pause

            //once in targetBox, it starts the pause
            if(inTargetBox & !AlreadyPausing){
                endTime = getRuntime() + waitTime;
                AlreadyPausing = true;
                bigBoolean = true;
            }

            //GO ONTO THE NEXT ACTION PROCESS
            if ((inTargetBox & actionNumber < (actions.size() -1) & getRuntime() > endTime)/* || promptedToContinueNow*/){
                AlreadyPausing = false;
                actionNumber += 1;
                inTargetBox = false;
                promptedToContinueNow = false;
                bigBoolean = false;
            }

/*//moveed up so it changes in time!!

            targetX = targetPose.getThisX();
            targetY = targetPose.getThisY();
            yawTarget = Math.toDegrees(targetPose.getThisHeading());
            telemetry.addData("yawTarget", yawTarget);

            errorX = finalX - targetX;
            errorY = finalY - targetY;
            errorH = posH - yawTarget;
*/


            double TelemX = -finalX;
            double TelemY = -finalY;//bc for some reason this works

            double adjH = normalHeading - (3.14159265/2);
            double[] xcordHead = {TelemX - 3, TelemX + 3, TelemX + 15 * Math.cos(normalHeading), TelemX + 15 * Math.cos(normalHeading)};
            double[] ycordHead = {TelemY, TelemY, TelemY + 15 * Math.sin(normalHeading), TelemY + 15 * Math.sin(normalHeading)};

            if(bigBoolean){
                bigBooleanRadius = 20;
            }else{
                bigBooleanRadius = 0;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("cyan")
                    .fillCircle(TelemX, TelemY, 6)
                    .fillCircle(TelemX + 15 * Math.cos(adjH), TelemY + 15 * Math.sin(adjH), 5)
                    .setFill("red")
                    .fillCircle(-targetX, -targetY, 5)
                    .setFill("purple")
                    .fillCircle(0, 0, bigBooleanRadius);

            //.setFill("green")//to show power outputs
            //.fillCircle(TelemX + 15 * Math.cos(adjH), TelemY + 15 * Math.sin(adjH), 2);

            dashboard.sendTelemetryPacket(packet);

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            /*double xpos = pos.x;
            double ypos = pos.y;
            */
            double xpos = posX;
            double ypos = posY;
            heading = pos.h + yawOrigin;
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
            lateral = lateral * 1.5;
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

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            //telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addData("X coordinate", finalX);
            telemetry.addData("Y coordinate", finalY);
            telemetry.addData("Heading radians", normalHeading);
            telemetry.addLine();
            telemetry.addData("target x", targetX);
            telemetry.addData("target Y", targetY);
            telemetry.addLine();
            telemetry.addLine("POWER OUTPUTS");
            telemetry.addData("yawOutput", yawOutput);
            telemetry.addData("vxOut", vxOutput);
            telemetry.addData("vyOut", vyOutput);
            // telemetry.addData("axial", axial);
            // telemetry.addData("lateral", lateral);
            telemetry.addLine("VELOCITYS");
            telemetry.addData("xspeed inches/sec", velocities.getVx());
            telemetry.addData("yspeed inches/second", velocities.getVy());
            telemetry.addData("angular velocity radians/second", velocities.getVh());


            telemetry.update();

            localXTarget = targetX * Math.cos(normalHeading) - targetY * Math.sin(normalHeading);//rotate counter clockwise or clockwise???
            localYTarget = -targetX * Math.sin(normalHeading) + targetY * Math.cos(normalHeading);//currently, clockwise

            yawOutput = PID.YawPID(/*pos.h*/pos.h + yawOrigin, getRuntime(), Math.toRadians(yawTarget));
            vxOutput = PID.vxPID(finalX, getRuntime(), targetX);
            vyOutput = PID.vyPID(finalY, getRuntime(), targetY);


            yawOutput = yawOutput * maxSpeed;
            vxOutput = vxOutput * maxSpeed;
            vyOutput = vyOutput * maxSpeed;

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