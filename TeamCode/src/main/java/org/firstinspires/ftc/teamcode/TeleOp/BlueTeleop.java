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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ComputePid;
import org.firstinspires.ftc.teamcode.Hardware.Position2d;
import org.firstinspires.ftc.teamcode.Hardware.VelocityAccelertaionSparkFun;
import org.firstinspires.ftc.teamcode.Hardware.VxVyAxAy;
import org.firstinspires.ftc.teamcode.Hardware.currentDoHicky;
import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies;
import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetVerticalIdea;

import org.firstinspires.ftc.teamcode.Hardware.doCoolThingies.targetHorizontalIdea;
import org.firstinspires.ftc.teamcode.Hardware.determineColor;

import org.firstinspires.ftc.teamcode.Hardware.GetWheeledLocalization;


@Config
@TeleOp(name = "BlueTeleop", group = "Sensor")
public class BlueTeleop extends LinearOpMode {

    doCoolThingies doCoolThingies = new doCoolThingies();//rename?

    GetWheeledLocalization getWheeledLocalization = new GetWheeledLocalization();

    //currentDoHicky hearMeOutLetsDoThis = new currentDoHicky(0,0,0,0,0,0,false);//the init params
    currentDoHicky horizontalPositions = new currentDoHicky(0,0,0,0,0,0,0,0,false, false, false);
    currentDoHicky verticalPositions = new currentDoHicky(0,0,0,0,0,0,0,0,false, false, false);

    targetVerticalIdea verticalTarget = targetVerticalIdea.COLLECT_SPECIMIN;
    targetHorizontalIdea horizontalTarget = targetHorizontalIdea.ZERO_HS_SLIDES;

    targetVerticalIdea nextVerticalTarget = targetVerticalIdea.STALKER;
    targetHorizontalIdea nextHorizontalTarget = targetHorizontalIdea.READY_HS_POS;

    VelocityAccelertaionSparkFun vectorSystem = new VelocityAccelertaionSparkFun();

    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    ColorSensor intakeColor;
    TouchSensor hsTouch;
    TouchSensor vsTouch;

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

    boolean driveHange = false;

    public static double engageL = 0.6;
    public static double engageR = 0.8;


    public static double disEngageL = 0.5;
    public static double disEngageR = 0.5;


    public static double hookLhook = 0.6;
    public static double hookRhook = 0.6;



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

    double flickPower = 0;


    double finalX = 0;
    double finalY = 0;


    double yawOrigin = 0;
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
    public static double hsTarget = 0;//that way it will imediently 0

    double vsOutput = 0;
    public static double vsTarget = 50;

    public static double clawTest = 0.5;

    boolean Ydelay = false;

    boolean Bdelay = false;

    public static Boolean killVertical = false;
    public static Boolean killHorizontal = false;

    boolean alreadyPressingY = false;
    boolean alreadyPressingX = false;
    boolean alreadyPressingX2= false;
    boolean alreadyPressingB = false;
    boolean alreadyPressingBack = false;

    boolean cancelColor = false;


    boolean Ymode = false;
    boolean Xmode = false;
    boolean Bmode= false;
    boolean X2mode = false;

    public static double clawClose = 0;//previous .25
    public static double clawOpen = 0.5;
    public static double clawLooseClose = 0.1;

    boolean pressClaw = false;

    double posX = 0;
    double posY = 0;
    double posH = yawOrigin;//this is the origin

    double scale = 1;

    double maxAcceleration = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        determineColor colorTest = new determineColor();
        horizontalPositions = doCoolThingies.magicalHorizontalMacro(horizontalTarget,1);//what should we use adjustment for?
        verticalPositions = doCoolThingies.magicalVerticalMacro(verticalTarget);

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

        //ptoL.setPosition(disEngageL);
        //ptoR.setPosition(disEngageR);


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
        waitForStart();
        resetRuntime();

        // Loop until the OpMode ends
        while (opModeIsActive()) {



            Position2d odoPos = getWheeledLocalization.wheeledLocalization(leftFrontDrive/*en1,left*/, rightBackDrive/*en2,right*/, rightFrontDrive/*enc3, bakc*/, posX, posY, posH);
            posX = odoPos.getThisX();
            posY = odoPos.getThisY();
            posH = odoPos.getThisHeading();

            telemetry.addData("posX", posX);

            telemetry.addData("posY", posY);

            telemetry.addData("posH", posH);

            telemetry.addData("en1", leftFrontDrive.getCurrentPosition());
            telemetry.addData("en2", rightBackDrive.getCurrentPosition());
            telemetry.addData("en3", rightFrontDrive.getCurrentPosition());


            double Time = runtime.time();


            telemetry.addData("hsSTATE", horizontalTarget);
            telemetry.addData("vsSTATE", verticalTarget);
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


            if(gamepad1.right_bumper && !pressClaw && (claw.getPosition() == clawOpen || claw.getPosition() == clawLooseClose)){
                claw.setPosition(clawClose);
                pressClaw = true;
            }else if (gamepad1.right_bumper && !pressClaw && claw.getPosition() == clawClose){
                claw.setPosition(clawLooseClose);
                pressClaw = true;
            }else if(!gamepad1.right_bumper){
                pressClaw = false;
            }

            if(gamepad1.left_bumper){
                claw.setPosition(clawOpen);
            }


            //YELLOW MODE
            if(gamepad1.y && !Ymode && !alreadyPressingY){
                verticalTarget = targetVerticalIdea.STALKER;

                Ymode = true;
                Xmode = false;
                X2mode = false;

                alreadyPressingY = true;
            }else if(gamepad1.y && !alreadyPressingY){


                verticalTarget = nextVerticalTarget;
                alreadyPressingY = true;
            }else if (!gamepad1.y){
                alreadyPressingY = false;
            }

            //SPECIMIN MODE
            if(gamepad1.x && !Xmode && !alreadyPressingX){

                Xmode = true;
                Ymode = false;
                X2mode = false;

                alreadyPressingX = true;

                verticalTarget = targetVerticalIdea.ZERO_COLLECT;

            }else if(gamepad1.x && !alreadyPressingX){
                verticalTarget = nextVerticalTarget;
                alreadyPressingX = true;
            }else if (!gamepad1.x){
                alreadyPressingX = false;
            }

            //HANG MODE
            if(gamepad2.x && !X2mode && !alreadyPressingX2){

                X2mode = true;
                Ymode = false;
                Xmode = false;

                alreadyPressingX2 = true;

                verticalTarget = targetVerticalIdea.READY_TO_HANG1;
                gamepad2.rumble(30000);


            }else if(gamepad2.x && !alreadyPressingX2){
                verticalTarget = nextVerticalTarget;
                alreadyPressingX2 = true;
            }else if (!gamepad2.x){
                alreadyPressingX2 = false;
            }


            if(gamepad1.back & !alreadyPressingBack & !cancelColor){
                alreadyPressingBack = true;
                cancelColor = true;
            }else if(gamepad1.back & !alreadyPressingBack & cancelColor){
                alreadyPressingBack = true;
                cancelColor = false;
            }else if (!gamepad1.back){
                alreadyPressingBack = false;
            }



            //HORIZONTAL ACXTIONS
            if(gamepad1.b && !Bmode && !alreadyPressingB){

                Bmode = true;
                //mode = false;

                alreadyPressingB = true;

                horizontalTarget = targetHorizontalIdea.HOVER_ACROSS_BARIER;

            }else if(gamepad1.b && !alreadyPressingB){
                horizontalTarget = nextHorizontalTarget;
                alreadyPressingB = true;
            }else if (!gamepad1.b){
                alreadyPressingB = false;
            }

            //VERTICAL LOGIC FLOW
/*
            if(verticalTarget == targetVerticalIdea.PRE_ZERO) {
                nextVerticalTarget = targetVerticalIdea.ZERO_VS_SLIDES;
            }else if(verticalTarget == targetVerticalIdea.ZERO_VS_SLIDES) {
                nextVerticalTarget = targetVerticalIdea.READY_VS_POS;
*/

            if(verticalTarget == targetVerticalIdea.STALKER){
                nextVerticalTarget = targetVerticalIdea.SNATCH_THAT_FISHY;
            }else if(verticalTarget == targetVerticalIdea.SNATCH_THAT_FISHY){
                nextVerticalTarget = targetVerticalIdea.SAFE_RAISE;
            }else if(verticalTarget == targetVerticalIdea.SAFE_RAISE){
                nextVerticalTarget = targetVerticalIdea.DEPOSIT_POTATO;
            }else if(verticalTarget == targetVerticalIdea.DEPOSIT_POTATO){
                nextVerticalTarget = targetVerticalIdea.STALKER;

            }else if(verticalTarget == targetVerticalIdea.ZERO_COLLECT) {
                nextVerticalTarget = targetVerticalIdea.COLLECT_SPECIMIN;
            }else if(verticalTarget == targetVerticalIdea.COLLECT_SPECIMIN){
                nextVerticalTarget = targetVerticalIdea.PRE_SCORE_SPECIMEN;
            }else if(verticalTarget == targetVerticalIdea.PRE_SCORE_SPECIMEN){
                nextVerticalTarget = targetVerticalIdea.SCORE_SPECIMEN;
            }else if(verticalTarget == targetVerticalIdea.SCORE_SPECIMEN){
                nextVerticalTarget = targetVerticalIdea.PRE_SCORE_SPECIMEN;


            }else if (verticalTarget == targetVerticalIdea.READY_TO_HANG1) {
                nextVerticalTarget = targetVerticalIdea.READY_TO_HANG2;
            }else if (verticalTarget == targetVerticalIdea.READY_TO_HANG2) {
                nextVerticalTarget = targetVerticalIdea.ENGAGE_PTO;
            }else if (verticalTarget == targetVerticalIdea.ENGAGE_PTO) {
                nextVerticalTarget = targetVerticalIdea.ACCEND;
            }else if (verticalTarget == targetVerticalIdea.ACCEND) {
                nextVerticalTarget = targetVerticalIdea.ENGANGE_SUPPORT;
            }


            //HORIZONTAL LOGIC FLOW
            if(horizontalTarget == targetHorizontalIdea.READY_HS_POS){
                nextHorizontalTarget = targetHorizontalIdea.HOVER_ACROSS_BARIER;
            }else if(horizontalTarget == targetHorizontalIdea.HOVER_ACROSS_BARIER){
                nextHorizontalTarget = targetHorizontalIdea.FULL_EXTENT_DROP;
            }else if(horizontalTarget == targetHorizontalIdea.FULL_EXTENT_DROP){
                nextHorizontalTarget = targetHorizontalIdea.ZERO_HS_SLIDES;
            }else if(horizontalTarget == targetHorizontalIdea.ZERO_HS_SLIDES){
                nextHorizontalTarget = targetHorizontalIdea.READY_HS_POS;
            }



            if(claw.getPosition() != clawClose && (horizontalTarget == targetHorizontalIdea.HOVER_ACROSS_BARIER || horizontalTarget == targetHorizontalIdea.FULL_EXTENT_DROP)){

            }


            //DETERMINING POSIIONS
            horizontalPositions = doCoolThingies.magicalHorizontalMacro(horizontalTarget,1);//setting to the corrosponding target
            verticalPositions = doCoolThingies.magicalVerticalMacro(verticalTarget);

            hsTarget = horizontalPositions.getHSpos();

            if(!killHorizontal){
                liftL.setPosition(horizontalPositions.getintakeLiftPos());
                liftR.setPosition(horizontalPositions.getintakeLiftPos());
            }

            vsTarget = verticalPositions.getVSpos();

            if(!killVertical){
                armL.setPosition(verticalPositions.getshoulderPos());
                armR.setPosition(verticalPositions.getshoulderPos());
                wristL.setPosition(verticalPositions.getwristLPos());
                wristR.setPosition(verticalPositions.getwristRPos());
            }


            currentColor = colorTest.color(intakeColor, getRuntime());




            telemetry.addData("currentColor is", currentColor);
            double colorChangeDelayServo = 0.9 + 0.25;

            if(!cancelColor){
                if(currentColor  == "yellow" || currentColor == "blue" && horizontalTarget == targetHorizontalIdea.FULL_EXTENT_DROP){
                    horizontalTarget = targetHorizontalIdea.ZERO_HS_SLIDES;
                }


                if(currentColor == "red" || currentColor  == "yellow" || currentColor == "blue"){
                    waitUntil = getRuntime() + colorChangeDelayServo;
                }
                if(getRuntime() < waitUntil){
                    flickPower = -1;
                    telemetry.addData("Servo power", -1);
                }else{
                    flickPower = 0;
                    //flick.setPower(-gamepad1.right_trigger);
                }
            }

            if(gamepad1.left_trigger > 0.1 & gamepad1.right_trigger < 0.1){//jui left
                intake.setPower(-gamepad1.left_trigger);
                flickPower = 1;

            }else if(gamepad1.right_trigger > 0.1 & gamepad1.left_trigger < 0.1){//just right
                intake.setPower(gamepad1.right_trigger);
            }else if (gamepad1.right_trigger > 0.1 & gamepad1.left_trigger > 0.1){
                intake.setPower(gamepad1.right_trigger);
                flickPower = -gamepad1.right_trigger;
            }else{
                intake.setPower(0);
                if(cancelColor){
                    flickPower = 0;
                }
            }

            flick.setPower(flickPower);




            //HANG
            if(verticalPositions.getHookThat()){
                hookL.setPosition(hookLhook);
               hookR.setPosition(hookRhook);
            }

            if(verticalPositions.getPTO()){
                if (gamepad2.right_bumper){
                    ptoR.setPosition(engageR);
                    gamepad2.stopRumble();
                }else{
                    ptoR.setPosition(disEngageR);
                }
                if (gamepad2.left_bumper){
                    ptoL.setPosition(engageL);
                    gamepad2.stopRumble();
                }else{
                    ptoL.setPosition(disEngageL);
                }

                driveHange = true;
            }else{
                //ptoL.setPosition(disEngageL);
                //ptoR.setPosition(disEngageR);
                driveHange = false;
            }



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

            //double unOriginedxpos = pos.x;
            //double unOriginedypos = pos.y;

            double unOriginedxpos = posX;
            double unOriginedypos = posY;

            double xpos = posX;
            double ypos = posY;
            double heading = pos.h + Math.toRadians(yawOrigin);
            //potential problem: taking more than one reading throughout the program

            normalHeading = heading;
            if (normalHeading != Math.abs(normalHeading)){//negatinve
                normalHeading = normalHeading + 2*3.14159265;
            }
//POSSIBLY KEEP INCASE A RESET IS NEEDED

            /*if (gamepad1.y) {
                myOtos.resetTracking();
            }

            */

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


            if(!driveHange){

                if(Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0){
                    axial   = gamepad1.left_stick_y;//drive  // Note: pushing stick forward gives negative value
                    lateral =  -gamepad1.left_stick_x;//strafe
                    yaw     =  -gamepad1.right_stick_x;
                }else{
                    axial   = gamepad2.left_stick_y;//drive  // Note: pushing stick forward gives negative value
                    lateral =  -gamepad2.left_stick_x;//strafe
                    yaw     =  -gamepad2.right_stick_x;
                }


            }else{
                axial = -vsOutput;
                lateral = 0;
                yaw     = 0;
            }

            /*if(gamepad1.a & !pressingA1){
                scale = 0.5;
            }*/

            if(gamepad1.a){
                scale = 0.5;
            }else{
                scale = 1;
            }


            telemetry.addData("axial", axial);


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

            leftFrontDrive.setPower(leftFrontPower * scale);
            rightFrontDrive.setPower(rightFrontPower * scale);
            leftBackDrive.setPower(leftBackPower * scale);
            rightBackDrive.setPower(rightBackPower * scale);

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
            finalX = xpos;// + originX;
            finalY = ypos;// + originY;


            VxVyAxAy velocities = vectorSystem.getvelocity(Time, finalX, finalY, posH);


            double maxDecc = -50;//inches/sec**2

            double errorY = targetY - finalY;
            double hypoVelo = Math.sqrt((Math.pow((velocities.getVy()), 2)) + 2 * maxDecc * errorY);//hypo velo is the velo upon arrival

            maxAcceleration = Math.max(velocities.getAx(), maxAcceleration);

            telemetry.addData("Run Time:", Time);
            //telemetry.addData("hypoVeloY:", hypoVelo);
            //telemetry.addData("maxAx:", maxAcceleration);
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

            //yawOutput = PID.YawPID(pos.h + yawOrigin, getRuntime(), Math.toRadians(yawTarget));
            yawOutput = PID.YawPID(heading, getRuntime(), Math.toRadians(yawTarget));
            vxOutput = PID.vxPID(finalX, getRuntime(), targetX);
            vyOutput = PID.vyPID(finalY, getRuntime(), targetY);

            //ZEROING SLIDES WITH TOUCH SENSOR
            //HS SLIDES


            if(hsTouch.isPressed() && horizontalTarget == targetHorizontalIdea.ZERO_HS_SLIDES){
                horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                horizontalTarget = targetHorizontalIdea.READY_HS_POS;
            }

            /*if(vsTouch.isPressed() && verticalTarget == targetVerticalIdea.ZERO_VS_SLIDES){
                verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalTarget = targetVerticalIdea.READY_VS_POS;
            }*/

            if(vsTouch.isPressed() && verticalTarget == targetVerticalIdea.ZERO_COLLECT){
                verticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalTarget = targetVerticalIdea.COLLECT_SPECIMIN;
            }

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