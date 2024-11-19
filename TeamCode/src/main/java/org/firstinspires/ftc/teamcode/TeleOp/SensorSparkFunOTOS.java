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
@TeleOp(name = "Sensor: SparkFun OTOS", group = "Sensor")
public class SensorSparkFunOTOS extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    public static double testttt = 1; // Example of FTC dashboard variable

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo servo = null;

    double finalX = 0;
    double finalY = 0;
    double originY = -50;
    double originX = -50;



    double avgHeading = 0;
    double currentDriveX = 0;
    double currentDriveY = 0;


    public static double yawKp = 0.7, yawKi = 0.01, yawKd = 0.001;  // PID gains

    private double previousError = 0;
    private double integralSum = 0;
    public static double yawTarget = (0);   // Desired target value
    public static double targetX;
    public static double targetY;




    private double previousTime = 0; // Time from previous iteration
    private double yawOutput = 0;
    private double vxOutput = 0;
    private double vyOutput = 0;

    ComputePid PID = new ComputePid();


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

        servo = hardwareMap.get(Servo.class, "servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        configureOtos();

        // Wait for the start button to be pressed
        waitForStart();

        // Loop until the OpMode ends
        while (opModeIsActive()) {




            if (gamepad1.b) {
                servo.setPosition(0);
            }
            if (gamepad1.a) {
                servo.setPosition(1);
            }

            double radius = gamepad1.left_stick_x;
            telemetry.addData("radius", radius);

            double TelemX = 1 * -finalX;
            double TelemY = 1 * -finalY;

            double adjH = avgHeading - (3.14159265/2);
            double[] xcordHead = {TelemX - 3, TelemX + 3, TelemX + 15 * Math.cos(avgHeading), TelemX + 15 * Math.cos(avgHeading)};
            double[] ycordHead = {TelemY, TelemY, TelemY + 15 * Math.sin(avgHeading), TelemY + 15 * Math.sin(avgHeading)};

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("cyan")
                    .fillCircle(TelemX, TelemY, 6)
                    .fillCircle(TelemX + 15 * Math.cos(adjH), TelemY + 15 * Math.sin(adjH), 5);

            dashboard.sendTelemetryPacket(packet);

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            double oldx = pos.x;
            double oldy = pos.y;
            double oldheading = pos.h;

            avgHeading = pos.h;
            if (avgHeading != Math.abs(avgHeading)){//negatinve
                avgHeading = avgHeading + 2*3.14159265;
            }

            if (gamepad1.y) {
                myOtos.resetTracking();
            }

            if (gamepad1.x) {
                myOtos.calibrateImu();
            }

            double max;
            //double axial = gamepad1.left_stick_y;
            //double lateral = -gamepad1.left_stick_x;
            double axial = -vyOutput * Math.sin(avgHeading) + -vyOutput * Math.cos(avgHeading);;
            double lateral = -vxOutput * Math.cos(avgHeading) - -vyOutput * Math.sin(avgHeading);
            //double yaw = -gamepad1.right_stick_x;

            //if (gamepad1.left_bumper){
            //    target = pos.h;
            //}
            //if(!gamepad1.right_bumper){
            double yaw = yawOutput;
            //}

            //lateral = lateral * Math.cos(avgHeading) - axial * Math.sin(avgHeading);
            //axial = lateral * Math.sin(avgHeading) + axial * Math.cos(avgHeading);


           /* if (lateral >= .2 || axial >= .2){
                yaw = 0;
            }*/

            if(gamepad1.left_trigger > .1){
                yaw = 0;
            }

            if (gamepad1.right_trigger > .1){
                lateral = 0;
                axial = 0;
            }

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

          //  avgHeading = (newHeading + oldheading) / 2;

            // Adjusting position calculation to maintain accuracy when heading changes drastically
            //double adjustedHeading = Math.abs(pos.h) > Math.PI ? pos.h - (Math.signum(pos.h) * 2 * Math.PI) : pos.h;

            /*double actualXchange = deltaX * Math.cos(avgHeading) - deltaY * Math.sin(avgHeading);
            double actualYchange = deltaX * Math.sin(avgHeading) + deltaY * Math.cos(avgHeading);*/

            //finalX = finalX + deltaX;
           // finalY = finalY + deltaY;
            finalX = pos.x + originX;
            finalY = pos.y + originY;


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addData("X coordinate", finalX);
            telemetry.addData("Y coordinate", finalY);
            telemetry.addData("Heading radians", avgHeading);
            telemetry.update();
/*
//heading pid testing
            double thisTime = getRuntime();
            // Calculate the error
            double error = Math.toRadians(target) - pos.h;

            // Calculate the time difference
            double deltaTime = thisTime - previousTime;
            if (deltaTime == 0) deltaTime = 0.001; // Prevent division by zero

            // Proportional term
            double proportionalTerm = yawKp * error;

            // Integral term (sum of previous errors)
            integralSum += error * deltaTime;
            double integralTerm = yawKi * integralSum;

            // Derivative term (rate of change of error)
            double derivativeTerm = yawKd * (error - previousError) / deltaTime;

            // Combine terms to get the final output
            output = proportionalTerm + integralTerm + derivativeTerm;

            // Store current error and time for next iteration
            previousError = error;
            previousTime = thisTime;

            // Return the output (e.g., motor power or other control signal)
*/
            yawOutput = PID.YawPID(pos.h, getRuntime(), Math.toRadians(yawTarget));
            vxOutput = PID.vxPID(finalX, getRuntime(), targetX);
            vyOutput = PID.vyPID(finalY, getRuntime(), targetY);

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