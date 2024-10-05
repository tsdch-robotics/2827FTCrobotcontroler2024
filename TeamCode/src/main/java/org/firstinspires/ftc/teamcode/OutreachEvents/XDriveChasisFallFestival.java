
package org.firstinspires.ftc.teamcode.OutreachEvents;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="XDriveFallFestival", group="Linear OpMode")
@Disabled
public class XDriveChasisFallFestival extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor quad1 = null;
    private DcMotor quad2 = null;
    private DcMotor quad3 = null;
    private DcMotor quad4 = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        quad1  = hardwareMap.get(DcMotor.class, "quad1");
        quad2  = hardwareMap.get(DcMotor.class, "quad2");
        quad3 = hardwareMap.get(DcMotor.class, "quad3");
        quad3 = hardwareMap.get(DcMotor.class, "quad4");

        quad1.setDirection(DcMotor.Direction.FORWARD);
        quad2.setDirection(DcMotor.Direction.REVERSE);
        quad3.setDirection(DcMotor.Direction.REVERSE);
        quad4.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;

            // stickAngle = Math.atan((gamepad1.left_stick_x/gamepad1.left_stick_y));


            double yaw  =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double quad1Power  = axial + lateral + yaw;
            double quad2Power = axial - lateral - yaw;
            double quad3Power   = axial - lateral + yaw;
            double quad4Power  = axial + lateral - yaw;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.??
            max = Math.max(Math.abs(quad1Power), Math.abs(quad2Power));
            max = Math.max(max, Math.abs(quad3Power));
            max = Math.max(max, Math.abs(quad4Power));

            if (max > 1.0) {
                quad1Power  /= max;
                quad2Power /= max;
                quad3Power   /= max;
                quad4Power  /= max;
            }
            //scales it in terms of the power that happens to be the highest


            // Send calculated power to wheels
            quad1.setPower(quad1Power);
            quad2.setPower(quad2Power);
            quad3.setPower(quad3Power);
            quad4.setPower(quad4Power);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", quad1Power, quad2Power);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", quad3Power, quad4Power);
            telemetry.update();
        }
    }}
//test