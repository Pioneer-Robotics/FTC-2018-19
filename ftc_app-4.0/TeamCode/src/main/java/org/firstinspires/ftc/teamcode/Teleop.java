package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="LL5156:POV", group="LL5156")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareLL5156 robot           = new HardwareLL5156();
    static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    //double          clawOffset      = 0;
    //final double    CLAW_SPEED      = 0.02 ;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        //double arm;
        //double armMax;

        robot.init(hardwareMap);

        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;
            //arm = gamepad1.right_stick_y;
            linear_up = gamepad2.left_trigger;
            linear_down = gamepad2.right_trigger;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            //armMax = Math.abs(arm);
            if (max > 1.0/* || armMax > 1.0*/)
            {
                left /= max;
                right /= max;
                //arm /= armMax;
            }

            // Output the safe vales to the motor drives.
            robot.motorLeft.setPower(left);
            robot.motorRight.setPower(right);
            robot.linearArm.setPower(linear_up);
            robot.linearArm.setPower(-linear_down);

            // Use gamepad left & right Bumpers to open and close the claw
            /*if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;*/

            // Move both servos to new position.  Assume servos are mirror image of each other.
            /*clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);*/

            // Use gamepad buttons to move arm up (Y) and down (A)
            /*if (gamepad1.y)
                robot.leftArm.setPower(robot.ARM_UP_POWER);
            else if (gamepad1.a)
                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
            else
                robot.leftArm.setPower(0.0);*/

            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("arm","%.2f",linear_up - linear_down);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }

    public void encoderArmSet()
    {
        if (gamepad2.left_trigger == 1) //linear arm down
        {
            robot.linearArm.setTargetPosition(4000); //test actual values
            robot.linearArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();

            robot.linearArm.setPower(0);
            robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.right_trigger == 1) //linear arm up
        {
            robot.linearArm.setTargetPosition(0); //test actual values
            robot.linearArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();

            robot.linearArm.setPower(0);
            robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
