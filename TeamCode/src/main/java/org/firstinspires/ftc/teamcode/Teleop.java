package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="LL5156:POV", group="LL5156")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareLL5156 robot           = new HardwareLL5156();
    static final double     TETRIX_TICKS_PER_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_CM   = 4.0*2.54 ;
    static final double     COUNTS_PER_INCH         = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    //double          clawOffset      = 0;
    //final double    CLAW_SPEED      = 0.02 ;
    // Servo Lunch box: 0.8 = straight up, 0.55 = dropped #2 - A
    // Servo rgtLatch: 0.95 = up, 0.63 = down #0 - up and down on D pad
    // Servo lftLatch; 0.06 = up, 0.36 = down #1 - up and down on D pad
    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double linear_up;
        double linear_down;
        double arm;
        double armMax;
        double interval = 0.1;
        /*
        lunchBox.MAX_POSITION = 0.8;
        lunchBox.MIN_POSITION = 0.55;
        rgtLatch.MAX_POSITION = 0.95;
        rgtLatch.MIN_POSITION = 0.63;
        lftLatch.MAX_POSITION = 0.06;
        lftLatch.MAX_POSITION = 0.36;
        */

        robot.init(hardwareMap);

        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.linearSwitch.setMode(DigitalChannel.Mode.INPUT);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;
            arm = gamepad2.right_stick_y;
            //linear_up = gamepad2.left_trigger;
            //linear_down = gamepad2.right_trigger;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = (Math.max(Math.abs(left), Math.abs(right)))/2;
            armMax = Math.abs(arm);
            if (max > 1.0 || armMax > 1.0)
            {
                left /= max;
                right /= max;
                arm /= armMax;
            }

            // Output the safe vales to the motor drives.
            robot.motorLeft.setPower(-left);
            robot.motorRight.setPower(-right);

            if (!robot.linearSwitch.getState())
            {
                robot.linearArm.setPower(-arm);
                telemetry.addData("Switch","is pressed");
            }
            else
            {
                telemetry.addData("Switch","is not pressed");
            }

            //robot.linearArm.setPower(linear_up);
            //robot.linearArm.setPower(-linear_down);

            /* Gradual implementation of latch
            if (gamepad1.dpad_up && (rgtLatch.getPosition() < rgtLatch.MAX_POSITION) && (lftLatch.getPosition() < lftLatch.MAX_POSITION)
            {
                rgtLatch.setPosition(rgtLatch.getPosition() + 0.05);
                lftLatch.setPosition(lftLatch.getPostition() + 0.05);
            }
            */
            /*
            if (gamepad1.dpad_up && (left < 1) && (left > -1) && (right < 1) && (right > -1))
            {

                if (left < 0)
                {
                    left += interval;
                }
                else
                {
                    left -= interval;
                }
                if (right < 0)
                {
                    right += interval;
                }
                else
                {
                    right -= interval;
                }
                robot.motorLeft.setPower(-left);
                robot.motorRight.setPower(-right);
            }
            if (gamepad1.dpad_down && (left < 1) && (left > -1) && (right < 1) && (right > -1))
            {
                if (left < 0)
                {
                    left -= interval;
                }
                else
                {
                    left += interval;
                }
                if (right < 0)
                {
                    right -= interval;
                }
                else
                {
                    right += interval;
                }
                robot.motorLeft.setPower(-left);
                robot.motorRight.setPower(-right);
            }
            */
            // Changes direction of movement (easier for driving backwards for scaling lander) and decreases power for more precise movement
            if (gamepad1.y)
            {
                left *= -0.25;
                right *= -0.25;
                robot.motorLeft.setPower(-right);
                robot.motorRight.setPower(-left);
            }
            // Controls latching servos on linear actuator
            if (gamepad2.dpad_up)
            {
                robot.rgtLatch.setPosition(HardwareLL5156.rgtLatchMAX_POSITION);

                robot.lftLatch.setPosition(HardwareLL5156.lftLatchMAX_POSITION);
            }
            if (gamepad2.dpad_down)
            {
                robot.rgtLatch.setPosition(HardwareLL5156.rgtLatchMIN_POSITION);

                robot.lftLatch.setPosition(HardwareLL5156.lftLatchMIN_POSITION);
            }
            // Drops team marker with servo
            if (gamepad2.x)
            {
                robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMIN_POSITION);
                sleep(1000);
                robot.lunchBox.setPosition(HardwareLL5156.lunchBoxMAX_POSITION);
            }
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
                robot.linearArm.setPower(robot.ARM_UP_POWER);
            else if (gamepad1.a)
                robot.linearArm.setPower(robot.ARM_DOWN_POWER);
            else
                robot.linearArm.setPower(0.0);*/

            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }

    /*public void encoderArmSet()
    {
        if (gamepad2.left_trigger == 1) //linear arm down
        {
            robot.linearArm.setTargetPosition(4000); //test actual values
            robot.linearArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.linearArm.setPower(0);
            robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.right_trigger == 1) //linear arm up
        {
            robot.linearArm.setTargetPosition(0); //test actual values
            robot.linearArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            robot.linearArm.setPower(0);
            robot.linearArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/
}
