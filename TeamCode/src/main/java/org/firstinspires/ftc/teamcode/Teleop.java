package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="FTCPio")
public class Teleop extends OpMode
{
    /* Declare OpMode members. */
    private HardwareInfinity robot = new HardwareInfinity();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private double pre_suq = 0;
    private boolean armBAuto;
    private boolean FBarAuto;
    private boolean dmac = false;
    private boolean flipster = true;
    private boolean flippy;
    private boolean telemet = false;
    private static final double TETRIX_TICKS_PER_REV = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CM = 4.0 * 2.54;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (TETRIX_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    // State used for updating telemetry;
    @Override
    public void init() {

        /*imu = hardwareMap.get(Gyroscope.class, "imu");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");*/
        //Acceleration gravity = imu.getGravity();

        robot.init(hardwareMap, this, runtime, COUNTS_PER_INCH);
        telemetry.addData("Teleop", "Initiate");    //
        telemetry.update();
        robot.Camera.setPosition(0);
        robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
        robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
        robot.armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
    }
    @Override
    public void start() {
        telemetry.addData("Teleop","Started");
        time.reset();
    }
    @Override
    public void loop()
    {
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        double drive = gamepad1.left_stick_y;
        double turn = -gamepad1.left_stick_x;
        double armB = -gamepad2.left_stick_y / 4 * (1 + 3 * gamepad2.left_trigger);
        double bar = gamepad2.right_stick_y;
        double activate_suq = -Math.copySign(Math.pow(gamepad1.right_stick_y, 2), gamepad1.right_stick_y);
        if (telemet) {
            telemetry.addData("Succq:", activate_suq);
            telemetry.addData("Succq Encoder: ", "%d", robot.Succq.getCurrentPosition());
            telemetry.addData("Condition: ", "%.5f", Math.abs(robot.dropTop.getPosition() - HardwareInfinity.DT_MIN));
            telemetry.addData("Flipster: ", flipster ? "True" : "False");
            telemetry.addData("DT pos: ", robot.dropTop.getPosition());
        }
        double arm;
        if (gamepad1.left_bumper) {
            arm = 1;
        } else if (gamepad1.right_bumper) {
            arm = -1;
        } else arm = 0;
        if (telemet) {
            //telemetry.addData("Trigger is", robot.trigger.isPressed() ? "Pressed" : "not Pressed");
            telemetry.addData("Bottom is", robot.botSwitch.getState() ? "Pressed" : "not Pressed");
            telemetry.addData("Top is", robot.topSwitch.getState() ? "Pressed" : "not Pressed");
        }
        // Output the safe vales to the motor drives.
        if ((!robot.botSwitch.getState() && !robot.topSwitch.getState() && !robot.trigger.isPressed()))
        {
            robot.linearArm.setPower(-arm);
        }
        else if ((robot.botSwitch.getState() || robot.trigger.isPressed()) && arm < 0)
        {
            robot.linearArm.setPower(-arm);
        }
        else if ((robot.botSwitch.getState() || robot.trigger.isPressed()) && arm >= 0)
        {
            robot.linearArm.setPower(0);
        }
        else if (arm > 0)
        {
            robot.linearArm.setPower(-arm);
        }
        else if (robot.topSwitch.getState())
        {
            robot.linearArm.setPower(0);
        }
        //blended motion
        double left = (drive + turn) / 2 * (gamepad1.right_trigger * 3 / 2 + 1) * 0.75;
        double right = (drive - turn) / 2 * (gamepad1.right_trigger * 3 / 2 + 1) * 0.75;
        if (telemet) {
            telemetry.addData("Multiplier:", (gamepad1.right_trigger * 3 / 2 + 1) * 0.75);
        }

        // Normalize the values so neither exceed +/- 1.0
        double max = (Math.max(Math.abs(left), Math.abs(right))) / 2;
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Decrease power input and flip direction so line-up for reattachment on lander is easier
        if (gamepad1.y)
        {
            left *= -0.5*(1-gamepad1.left_trigger);
            right *= -0.5*(1-gamepad1.left_trigger);
            robot.motorLeft.setPower(right);
            robot.motorRight.setPower(left);
            if (telemet) telemetry.addData("Reverse","Activated");
        }
        else if (gamepad1.x)
        {
            left *= -1;
            right *= -1;
            robot.motorLeft.setPower(right);
            robot.motorRight.setPower(left);
            if (telemet) telemetry.addData("Reverse","Mineral Mode");
        }
        else
        {
            robot.motorLeft.setPower(left);
            robot.motorRight.setPower(right);
            if (telemet) telemetry.addData("Reverse", "Deactivated");
        }

        /*if (gamepad1.right_bumper) {
            if (!flipster) {
                if (asuq == 0) {
                    asuq = 1;
                    activate_suq = asuq;
                    robot.Succq.setPower(activate_suq);
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e){
                    }
                } else {
                    asuq = 0;
                }
                flipster = true;
            }
        } else {
            flipster = false;
        }*/
        /*if (gamepad2.left_bumper) robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        else robot.armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.right_bumper) robot.FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        else robot.FBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        /*if (gamepad1.left_bumper) {
            if (!flipster1) {
                asuq = -asuq;
            }
            flipster1 = true;
        } else {
            flipster1 = false;
        }*/

        if (gamepad1.b) {
            if (flipster) {
                if (Math.abs(robot.dropTop.getPosition() - HardwareInfinity.DT_MIN)<=0.1) {
                    robot.dropTop.setPosition(HardwareInfinity.DT_MAX);
                } else robot.dropTop.setPosition(HardwareInfinity.DT_MIN);
            }
            flipster = false;
        } else {
            flipster = true;
        }
        if (robot.armBase.getCurrentPosition()<-3000 && !gamepad1.a) {
            robot.dropTop.setPosition(HardwareInfinity.DT_MAX);
        }
        //if the Succq isn't moving then stop it to save the motor
        if ((activate_suq !=0) && (pre_suq == robot.Succq.getCurrentPosition()) && gamepad1.right_stick_y==0) {
            activate_suq = 0;
        }
        robot.Succq.setPower(activate_suq);
        pre_suq = robot.Succq.getCurrentPosition();
        if (telemet) {
            telemetry.addData("Arm Base Power: ", "%.5f", armB);
            telemetry.addData("Arm Base Encoder: ", "%d", robot.armBase.getCurrentPosition());
        }
        if (!armBAuto) {
            robot.armBase.setPower(armB);
        } else if (armB !=0 || gamepad2.left_bumper || gamepad2.a) {
            robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armBase.setPower(0);
            armBAuto = false;
            dmac = false;
        }
        if (telemet) {
            telemetry.addData("4Bar Power: ", "%.5f", bar);
            telemetry.addData("4Bar Encoder: ", "%d", robot.FBar.getCurrentPosition());
        }
        if (!FBarAuto) {
            robot.FBar.setPower(bar);
        } else if (bar !=0 || gamepad2.right_bumper|| gamepad2.a) {
            robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FBar.setPower(0);
            FBarAuto = false;
            dmac = false;
        }
        // Controls latching servos on linear actuator
        // Latch open
        if (gamepad1.dpad_right)
        {
            robot.Latch.setPosition(HardwareInfinity.LatchMAX_POSITION);
            if (telemet) telemetry.addData("Latches","Max");
        }
        // Latch closed
        if (gamepad1.dpad_left)
        {
            robot.Latch.setPosition(HardwareInfinity.LatchMIN_POSITION);
            if (telemet) telemetry.addData("Latches","Min");
        }
        if ((gamepad1.dpad_down || gamepad2.dpad_down) && robot.dropTop.getPosition()>HardwareInfinity.DT_MIN) {
            robot.dropTop.setPosition(robot.dropTop.getPosition()-0.015);
        }
        if ((gamepad1.dpad_up || gamepad2.dpad_up) && robot.dropTop.getPosition()<HardwareInfinity.DT_MAX) {
            robot.dropTop.setPosition(robot.dropTop.getPosition()+0.015);
        }

        // The following gamepad2 inputs are macros to move the EMCD 2.0 between its collection and depositing positions
        if (gamepad2.x) {
            armBAuto = true;
            robot.armBase.setTargetPosition(-300);
            robot.armBase.setPower(-1);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FBarAuto = true;
            robot.FBar.setTargetPosition(-25000);
            robot.FBar.setPower(-1);
            robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dmac = true;
            time.reset();
        }

        if (gamepad2.a) {
            armBAuto = true;
            robot.armBase.setTargetPosition(-300);
            robot.armBase.setPower(1);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FBarAuto = true;
            robot.FBar.setTargetPosition(-300);
            robot.FBar.setPower(1);
            robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.y) {
            armBAuto = true;
            robot.armBase.setTargetPosition(-300);
            robot.armBase.setPower(-1);
            robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FBarAuto = true;
            robot.FBar.setTargetPosition(-20500);
            robot.FBar.setPower(-1);
            robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (telemet) {
            telemetry.addData("Time: ", "%.5f", time.milliseconds());
            telemetry.addData("dmac:", "%b", dmac);
        }
        if (dmac && (time.milliseconds() >= 4000 || (!robot.armBase.isBusy() && !robot.FBar.isBusy()))) {
                robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armBase.setTargetPosition(-6750);
                robot.armBase.setPower(-1);
                robot.armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.FBar.setTargetPosition(-24750);
                robot.FBar.setPower(-1);
                robot.FBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            dmac = false;
        }
        if (robot.armBase.isBusy()) {
            armBAuto = true;
        } else if (armBAuto) {
            armBAuto = false;
            robot.armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (robot.FBar.isBusy()) {
            FBarAuto = true;
        } else if (FBarAuto) {
            FBarAuto = false;
            robot.FBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
         // Drops team marker with servo
        if (gamepad1.x)
        {
            if (flippy) {
                if (Math.abs(robot.lunchBox.getPosition() - HardwareInfinity.lunchBoxMIN_POSITION)<=0.1) {
                    robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMAX_POSITION);
                } else robot.lunchBox.setPosition(HardwareInfinity.lunchBoxMIN_POSITION);
            }
            flippy = false;
        } else {
            flippy = true;
        }
        // Send telemetry message to signify robot running;
        if (telemet) {
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("arm", "%.2f", arm);
            telemetry.addData("Motor Encoder", "%d", robot.linearArm.getCurrentPosition());
        }
    }
}
