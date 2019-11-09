package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.nio.ReadOnlyBufferException;

@TeleOp(name = "Teleop", group = "Sensor")
public class TeleopTester extends LinearOpMode {

    Robot robot = new Robot();

    DeltaTime deltaTime = new DeltaTime();

    int last_bl;

    int last_br;

    double targetRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armWintch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armWintch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        targetRotation = robot.GetRotation();

        while (opModeIsActive()) {
            deltaTime.Start();

            //Rotate 180 degrees in one seconds
            targetRotation = 25 * gamepad1.right_stick_x * deltaTime.deltaTime();

            //Move via joystick and maintain target rotation, rotation might not work
            robot.MoveComplex(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.a ? 1 : 0.1, 0);
//            robot.MoveComplex(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.a ? 1 : 0.1, Math.toRadians(targetRotation));



            if (gamepad1.dpad_right) {
//                robot.gripServo.setPosition(1);
            }
            if (gamepad1.dpad_left) {
//                robot.gripServo.setPosition(-1);
            }
//            telemetry.addData("encoder value", robot.armWintch.getCurrentPosition());
//            telemetry.update();
//            robot.armWintch.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);

            targetRotation = 0;
            deltaTime.Stop();
        }

    }
}