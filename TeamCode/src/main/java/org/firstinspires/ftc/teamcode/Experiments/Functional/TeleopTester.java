package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.nio.ReadOnlyBufferException;

@TeleOp(name = "Teleop", group = "Sensor")
public class TeleopTester extends LinearOpMode {

    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    int last_bl;

    int last_br;

    double targetRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
//        robot.armWintch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armWintch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        targetRotation = robot.GetRotation();

        while (opModeIsActive()) {

            targetRotation = gamepad1.right_stick_x / 10;

            //Move via joystick and maintain target rotation, rotation might not work
//            robot.MoveSimple(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.a ? 0.1 : 1, targetRotation);


//            robot.MoveComplex(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.a ? 1 : 0.1, Math.toRadians(targetRotation));
            robot.arm.rotation.setPower(gamepad1.right_stick_y);
            robot.arm.length.setPower(gamepad1.right_stick_x);


            if (gamepad1.dpad_right) {
//                robot.gripServo.setPosition(1);
            }
            if (gamepad1.x) {
                robot.arm.SetState(0.5, 1, 0.5, 0.5);
            }

            if (gamepad1.a) {
                robot.arm.SetState(0, 0, 0.5, 0.5);
            }
//            telemetry.addData("encoder value", robot.armWintch.getCurrentPosition());
//            telemetry.update();
//            robot.armWintch.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            deltaTime.reset();

        }
        robot.Stop();
    }
}