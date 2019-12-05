package org.firstinspires.ftc.teamcode.Experiments.Functional;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

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
//            robot.arm.SetArmState(0.1, gamepad1.left_stick_x, 1, 1);
//            robot.arm.SetGripState(gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Grip Postion", gamepad1.left_stick_x);
            telemetry.addData("Grip Rotaiton Position", gamepad1.left_stick_y);
            telemetry.update();

//            if (gamepad1.x) {
//                robot.arm.SetArmState(0.5, 1, 0.5, 1);
//            }
//
//            if (gamepad1.a) {
//                robot.arm.SetArmState(0, 0, 0.5, 1);
//            }
//            telemetry.addData("encoder value", robot.armWintch.getCurrentPosition());
//            telemetry.update();
//            robot.armWintch.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            deltaTime.reset();

        }
        robot.Stop();
    }
}