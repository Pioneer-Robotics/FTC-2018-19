package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@TeleOp(name = "Teleop2", group = "Sensor")
public class TeleopTester2 extends LinearOpMode {

    RobotWallTrack.SensorGroup targetWallTrackGroup = null;

    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    public boolean lockRotation;
    public boolean lastLockRotation;

    double targetRotation;

    double moveSpeed;
    double rotateSpeed;
    double targetRotationOffset;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
//        robot.armWintch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armWintch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        targetRotation = robot.GetRotation();

        while (opModeIsActive()) {

            moveSpeed = bMath.Clamp(gamepad1.right_trigger + 0.35, 0, 1);
            rotateSpeed = bMath.Clamp(gamepad1.left_trigger + 0.35, 0, 1);


            if (targetWallTrackGroup == null) {
                if (lockRotation) {
                    robot.MoveComplex(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), moveSpeed, robot.GetRotation() - (targetRotation  /* + targetRotationOffset*/));
                } else {
                    robot.MoveSimple(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), moveSpeed, gamepad1.right_stick_x * rotateSpeed);
                }
            } else {
//                robot.MoveSimple(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y) + bMath.degreesToHeadingVector(targetWallTrackGroup.getWallAngle()), moveSpeed, gamepad1.right_stick_x * rotateSpeed);
            }

            if (gamepad1.a) {
                lockRotation = !lockRotation;
                sleep(350);
            }

            if (lastLockRotation != lockRotation) {
                lastLockRotation = lockRotation;
                if (lockRotation) {
                    targetRotation = robot.GetRotation();
                }
            }

            //Move relative to a sensor group
            if (gamepad1.left_stick_button) {
                targetWallTrackGroup = robot.wallTrack.closestGroup();
            }

            //Clear the current sensor group
            if (gamepad1.right_stick_button) {
                targetWallTrackGroup = null;
            }

            if (lockRotation) {
//                targetRotationOffset += deltaTime.seconds() * 180 * gamepad1.right_stick_x;

                if (gamepad1.x) {
                    targetRotationOffset = bMath.MoveTowards(targetRotationOffset, 0, deltaTime.seconds() * 2);
                }
            } else {
                targetRotationOffset = 0;
            }
            telemetry.addData("Rotation Locked ", lockRotation);
            telemetry.addData("", "");
            telemetry.addData("Current Rotation ", robot.GetRotation());
            telemetry.addData("Current Target Rotation", targetRotation);
            telemetry.update();

            deltaTime.reset();

        }
        robot.Stop();
    }
}