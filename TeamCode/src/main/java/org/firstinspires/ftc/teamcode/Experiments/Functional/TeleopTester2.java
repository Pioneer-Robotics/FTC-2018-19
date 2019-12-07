package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotArm;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@TeleOp(name = "Teleop2", group = "Sensor")
public class TeleopTester2 extends LinearOpMode {

    RobotWallTrack.SensorGroup targetWallTrackGroup = null;

    Robot robot = new Robot();

    ElapsedTime deltaTime = new ElapsedTime();

    public boolean lockRotation = false;
    boolean aButton1Check = false;

    double targetRotation;

    double moveSpeed;
    double rotateSpeed;
    double raiseSpeed = 0;
    double targetRotationOffset;

    boolean grab = false;
    boolean bButton2Check = false;

    double extension = 0;
    double armAngle = 0;
    double gripAngle = 180;
    double aTad = 0;
    boolean xButton2Check = false;
    boolean idle = false;
    boolean aButton2Check = false;
    boolean pointDown = false;


    double lunchboxRot = 0.5;



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);


        waitForStart();
        double rotationLockValue = 0;
        targetRotation = robot.GetRotation();

        while (opModeIsActive()) {

            ///DRIVER CONTROLS
            moveSpeed = bMath.Clamp(gamepad1.right_trigger + 0.35, 0, 1);
            rotateSpeed = bMath.Clamp(gamepad1.left_trigger + 0.35, 0, 1);

            targetRotation += gamepad1.left_stick_x * deltaTime.seconds() * 90;
            //targetRotation = bMath.Loop(targetRotation, 360);
            //shouldn't we use deltatime?

            if (gamepad1.a && !aButton1Check) {
                lockRotation = !lockRotation;
                rotationLockValue = robot.GetRotation();
            }
            aButton1Check = gamepad1.a;

            if (lockRotation) {
                targetRotation = rotationLockValue;
                //Ensures that other code cannot change targetRotation while rotationLock is on
                robot.MoveComplex(new Double2(gamepad1.right_stick_x, gamepad1.right_stick_y), moveSpeed, robot.GetRotation() - (targetRotation));
            } else {
                robot.MoveSimple(new Double2(gamepad1.right_stick_x, gamepad1.right_stick_y), moveSpeed, gamepad1.left_stick_x * rotateSpeed);
            }

/*
            if (gamepad1.a) {
                lockRotation = !lockRotation;
                sleep(100);
            }

            if (lastLockRotation != lockRotation) {
                lastLockRotation = lockRotation;
                if (lockRotation) {
                    targetRotation = robot.GetRotation();
                }
            }
*/


            //rotate lunchbox up with the up dpad
            lunchboxRot -= gamepad1.dpad_up ? deltaTime.seconds() * 0.5 : 0;
            //rotate lunchbox down with the down dpad
            lunchboxRot += gamepad1.dpad_down ? deltaTime.seconds() * 0.5 : 0;
            lunchboxRot = bMath.Clamp(lunchboxRot, 0, 1);
            robot.lunchbox.setPosition(lunchboxRot);


            //ARM CONTROLS

            //press the X button to put the grabber in "idle" position
            if (gamepad2.x && !xButton2Check) {
                idle = true;
            }
            xButton2Check = gamepad2.x;

            //press the B button to open or close grabber
            if (gamepad2.b && !bButton2Check) {
                if (idle){
                    grab = false; //if it's in idle, pressing "B" should open it
                }
                else {
                    grab = !grab;
                }
                idle = false;
            }
            bButton2Check = gamepad2.b;


            if (idle) {
                robot.arm.SetGripState(RobotArm.GripState.IDLE, gripAngle/180);
            } else {
                if (grab) {
                    robot.arm.SetGripState(RobotArm.GripState.CLOSED, gripAngle/180);
                } else {
                    robot.arm.SetGripState(RobotArm.GripState.OPEN, gripAngle/180);
                }

            }

            //press a button to make the gripper point down
            if (gamepad2.a && !aButton2Check){
                pointDown = true;
            }
            aButton2Check = gamepad2.a;

            if (pointDown){
                gripAngle=90-robot.arm.thetaAngle(177,76.9,135,((double)robot.arm.rotation.getCurrentPosition()*0.5)/480);
            }

            //rotate gripper down with the down dpad
            if (gamepad2.dpad_down){
                gripAngle += deltaTime.seconds() * 135;
                pointDown = false;
            }

            //rotate gripper up with the up dpad
            if (gamepad2.dpad_up){
                gripAngle -=deltaTime.seconds() * 135;
                pointDown = false;
            }


            //extend arm by tapping right trigger
            extension += gamepad2.right_trigger * deltaTime.seconds();
            //retract arm by tapping left trigger
            extension -= gamepad2.left_trigger * deltaTime.seconds();


            aTad = gamepad2.y ? 1 : 0;
            extension = bMath.Clamp(extension, 0, 1);
            armAngle = bMath.Clamp(armAngle, 0, 1);
            gripAngle= bMath.Clamp(gripAngle,0,180);
            raiseSpeed = bMath.Clamp(gamepad2.left_stick_y + aTad, 0, 1);
            robot.arm.SetArmState(extension, raiseSpeed, 1);


            telemetry.addData("Rotation Locked ", lockRotation);
            telemetry.addData("", "");
            telemetry.addData("Current Rotation ", robot.GetRotation());
            telemetry.addData("Current Target Rotation", targetRotation);
            telemetry.addData("Current Lunchbox", lunchboxRot);
            telemetry.update();

            deltaTime.reset();

        }
        robot.Stop();
    }
}