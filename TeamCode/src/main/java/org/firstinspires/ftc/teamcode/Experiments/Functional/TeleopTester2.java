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
    double targetRotationOffset;

    boolean grab = false;
    boolean bButton2Check = false;

    double extension = 0;
    double armAngle = 0;
    double gripAngle = 180;


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

            targetRotation += gamepad1.left_stick_x;
//            targetRotation = bMath.Loop(targetRotation, 360);
            //shouldn't we use deltatime?


            if (gamepad1.a && !aButton1Check){
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
            //ARM CONTROLS

            //press the B button to change the state of grab if it's state is different to it's previous state
            if (gamepad2.b && !bButton2Check) {
                grab =!grab;
            }

            bButton2Check = gamepad2.b;

            if (grab) {
                robot.arm.SetGripState(RobotArm.GripState.CLOSED, (90-gripAngle)/180);
            } else{
                robot.arm.SetGripState(RobotArm.GripState.OPEN, (90-gripAngle)/180);
            }






            //extend arm by tapping right trigger
            extension += gamepad2.right_trigger * deltaTime.seconds();
            //retract arm by tapping left trigger
            extension -= gamepad2.left_trigger * deltaTime.seconds();
            //rotate gripRotator up with the up dpad
            gripAngle -= gamepad2.dpad_up ? deltaTime.seconds() *90 : 0;
            //rotate down with the down dpad
            gripAngle += gamepad2.dpad_down ? deltaTime.seconds() *90 : 0;

            extension = bMath.Clamp(extension, 0, 1);
            armAngle = bMath.Clamp(armAngle, 0, 1);
            robot.arm.SetArmState(extension, gamepad2.left_stick_y, 1);


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