package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Teleop", group = "Sensor")
public class TeleopTester extends LinearOpMode {

    Robot robot = new Robot();

    DeltaTime deltaTime = new DeltaTime();

    double targetRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        targetRotation = robot.GetRotation();

        while (opModeIsActive()) {
            deltaTime.Start();

            //Rotate 180 degrees in one seconds
            targetRotation += 180 * gamepad1.right_stick_x * deltaTime.deltaTime();

            //Move via joystick and maintain target rotation, rotation might not work
            robot.MoveComplex(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.a ? 1 : 0.1, targetRotation);

            deltaTime.Stop();
        }

    }
}