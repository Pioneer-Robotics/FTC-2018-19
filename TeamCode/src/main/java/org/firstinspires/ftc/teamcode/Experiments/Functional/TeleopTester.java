package org.firstinspires.ftc.teamcode.Experiments.Functional;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Teleop", group = "Sensor")
public class TeleopTester extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        while (opModeIsActive()) {
            //Move via joystick and maintain rotation
            robot.MoveComplex(new Double2(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.a ? 1 : 0.1, robot.GetRotation());


        }

    }
}