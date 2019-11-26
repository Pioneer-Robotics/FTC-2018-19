package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "TestingOpMode4", group = "Sensor")
public class TestingOpMode4 extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime deltaTime = new ElapsedTime();


    int last_bl;

    int last_br;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.RotatePID(90, 1, 100);

                sleep(500);
                        }
        }


        robot.Stop();

    }
}