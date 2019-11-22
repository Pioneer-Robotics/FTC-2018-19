package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "TestingOpMode2", group = "Sensor")
public class TestingOpMode2 extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime deltaTime = new ElapsedTime();


    int last_bl;

    int last_br;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();


        robot.MoveSimple(0, 0.1);
        sleep(2500);


        robot.MoveSimple(90, 0.1);
        sleep(2500);


        robot.MoveSimple(180, 0.1);
        sleep(2500);


        robot.MoveSimple(270, 0.1);
        sleep(2500);


        robot.Stop();

    }
}