package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.bDeltaValue;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode3", group = "Debugging")
//Used to ensure that all wheels are set up correctly
public class TestingOpMode3 extends LinearOpMode {

    Robot robot = new Robot();


    public ElapsedTime deltaTime = new ElapsedTime();


    double targetRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();
        targetRotation = robot.GetRotation();
        while (opModeIsActive()) {
            robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.2, 20, 3, 90, 90, targetRotation);
            telemetry.update();
        }

        robot.Stop();
    }

}
