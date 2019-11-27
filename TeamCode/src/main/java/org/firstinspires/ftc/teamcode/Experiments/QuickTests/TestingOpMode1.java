package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode1", group = "Sensor")
public class TestingOpMode1 extends LinearOpMode {
    Robot robot = new Robot();

    public PID controller = new PID();

    public ElapsedTime deltaTime = new ElapsedTime();

    double targetRotation;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();
        targetRotation = robot.GetRotation();
        controller.Start(4.95, 0.06, 0.05);
        while (opModeIsActive()) {
            robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group180, 1, 20, controller, 90, 90, targetRotation);
            telemetry.update();
        }

        robot.Stop();
    }

}
