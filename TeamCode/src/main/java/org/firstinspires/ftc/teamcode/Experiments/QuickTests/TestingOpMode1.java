package org.firstinspires.ftc.teamcode.Experiments.QuickTests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "TestingOpMode1", group = "Sensor")
public class TestingOpMode1 extends LinearOpMode {

    Robot robot = new Robot();

    public DeltaTime deltaTime = new DeltaTime();

    int position;
    int lastPosition;

    int position2;
    int lastPosition2;

    double speed;
    double speed2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.driveManager.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveManager.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            deltaTime.Start();

            robot.driveManager.backRight.setPower(0.5);
            position = robot.driveManager.frontRight.motor.getCurrentPosition();
            position2 = robot.driveManager.backRight.motor.getCurrentPosition();

            speed = (position - lastPosition) / deltaTime.deltaTime();
            speed2 = (position2 - lastPosition2) / deltaTime.deltaTime();


            lastPosition = position;
            lastPosition2 = position2;

            telemetry.addData("Front Right : ", speed);
            telemetry.addData("Back Right : ", speed2);
            telemetry.addData("delta Time: ", deltaTime.deltaTime());

            telemetry.update();

            deltaTime.Stop();
        }
    }

}
