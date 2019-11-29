package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//WIP WIP WIP
//Going to be used when placing the robot on the field to ensure consistent placement
public class RobotSetUpHelper {


    public Rev2mDistanceSensor sensor90A;
    public Rev2mDistanceSensor sensor90B;

    public Rev2mDistanceSensor sensor180A;
    public Rev2mDistanceSensor sensor180B;


    public Rev2mDistanceSensor sensor270A;
    public Rev2mDistanceSensor sensor270B;


    public void Update(Robot robot, Telemetry telemetry) {
        double wallAngle = robot.wallTrack.sensorIDGroupPairs.get(RobotWallTrack.groupID.Group90).getWallAngle();

        telemetry.addData("Lined up with wall ", wallAngle < 5);
    }
}
