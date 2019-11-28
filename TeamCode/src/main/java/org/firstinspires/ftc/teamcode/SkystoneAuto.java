package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "Skystone", group = "ftcPio")
public class SkystoneAuto extends Auto {

    public double startRotation;

    @Override
    public void runOpMode() {
        StartRobot();

        startRotation = robot.GetRotation();

        waitForStart();

        //Line up with a skystone
        SkystoneAlign(speed_low, 30, 2, 0.5, 0.15, startRotation);

        //Drive forward while adjusting heading to line up with the skystone
        DriveAtSkystone(speed_med, 45, 38, startRotation);

        //Deploy the arm and grab dat stone
        ActuateArm();

        //Roll back a wee bit
        robot.DriveByDistance(speed_med, -10);

        //Rotate based on our side to face the foundation
        if (side == feildSide.SIDE_BLUE) {
            robot.RotatePID(-90, speed_high, 100);
        } else {
            robot.RotatePID(90, speed_high, 100);
        }

        //Move towards the foundation by wall tracking along the wall
        while (robot.GetDistance(RobotWallTrack.groupID.Group0, DistanceUnit.CM) < 36) {
            if (side == feildSide.SIDE_BLUE) {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group270, speed_high, 36, walltrackingController, 35, 90, startRotation);
            } else {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group90, speed_high, 36, walltrackingController, 35, -90, startRotation);
            }
        }

        //Rotate to face the foundation
        robot.RotatePID(startRotation, speed_high, 100);

        //Deploy the arm to latch to the foundation
        ActuateArm();

        StopRobot();
    }
}
