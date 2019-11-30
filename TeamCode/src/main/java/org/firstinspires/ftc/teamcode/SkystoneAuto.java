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
        //A lockThreshold of .25 will get is within 19.5 degrees of the stone
        SkystoneAlign(speed_low, 45, 2, 0.5, 0.25, startRotation);

        //Drive forward while adjusting heading to line up with the skystone
        DriveAtSkystone(speed_med, 25, 35, startRotation);

        //Deploy the arm and grab dat stone
        ActuateArm();

        //Roll back a wee bit
        robot.DriveByDistance(speed_med, -5);

        //Rotate based on our side to face the foundation
        if (side == FieldSide.SIDE_BLUE) {
            robot.RotatePID(startRotation - 90, speed_med, 100000);
        } else {
            robot.RotatePID(startRotation + 90, speed_med, 10000);
        }


        //Move towards the foundation by wall tracking along the wall
        ResetWallPID();
        while (opModeIsActive() && robot.GetDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM) < 150) {
            telemetry.addData("Attempting to wall track", "");
            telemetry.addData("Stop distance", robot.GetDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM));
            telemetry.update();
            if (side == FieldSide.SIDE_BLUE) {

                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group270, speed_med, 36, walltrackingController, 35, 90, robot.GetRotation());
            } else {
                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group90, speed_med, 36, walltrackingController, 35, -90, robot.GetRotation());

            }
        }


        //Rotate to face the foundation
        robot.RotatePID(0, speed_high, 10000);

        //Deploy the arm to latch to the foundation
        ActuateArm();

        StopRobot();
    }
}
