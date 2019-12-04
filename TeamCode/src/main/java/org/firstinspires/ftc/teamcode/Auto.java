package org.firstinspires.ftc.teamcode;

import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

public class Auto extends LinearOpMode {

    Robot robot = new Robot();

    public JobManager jobs = new JobManager();

    public double speed_low = 0.15;
    public double speed_med = 0.35;
    public double speed_high = 0.85;

    public PID walltrackingController = new PID();

    //What side we are playing on, based on the bridge colors

    public FieldSide side;

    public enum FieldSide {
        SIDE_BLUE,
        SIDE_RED
    }

    @Override
    public void runOpMode() {

    }

    public void StartRobot() {

        print("Status: Initiating robot.");

        //init the bot hardware! This sets up the static references for the bot as well so make sure to run this before any other code
        robot.init(hardwareMap, this);

        print("Status: Initiating all jobs.");

        //Init's all of jobs we can use in the OpMode. Right now the job system isnt super useful so just use Tfod jobs
        jobs.initAll(this);

        print("Status: Starting TensorFlow Thread.");
        //Start the TF thread after it's init
        jobs.tensorFlowaJob.Start(this);

        print("Status: Determining current play side");

        //If we are closest to the 90 degree side we know were playing on the BLUE side
        if (robot.GetDistance(RobotWallTrack.groupID.Group90, DistanceUnit.CM) < robot.GetDistance(RobotWallTrack.groupID.Group270, DistanceUnit.CM)) {
            side = FieldSide.SIDE_BLUE;
        } else {
            side = FieldSide.SIDE_RED;
        }

        robot.arm.SetGripState(0.25, 0.9);


        print("Status: Awaiting start.");
    }

    public void StopRobot() {
        robot.SetPowerDouble4(0, 0, 0, 0, 0);
        jobs.stopAll();
        robot.Stop();
    }

    /**
     * @param moveSpeed             How fast we will be moving side to side (lower speeds recommended)
     * @param wallDistance          How far away from the rear wall we should try to be, in CM
     * @param correctionCoefficient How sensitive we are to the position of the skystone, lower values recommended
     * @param lockTime              How long we must be within lockThreshold, should be less than one second
     * @param lockThreshold         How close the skystone needs to be to the center of the camera in order for us to stop (0.1 - 0.3)
     */
    //This loop uses the rear sensors to line up with a skystone
    public void SkystoneAlign(double moveSpeed, double wallDistance, double correctionCoefficient, double lockTime, double lockThreshold, double startRotation) {
        ResetWallPID();
        ElapsedTime deltaTime = new ElapsedTime();

        double onSkystoneTime = 0;

        while (opModeIsActive()) {
            Recognition skystone = jobs.tensorFlowaJob.getCurrentRecognition();

            if (skystone != null) {
                if (Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)) < lockThreshold) {
                    StopAndMaintainRotation(startRotation);
                    onSkystoneTime += deltaTime.seconds();
                } else {
                    robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group180, bMath.Clamp(Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)) * correctionCoefficient, -moveSpeed, moveSpeed), wallDistance, walltrackingController, 45, jobs.tensorFlowaJob.getCurrentXFactor(skystone) > 0 ? -90 : 90, startRotation);
                }


                telemetry.addData("Skystones distance ", Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)));
                telemetry.addData("Rotation Goal ", startRotation);
                telemetry.addData("Current Rotation     ", robot.GetRotation());
                telemetry.addData("Rotation Factor ", robot.GetRotation() - startRotation);
            } else {
                robot.MoveComplex(new Double2(0, 0), speed_med, robot.GetRotation() - startRotation);
            }

            if (onSkystoneTime > lockTime) {
                break;
            }

            telemetry.update();
            deltaTime.reset();
        }
    }

    /**
     * @param moveSpeed          How fast we will be moving forward (lower speeds recommended)
     * @param maxCorrectionAngle How fast we will correct our heading, in degrees
     * @param wallStopDistance   At what distance from the back wall will we stop moving
     */
    //This drives at a skystone while correcting itself
    public void DriveAtSkystone(double moveSpeed, double maxCorrectionAngle, double wallStopDistance, double startRotation) {
        ResetWallPID();
        while (opModeIsActive()) {
            Recognition skystone = jobs.tensorFlowaJob.getCurrentRecognition();
            if (skystone == null) {
                StopAndMaintainRotation(startRotation);
            } else {
                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, moveSpeed, 180 + bMath.Lerp(-maxCorrectionAngle, maxCorrectionAngle, (jobs.tensorFlowaJob.getCurrentXFactor(skystone) + 1) / 2), startRotation);
                if (robot.GetDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM) > wallStopDistance) {
                    StopMovement();
                    break;
                }
            }
            telemetry.update();
        }
    }

    //Freezes the robots movement but continues to seek its correct rotation
    public void StopAndMaintainRotation(double rotation) {
        robot.MoveComplex(new Double2(0, 0), 1, robot.GetRotation() - rotation);
    }

    //Freezes the robots movement but continues to seek its correct rotation
    public void StopMovement() {
        robot.SetPowerDouble4(0, 0, 0, 0, 0);
    }

    public void ResetWallPID() {
        walltrackingController.Start(4.95, 0.0, 0.1);
    }


    public void LostRecognition() {

    }

    public void InitArm() {
        robot.arm.SetArmState(0.2, 0, 1, 1);

    }

    public void ActuateArm() {
    }

    public void DriveToMidField(double moveSpeed, double distanceToWall) {
        ResetWallPID();
        if (side == FieldSide.SIDE_BLUE) {
            robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group270, moveSpeed, distanceToWall, walltrackingController, 45, 90, -90);
        }
    }


    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

}
