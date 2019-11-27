package org.firstinspires.ftc.teamcode;

import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helpers.PID;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

//10.11.19: Moves towards any skystone it sees!
@Autonomous(name = "Skystone", group = "Auto Testing")
public class SkystoneAutoTest extends LinearOpMode {

    //This robot is the one used for all jobs!
    Robot robot = new Robot();

    public double startRotation;

    public JobManager jobs = new JobManager();

    boolean locatedSkystone = false;
    boolean alignedWithSkyStone = false;
    boolean nearSkystone = false;

    public double speed_low = 0.15;
    public double speed_med = 0.35;
    public double speed_high = 0.85;

    public PID walltrackingController = new PID();

    @Override
    public void runOpMode() {

        print("Status: Initiating robot.");

        //init the bot! This sets up the static references for the bot as well so make sure to run this early
        robot.init(hardwareMap, this);

        print("Status: Initiating all jobs.");

        //Init's all of jobs we can use in the OpMode
        jobs.initAll(this);

        print("Status: Starting TensorFlow Thread.");

        //Start the TF thread after it's init
        jobs.tensorFlowaJob.Start(this);

        print("Status: Awaiting start. You are cleared for release.");

        walltrackingController.Start(4.95, 0, 0.1);

        //Wait for the driver to start the op mode
        waitForStart();

        startRotation = robot.GetRotation();

        print("Status: Mission started");

//        jobs.wallTrackJob.StartValues(25,5,25,new WallTrack.SensorGroup());
//        jobs.wallTrackJob.Start(this);


        while (opModeIsActive()) {
            Recognition skystone = jobs.tensorFlowaJob.currentRecognition;
            if (skystone == null) {
                robot.MoveComplex(new Double2(0, 0), speed_med, startRotation);
            } else {
                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, speed_med, bMath.Lerp(-90, 90, jobs.tensorFlowaJob.getCurrentXFactor(skystone) - 1), startRotation);
            }
        }


//        robot.DriveByDistance(speed_med, 20);
//
//        while (opModeIsActive()) {
//            Recognition skystone = jobs.tensorFlowaJob.currentRecognition;
//            if (skystone == null) {
//                robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group180, speed_med, 45, walltrackingController, 45, 90, startRotation);
//            } else {
//                break;
//            }
//
//            telemetry.addData("Looking for skystones", skystone != null ? "Found it!" : "Where is it!");
//            telemetry.update();
//        }
//
//        robot.SetPowerDouble4(0, 0, 0, 0, 0);
//        sleep(250);
//        walltrackingController.Start(4.95, 0.06, 0.05);
//
//        while (opModeIsActive()) {
//            Recognition skystone = jobs.tensorFlowaJob.currentRecognition;
//
//            if (skystone != null) {
//                if (Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)) < 0.1) {
//                    //Hold still while maintaining our start rotation
//                    robot.MoveComplex(new Double2(0, 0), speed_med, robot.GetRotation() - startRotation);
//                } else {
//                    robot.wallTrack.MoveAlongWallComplexPID(RobotWallTrack.groupID.Group180, bMath.Clamp(Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)) * 5 + 0.1, -speed_low, speed_low), 45, walltrackingController, 45, jobs.tensorFlowaJob.getCurrentXFactor(skystone) > 0 ? -90 : 90, startRotation);
//                }
//
//
//                telemetry.addData("Skystones distance ", Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)));
//                telemetry.addData("Rotation Goal ", startRotation);
//                telemetry.addData("Current Rotation     ", robot.GetRotation());
//                telemetry.addData("Rotation Factor ", robot.GetRotation() - startRotation);
//            } else {
//                telemetry.addData("Lost Stone! ", "");
//                robot.MoveComplex(new Double2(0, 0), speed_med, robot.GetRotation() - startRotation);
//            }
//            telemetry.update();
//        }


//        while (opModeIsActive()) {
//            Recognition skystone = jobs.tensorFlowaJob.currentRecognition;
//
//            if (!alignedWithSkyStone) {
//                telemetry.addData("Not aligned with skystone", "");
//
//                if (skystone == null) {
//                    telemetry.addData("No skystone", "");
//
//                    if (locatedSkystone) {
//                        telemetry.addData("No skystone and Stopping", "");
//                        robot.SetPowerDouble4(0, 0, 0, 0, 0);
//                    } else {
//                        telemetry.addData("No skystone and Seeking", "");
//                        robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.25, 15, 1, 15, 90, startRotation);
//                    }
//                } else {
//                    telemetry.addData("Skystone and aligning", jobs.tensorFlowaJob.getCurrentXFactor(skystone));
//                    locatedSkystone = true;
//                    robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.25 * jobs.tensorFlowaJob.getCurrentXFactor(skystone), 15, 1, 15, -90, startRotation);
//
//                    if (Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)) < 0.05) {
//                        alignedWithSkyStone = true;
//                    }
//                }
//            }
//
//            if (alignedWithSkyStone && !nearSkystone) {
//                telemetry.addData("Aligned with skystone but not near", "");
//                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.5, 0, startRotation);
//
//                if (robot.WallDistance(RobotWallTrack.groupID.Group180, DistanceUnit.CM) > 50) {
//                    nearSkystone = true;
//                }
//            }
//
//            if (nearSkystone) {
//                telemetry.addData("Near skysonte", "");
//                robot.SetPowerDouble4(0, 0, 0, 0, 0);
//
//            }
//            telemetry.update();
//        }

        jobs.stopAll();

        robot.Stop();

    }

    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

}
