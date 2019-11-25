package org.firstinspires.ftc.teamcode;

import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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

        //Wait for the driver to start the op mode
        waitForStart();

        startRotation = robot.GetRotation();

        print("Status: Mission started");

//        jobs.wallTrackJob.StartValues(25,5,25,new WallTrack.SensorGroup());
//        jobs.wallTrackJob.Start(this);

        print("Status: Searching for Skystone.");


        while (opModeIsActive()) {
            Recognition skystone = jobs.tensorFlowaJob.currentRecognition;

            if (!alignedWithSkyStone) {
                if (skystone == null) {
                    if (locatedSkystone) {
                        robot.SetPowerDouble4(0, 0, 0, 0, 0);
                    } else {
                        robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.5, 15, 5, 45, -90, startRotation);
                    }
                } else {
                    locatedSkystone = true;
                    robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.25 * jobs.tensorFlowaJob.getCurrentXFactor(skystone), 15, 5, 45, -90, startRotation);

                    if (Math.abs(jobs.tensorFlowaJob.getCurrentXFactor(skystone)) < 0.05) {
                        alignedWithSkyStone = true;
                    }
                }
            }

            if (alignedWithSkyStone && !nearSkystone) {
                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, 0.5, 0, startRotation);

                if (robot.WallDistance(RobotWallTrack.groupID.Group0, DistanceUnit.CM) < 10) {
                    nearSkystone = true;
                }
            }

            if (nearSkystone) {


            }
        }
    }

    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

}
