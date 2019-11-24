package org.firstinspires.ftc.teamcode;


import android.renderscript.Double2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.JobManager;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

@Autonomous(name = "SkystoneALignTest", group = "Sensor")
public class SkyStoneAlignTest extends LinearOpMode {


    //This robot is the one used for all jobs!
    Robot robot = new Robot();

    public double lockedRotation;

    double speed;
    Double2 heading;

//    public TensorFlow_bThread tensorFlowThread = new TensorFlow_bThread();

    public JobManager jobs = new JobManager();

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
        print("Status: Ready");

        //Wait for the driver to start the op mode
        waitForStart();

        Recognition recognition = null;

        lockedRotation = robot.GetRotation();

        //Look for the skystone
        while (opModeIsActive()) {

            recognition = jobs.tensorFlowaJob.getCurrentRecognition();

            if (recognition != null) {
                telemetry.addData("XFactor ", jobs.tensorFlowaJob.getCurrentXFactor(recognition));
                heading = new Double2(jobs.tensorFlowaJob.getCurrentXFactor(recognition) * 0.5, 0);
                robot.wallTrack.MoveAlongWallComplex(RobotWallTrack.groupID.Group180, jobs.tensorFlowaJob.getCurrentXFactor(recognition) * 0.5, 25, 5, 25, -90, lockedRotation);
            } else {
                telemetry.addData("No SkyStone", "");
                heading = new Double2(0, 0);
            }

//            robot.MoveComplex(heading, 0.35, robot.GetRotation() - lockedRotation);
            telemetry.update();
        }

        robot.Stop();
    }


    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

}