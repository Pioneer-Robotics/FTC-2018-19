package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

//10.11.19: Moves towards any skystone it sees!
@Autonomous(name = "Skystone", group = "Auto Testing")
public class SkystoneAutoTest2 extends LinearOpMode {

    //This robot is the one used for all jobs!
    Robot robot = new Robot();

    public double rotation;

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

        print("Status: Awaiting start.");

        //Wait for the driver to start the op mode
        waitForStart();

        print("Status: Mission started");


        print("Status: Searching for Skystone.");

        //Look for the skystone
        while (!jobs.tensorFlowaJob.hasRecognition()) {
            robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.25, 25, 1, 25, -90, 0);
        }

        //Start by lining up with the skystones
        jobs.findSkystoneJob.Start(this);

        print("Status: Skystone found.");

        robot.RotatePID(-90, 1, 100);

    }

    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

}
