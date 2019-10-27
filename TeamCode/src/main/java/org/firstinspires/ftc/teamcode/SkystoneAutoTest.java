package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//10.11.19: Moves towards any skystone it sees!
@Autonomous(name = "Skystone", group = "Auto Testing")
public class SkystoneAutoTest extends LinearOpMode {

//    Robot robot = new Robot();

//    public TensorFlow_bThread tensorFlowThread = new TensorFlow_bThread();

    public JobManager jobs = new JobManager();

    @Override
    public void runOpMode() {

        telemetry.addData("Status: ", "Running initiating all jobs.");

        print("Status: Initiating all jobs.");


        jobs.initAll(this);

        print("Status: Starting TensorFlow Thread.");


        //Start the TF thread after it's inited
        jobs.tensorFlowaJob.Start(this);

        print("Status: Awaiting start. You are cleared for release.");

        waitForStart();

        print("Status: Mission started");


//        jobs.wallTrackJob.StartValues(25,5,25,new WallTrack.SensorTriplet());
//        jobs.wallTrackJob.Start(this);

        print("Status: Searching for Skystone.");

        //Start by lining up with the skystones
        jobs.findSkystoneJob.Start(this);

        print("Status: Skystone found.");


        telemetry.addData("stopped", "");
        telemetry.update();
    }

    //Sends the 'message' to telemetry and updates it, mostly for C#-ness
    void print(String message) {
        telemetry.addData("", message);
        telemetry.update();
    }

}
