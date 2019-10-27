package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//10.11.19: Moves towards any skystone it sees!
@Autonomous(name = "Skystone", group = "Auto Testing")
public class SkystoneAutoTest extends LinearOpMode {

//    HardwareInfinityMec robot = new HardwareInfinityMec();

//    public TensorFlow_bThread tensorFlowThread = new TensorFlow_bThread();

    public JobManager jobs = new JobManager();

    @Override
    public void runOpMode() {
        jobs.initAll(this);

        //Start TF
        jobs.tensorFlowaJob.Start(this);

        waitForStart();


//        jobs.wallTrackJob.StartValues(25,5,25,new WallTrack.SensorTriplet());
//        jobs.wallTrackJob.Start(this);

        //Start by lining up with the skystones
        jobs.findSkystoneJob.Start(this);
        telemetry.addData("stopped", "");
        telemetry.update();
    }

}
