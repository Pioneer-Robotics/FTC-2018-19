
package org.firstinspires.ftc.teamcode;


import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

//A lovely picture of the robot
//
//        90
//       ______
//      |  F   |
//  180 |      || => 0 phone
//      |______|
//
//         270

//The idea here is that we can have a bunch of functions here, like move to sky stone or navigate to center and then call them in a modular fashion in other programs

//Might crash everything!
public class JobManager {
    public FindSkystoneJob findSkystoneJob = new FindSkystoneJob();
}

//Simple sub op mode, needs an LinearOpMode to function
class Job {

    public LinearOpMode opMode;

    public DeltaTime deltaTime = new DeltaTime();

    public boolean running = false;

    public final void Start(LinearOpMode op) {
        OnStart(op);
        running = true;
        RunLoop();
    }

    //Used to actually call Loop, don't touch dis
    final void RunLoop() {
        while (running) {
            deltaTime.Start();
            Loop();
            deltaTime.Stop();
        }
        OnStop();
    }


    //Loop for this job, override and add functionality
    public void Loop() {

    }

    //Stops the current job, don't touch
    public final void Stop() {
        running = false;
    }

    //Called when this job is complete
    public void OnStop() {

    }

    //Called when the job is first started, sets opMode
    public void OnStart(LinearOpMode op) {
        opMode = op;
    }
}

//A job that includes a robot!
class NavigationJob extends Job {

    HardwareInfinityMec robot = new HardwareInfinityMec();

    @Override
    public void OnStart(LinearOpMode op) {
        super.OnStart(op);
        robot.init(op.hardwareMap, op);
    }

    //Sets up the motors for actual encoder use
    public void PrepareMotors() {
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Stops the motors and reset encoders
    public void StopMotors() {
        robot.SetDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

class FindSkystoneJob extends NavigationJob {

    //Our wee little TF thread
    public TensorFlowThread tensorFlowThread = new TensorFlowThread();

    //current recognition
    Recognition recognition;

    //the
    double xFactor = 0;

    //Move speed, will scale in proportion to how close we are to the stone
    double moveSpeed = 0;

    //Time since we've last seen the skystone
    double lostRecognitionTimer = 0;


    @Override
    public void Loop() {
        super.Loop();

        opMode.telemetry.addData("Loop called!", "");
        moveSpeed = 0;
        recognition = tensorFlowThread.getCurrentRecognition();

        //if we can see a skystone
        if (recognition != null) {
            lostRecognitionTimer = 0;

            //Find how far left/right the skystone is relative to the camera (-1 == left, 1 == right) with a tolerance of 1/10 the screen
            xFactor = tensorFlowThread.getCurrentXFactor(recognition) > 0 ? 1 : 0;
            xFactor = tensorFlowThread.getCurrentXFactor(recognition) < 0 ? -1 : 0;

            //Lerp the move speed
            moveSpeed = bMath.MoveTowards(moveSpeed, bMath.Clamp(tensorFlowThread.getCurrentXFactor(recognition), 0.1, 1), deltaTime.deltaTime());

            opMode.telemetry.addData("XFactor ", xFactor);
            opMode.telemetry.addData("Movement Factor ", (-xFactor * 90));
            opMode.telemetry.addData("cFactor ", tensorFlowThread.getCurrentXFactor(recognition));


            //TODO: After stopping double check that we are lined up after 250ms
            //If we are lined up nicely stop the job, if not then move to be
            if (Math.abs(tensorFlowThread.getCurrentXFactor(recognition)) < 0.05) {
//Disabled for testing purposes
//Stop();
            } else {
//                bMath.Clamp(tensorFlowThread.getCurrentXFactor(recognition), 0.1, 0.5)


                //Move left or right (strafe) until we are lined up with the skystone
                robot.MoveSimple(tensorFlowThread.getCurrentXFactor(recognition) > 0 ? 180 : 0, moveSpeed);

            }

        } else {

            //Tick the timer!
            lostRecognitionTimer += deltaTime.deltaTime();


            //If we've lost sight of the stone for more than 0.25 seconds stop moving (to avoid issues while testing, ei running over my feets)
            if (lostRecognitionTimer >= 0.25) {
                opMode.telemetry.addData("Lost sight, stopping ", "");

                robot.SetPowerDouble4(new Double4(0, 0, 0, 0), 0);
            }
        }

        opMode.telemetry.update();

    }

    @Override
    public void OnStop() {
        super.OnStop();

        //Disposes of the thread
        tensorFlowThread.stopThread();

        //Stop motorz
        StopMotors();
    }

    @Override
    public void OnStart(LinearOpMode op) {
        super.OnStart(op);
        //Starts up a tensor flow thread
        tensorFlowThread.startThread(op, "Skystone", 0.75);
//        tensorFlowThread.StartTensorFlow(op, "Skystone", 0.75);
//        tensorFlowThread.start();
        PrepareMotors();
    }
}

/*
class OpJob {

    public boolean running = false;

    public void Start() {
        OnStart();
        running = true;
        RunLoop();
    }

    //Used to actually call Loop, don't touch dis
    final void RunLoop() {
        while (running) {
            Loop();
        }
        OnStop();
    }


    //Loop for this job, override and add functionality
    public void Loop() {

    }

    //Stops the current job, don't touch
    public final void Stop() {
        running = false;
    }

    //Called when this job is complete
    public void OnStop() {

    }

    //Called when the job is first started
    public void OnStart() {

    }
}
*/