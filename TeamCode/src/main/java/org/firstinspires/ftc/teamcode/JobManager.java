
package org.firstinspires.ftc.teamcode;


import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
//These are all of the base classes that are used in Jobs, feel free to right your own!
//TODO: Async Jobs


//Holds all of the current jobs for using in other scripts
public class JobManager {
    public FindSkystoneJob findSkystoneJob = new FindSkystoneJob();

    //Inits all job's
    public void initAll(LinearOpMode op) {
        findSkystoneJob.Init(op);
    }
}

//Simple sub op mode, needs an LinearOpMode to function
//Cool things: Has a built in deltaTime
class Job {

    public LinearOpMode opMode;

    public DeltaTime deltaTime = new DeltaTime();

    public boolean running = false;

    //Called when the JobManager is set up, this should have all init stuffs that we don't wanna run durring the automode
    public void Init(LinearOpMode op) {

    }


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

            if (!opMode.opModeIsActive()) {
                running = false;
            }
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
//Cool things: Has a built in HardwareInfinityMec, has some motor functions for stopping/starting
class NavigationJob extends Job {

    HardwareInfinityMec robot = new HardwareInfinityMec();

    @Override
    public void Init(LinearOpMode op) {
        super.Init(op);
        robot.init(op.hardwareMap, op);
    }

    @Override
    public void OnStart(LinearOpMode op) {
        super.OnStart(op);
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

//Async job, similar syntax to Job but its threaded all fancy like
class aJob implements Runnable {

    public LinearOpMode opMode;

    public DeltaTime deltaTime = new DeltaTime();

    public boolean running = false;

    public final void Start(LinearOpMode op) {

        //Passes op to OnStart
        OnStart(op);


        //Creates a new threaded instance and starts it
        //Calls run but in a fancy way
        new Thread(this).start();
    }

    public void run() {
        running = true;
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

//aJob version of NavigationJob, has robot access and motor commands
class aJobNavigation extends aJob implements Runnable {
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