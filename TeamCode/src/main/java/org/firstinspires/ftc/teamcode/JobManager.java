
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helpers.DeltaTime;

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

    //This aJob is used in all other jobs that use tensor flow
    public static TensorFlowaJob tensorFlowaJob = new TensorFlowaJob();

    public WallTrack wallTrackJob = new WallTrack();


    //Inits all job's
    //The idea here is that all jobs have there init stuff pre loaded before they need to run; this is to avoid waiting for 10 seconds of our auto period on loading tensorflow
    public void initAll(LinearOpMode op) {
        //Make sure to init TF first cause everything uses it
        tensorFlowaJob.Init(op);

        //Setup the skystone job
        findSkystoneJob.Init(op);

        //Doesnt do anything
        wallTrackJob.Init(op);
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
//Cool things: Has a built in Robot, has some motor functions for stopping/starting
class NavigationJob extends Job {

    Robot robot;

    //Assigns the robot, make sure to do this before starting
    public void SetRobot(Robot bot) {
        robot = bot;
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

    //Called when the JobManager is set up, this should have all init stuffs that we don't wanna run durring the automode
    public void Init(LinearOpMode op) {

    }


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

//aJob version of NavigationJob, has robot access and motor commands. DO NOT USE IT DO NOT DO THE WORKING VERY WELL
class aJobNavigation extends aJob implements Runnable {
    Robot robot = new Robot();


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