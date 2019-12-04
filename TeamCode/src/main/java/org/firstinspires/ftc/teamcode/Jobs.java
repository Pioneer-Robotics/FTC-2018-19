
package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.Helpers.bTelemetry;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

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
class FindSkystoneJob extends NavigationJob {

    public double rotationLockAngle = 0;

    //New tensor flow aJOB (wip)
    public TensorFlowaJob tensorFlowaJob = new TensorFlowaJob();

    //current recognition
    Recognition recognition;

    //the
    double xFactor = 0;

    //Move speed, will scale in proportion to how close we are to the stone
    double moveSpeed = 0;

    //Time since we've last seen the skystone
    double lostRecognitionTimer = 0;

    //After the robot is aligned it will set settle to true, wait, and then recheck its alignment (to see if the bots shifted at all)
    boolean settled = false;

    double settledTimer = 0;

    @Override
    public void Loop() {
        super.Loop();

        opMode.telemetry.addData("Loop called!", "");
        moveSpeed = 0;
        recognition = tensorFlowaJob.getCurrentRecognition();

        //if we can see a skystone
        if (recognition != null) {
            lostRecognitionTimer = 0;

            //Find how far left/right the skystone is relative to the camera (-1 == left, 1 == right) with a tolerance of 1/10 the screen
            xFactor = tensorFlowaJob.getCurrentXFactor(recognition) > 0 ? 1 : 0;
            xFactor = tensorFlowaJob.getCurrentXFactor(recognition) < 0 ? -1 : 0;

            //Lerp the move speed
//            moveSpeed = bMath.MoveTowards(moveSpeed, bMath.Clamp(Math.abs(tensorFlowaJob.getCurrentXFactor(recognition)), 0, 0.2), deltaTime.seconds());
            //Move left or right (positive/negative) relative to the wall that is at angle 180
            opMode.telemetry.addData("Skystone found, aligning", "");
            robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.2, 15, 1, 25, -xFactor * 90);


        } else {
//            lostRecognitionTimer += deltaTime.seconds();
//            if (lostRecognitionTimer < 0.2) {
            opMode.telemetry.addData("Skystone not found", "");

            robot.wallTrack.MoveAlongWallSimple(RobotWallTrack.groupID.Group180, 0.2, 15, 5, 15, -90);
//            }
        }
        opMode.telemetry.addData("DT ", deltaTime.seconds());
        opMode.telemetry.update();

    }

    @Override
    public void OnStop() {
        super.OnStop();


        //Disposes of the thread
        tensorFlowaJob.Stop();
        robot.Stop();
        //Stop motorz
        StopMotors();
    }

    @Override
    public void OnStart(LinearOpMode op) {
        super.OnStart(op);


        //Starts up a tensor flow thread
//        tensorFlowThread.StartTensorFlow(op, "Skystone", 0.75);
//        tensorFlowThread.start();
        PrepareMotors();
    }

    @Override
    public void Init(LinearOpMode op) {
        super.Init(op);

        tensorFlowaJob = JobManager.tensorFlowaJob;

        tensorFlowaJob.Start(op);
    }
}


//This is the thread tensor flow job, we should only need one of these and it should be stored in the JobManager
class TensorFlowaJob extends aJob implements Runnable {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static String TARGET_LABEL = "";
    private static final String SKYSTONE_LABEL = "Skystone";
    private static final String STONE_LABEL = "Stone";

    private static final String VUFORIA_KEY = "AQMfl/L/////AAABmTblKFiFfUXdnoB7Ocz4UQNgHjSNJaBwlaDm9EpX0UI5ISx2EH+5IoEmxxd/FG8c31He17kM5vtS0jyAoD2ev5mXBiITmx4N8AduU/iAw/XMC5MiEB1YBgw5oSO1qd4jvCOgbzy/HcOpN3KoVVnYqKhTLc8n6/IIFGy+qyF7b8WkzscJpybOSAT5wtaZumdBu0K3lHV6n+fqGJDMvkQ5xrCS6HiBtpZScAoekd7iP3IxUik2rMFq5hqMsOYW+qlxKp0cj+x4K9CIOYEP4xZsCBt66UxtDSiNqaiC1DyONtFz4oHJf/4J5aYRjMNwC2BpsVJ/R91WIcC0H0dpP9gtL/09J0bIMjm3plo+ac+OM0H3";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    Recognition currentRecognition;

    static List<Recognition> recognitions;

    ElapsedTime debuggingDeltaTime = new ElapsedTime();

    //<editor-fold desc="External Calls?">
    public Recognition getCurrentRecognition() {
        debuggingDeltaTime.reset();

        //Iterate through all recognitions and return the TARGET_LABEL
        for (Recognition recognition : recognitions) {

            //Only set the one that we want
            if (recognition.getLabel() == SKYSTONE_LABEL) {
                return recognition;
            }
        }
        opMode.telemetry.addData("Get Recognition Time", debuggingDeltaTime.seconds());

        return null;
    }

    //The a number between -1 and 1 representing how close the recognition is to the center of the camera
    public double getCurrentXFactor(Recognition recognition) {
        return (getXPosition(recognition) - (recognition.getImageWidth() / 2)) / (recognition.getImageWidth() / 2);
    }

    //Returns the average between the left bound and right bound
    public float getXPosition(Recognition recognition) {
        float factor = (recognition.getLeft() + (getWidth(recognition) / 2));
        return factor;
    }

    //Returns the current width of our recognition
    public float getWidth(Recognition recognition) {
        return recognition.getWidth();
    }

    @Override
    public void Init(LinearOpMode op) {
        super.Init(op);
        //Start up the tensor flow stuffs
        StartTensorFlow(op, "Skystone", 0.80);
    }

    @Override
    public void Loop() {
        super.Loop();
        deltaTime.reset();

        //Fetch all of TF's current recognitions
        recognitions = tfod.getRecognitions();

//        //Iterate through all recognitions and tag the TARGET_LABEL
//        for (Recognition recognition : recognitions) {
//
//            //Only set the one that we want
//            if (recognition.getLabel() == TARGET_LABEL) {
//                currentRecognition = recognition;
//            }
//        }

    }

    @Override
    public void OnStart(LinearOpMode op) {
        super.OnStart(op);

    }

    @Override
    public void OnStop() {
        super.OnStop();
        //Shut down TF once we are stopped
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //Starts TF
    //<editor-fold desc="Starting Tensorflow">
    public void StartTensorFlow(LinearOpMode opMode, String label, double minConfidence) {
        TARGET_LABEL = label;
        WebcamName camera = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        int moniterID = opMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());


        initAll(camera, moniterID, minConfidence);

        //Make sure TF is started before we boot up the thread
        if (tfod != null) {
//            tfod.setClippingMargins();
            tfod.activate();
        }
    }

    private void initAll(WebcamName camera, int tfodMonitorViewId, double minConfidence) {
        initVuforia(camera);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTensorFlow(tfodMonitorViewId, minConfidence);
        }
    }

    private void initVuforia(WebcamName camera) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = camera;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTensorFlow(int tfodMonitorViewId, double minConfidence) {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = minConfidence;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE_LABEL, SKYSTONE_LABEL);
    }
    //</editor-fold>

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