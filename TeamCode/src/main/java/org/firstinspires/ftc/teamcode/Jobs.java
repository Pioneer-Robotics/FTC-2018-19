
package org.firstinspires.ftc.teamcode;


import android.renderscript.Double4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

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
            moveSpeed = bMath.MoveTowards(moveSpeed, bMath.Clamp(Math.abs(tensorFlowaJob.getCurrentXFactor(recognition)), 0.1, 1), deltaTime.deltaTime());

            opMode.telemetry.addData("XFactor ", xFactor);
            opMode.telemetry.addData("Movement Factor ", tensorFlowaJob.getCurrentXFactor(recognition) > 0 ? 180 : 0);
            opMode.telemetry.addData("cFactor ", tensorFlowaJob.getCurrentXFactor(recognition));


            //TODO: After stopping double check that we are lined up after 250ms
            //If we are lined up nicely stop the job, if not then move to be
            if (Math.abs(tensorFlowaJob.getCurrentXFactor(recognition)) < 0.1) {

                if (!settled) {
                    settledTimer += deltaTime.deltaTime();
                    if (settledTimer > 0.25) {
                        settled = true;
                    }
                } else {
//If we've settled then stop this job
                    Stop();

                }
//Stop();
            } else {
//                bMath.Clamp(tensorFlowThread.getCurrentXFactor(recognition), 0.1, 0.5)


                //Move left or right (strafe) until we are lined up with the skystone
                robot.MoveSimple(tensorFlowaJob.getCurrentXFactor(recognition) > 0 ? 180 : 0, moveSpeed);

            }

        } else {

            //Tick the timer!
            lostRecognitionTimer += deltaTime.deltaTime();


            //If we've lost sight of the stone for more than 0.75 seconds stop moving (to avoid issues while testing, ei running over my feets)
            if (lostRecognitionTimer >= 0.75) {
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
        tensorFlowaJob = JobManager.tensorFlowaJob;

        tensorFlowaJob.Start(op);
    }
}


//This is the thread tensor flow job, we should only need one of these and it should be stored in the JobManager
class TensorFlowaJob extends aJob implements Runnable {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static String TARGET_LABEL = "";

    private static final String VUFORIA_KEY = "AQMfl/L/////AAABmTblKFiFfUXdnoB7Ocz4UQNgHjSNJaBwlaDm9EpX0UI5ISx2EH+5IoEmxxd/FG8c31He17kM5vtS0jyAoD2ev5mXBiITmx4N8AduU/iAw/XMC5MiEB1YBgw5oSO1qd4jvCOgbzy/HcOpN3KoVVnYqKhTLc8n6/IIFGy+qyF7b8WkzscJpybOSAT5wtaZumdBu0K3lHV6n+fqGJDMvkQ5xrCS6HiBtpZScAoekd7iP3IxUik2rMFq5hqMsOYW+qlxKp0cj+x4K9CIOYEP4xZsCBt66UxtDSiNqaiC1DyONtFz4oHJf/4J5aYRjMNwC2BpsVJ/R91WIcC0H0dpP9gtL/09J0bIMjm3plo+ac+OM0H3";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    Recognition currentRecognition;

    List<Recognition> recognitions;

    //<editor-fold desc="External Calls?">
    public Recognition getCurrentRecognition() {
        return currentRecognition;
    }

    public Boolean hasRecognition() {
        return currentRecognition != null;
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
        StartTensorFlow(op, "Skystone", 0.85);
    }

    @Override
    public void Loop() {
        super.Loop();


        //Fetch all of TF's current recognitions
        recognitions = tfod.getRecognitions();

        //Clear our last recognition
        currentRecognition = null;

        //Iterate through all recognitions and tag the TARGET_LABEL
        for (Recognition recognition : recognitions) {

            //Only set the one that we want
            if (recognition.getLabel() == TARGET_LABEL) {
                currentRecognition = recognition;
            }
        }
    }

    @Override
    public void OnStart(LinearOpMode op) {
        super.OnStart(op);
        //Make sure TF is started before we boot up the thread
        if (tfod != null) {

            tfod.activate();
        }
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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, "Stone", "Skystone");
    }
    //</editor-fold>

}

//Using a specific sensor group (ID'd by the center distance sensor) wall track
class WallTrack extends NavigationJob {

    AvoidanceConfiguration avoidanceConfig = new AvoidanceConfiguration();

    SensorTriplet sensors = new SensorTriplet();

    double curDriveAngle = 0;
    double wallAngle = 0;
    double currentAngle = 0;
    double distance = 0;
    double weightedWallAngle = 0;


    public static class SensorTriplet {

        DistanceSensor[] distanceSensors = new DistanceSensor[3];

        enum TripletType {
            Right,
            Center,
            Left
        }

        //<editor-fold desc="Init setups">
        //dL = the left most sensor
        //dC = the center sensor
        //dR = the right most sensor
        public SensorTriplet(DistanceSensor dL, DistanceSensor dC, DistanceSensor dR) {
            distanceSensors[0] = dL;
            distanceSensors[1] = dC;
            distanceSensors[2] = dR;
        }

        public SensorTriplet(OpMode opMode, String dL, String dC, String dR) {
            distanceSensors[0] = opMode.hardwareMap.get(DistanceSensor.class, dL);
            distanceSensors[1] = opMode.hardwareMap.get(DistanceSensor.class, dC);
            distanceSensors[2] = opMode.hardwareMap.get(DistanceSensor.class, dR);
        }

        public SensorTriplet() {
        }
        //</editor-fold>

        //<editor-fold desc="External return groups">
        public double getDistance(SensorTriplet.TripletType type, DistanceUnit unit) {
            return sensor(type).getDistance(unit);
        }

        public double getDistanceAverage(DistanceUnit unit) {
            return (getDistance(SensorTriplet.TripletType.Center, unit) + getDistance(SensorTriplet.TripletType.Right, unit) + getDistance(SensorTriplet.TripletType.Left, unit)) / 3;
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(SensorTriplet.TripletType type) {
            return type == SensorTriplet.TripletType.Center ? distanceSensors[1] : (type == SensorTriplet.TripletType.Right ? distanceSensors[2] : distanceSensors[0]);
        }

        public double getWallAngle() {
            return Math.toDegrees(((bMath.pi() * 3) / 4) - Math.atan(getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM) / getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM)));
        }

        //Returns true if the three sensors have hit a perfect (with in 5%, see "error") line, this can be used to check if there's another robot or obstacle near us
        //Get bounds to work on the sensors
        //Error is between 0 (0%) and 1 (100%)
        public boolean isValid(double error) {
            double sinPiOver4 = bMath.sq2() / 2;
            double e = getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM) * sinPiOver4;
            double w = getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM);
            double q = getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM) * sinPiOver4;
            double difference = Math.abs((q - w) / q - (w - e) / e);
            if (difference <= error) {
                return true;
            } else {
                return false;
            }

        }


        //</editor-fold>
    }

    public static class AvoidanceConfiguration {

        public double currentDistance;

        //Target distance to track too
        public double range;

        //Bounds around the range
        //-b+[range]+b
        public double bounds;

        //The amount of degrees to move in order to correct movement, 45 = fast, 0 = no correction, 25 recommended
        public double correctionScale;

        //Init the config with range bounds and scale
        public AvoidanceConfiguration(double _range, double _bounds, double _correctionScale) {
            range = _range;
            bounds = _bounds;
            correctionScale = _correctionScale;
        }

        public AvoidanceConfiguration() {
        }

        //Returns a number between -1 and 1 based on which way we need to move deh bot and how fast we need too
        public Double CorrectionCoefficient() {

            double factor = 0;

            factor = (Math.abs(currentDistance - range) - bounds) / bounds;

            factor = bMath.Clamp(factor, 0, 1);

            //Multiply by -1 if we need to move away from the wall
            factor *= currentDistance > range ? 1 : -1;

            return factor;
        }

        //The angle that we wanna move in (additive)
        public Double targetDirection() {
            return CorrectionCoefficient() * correctionScale;
        }

        //Called to update the current distance var
        public void SetCurrentDistance(double value) {
            currentDistance = value;
        }

    }

    @Override
    public void Init(LinearOpMode op) {
        super.Init(op);
        avoidanceConfig = new AvoidanceConfiguration(50, 1, 25);
    }


    //Called after we run this in a opmode, sets up sensors n jazz
    public void StartValues(Double targetDistance, Double targetDistanceTolorance, Double correctionScale, SensorTriplet sensorTriplet) {
        sensors = sensorTriplet;
        avoidanceConfig = new AvoidanceConfiguration(targetDistance, targetDistanceTolorance, correctionScale);

    }

    @Override
    public void Loop() {
        super.Loop();

        wallAngle = sensors.getWallAngle();
        weightedWallAngle = bMath.MoveTowardsRadian(weightedWallAngle, Math.toRadians(wallAngle - 90), deltaTime.deltaTime() * 3);

        distance = sensors.getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM);
        avoidanceConfig.SetCurrentDistance(distance);

        curDriveAngle = wallAngle + avoidanceConfig.targetDirection();

        robot.MoveComplex(curDriveAngle, 0.5, weightedWallAngle);
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