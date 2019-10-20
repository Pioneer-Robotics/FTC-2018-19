
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Helpers.bThread;

import java.util.List;


//Super simple threader class to allow for threaded recognition, not commented or pretty yet! (its also set up in a not great way I'll be reworkin parts of it later for better performance!)
//Only can see one of TARGET_LABEL at a time
public class TensorFlow_bThread extends bThread {


    //<editor-fold desc="String declarations ">
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static String TARGET_LABEL = "";
    private static final String VUFORIA_KEY = "AQMfl/L/////AAABmTblKFiFfUXdnoB7Ocz4UQNgHjSNJaBwlaDm9EpX0UI5ISx2EH+5IoEmxxd/FG8c31He17kM5vtS0jyAoD2ev5mXBiITmx4N8AduU/iAw/XMC5MiEB1YBgw5oSO1qd4jvCOgbzy/HcOpN3KoVVnYqKhTLc8n6/IIFGy+qyF7b8WkzscJpybOSAT5wtaZumdBu0K3lHV6n+fqGJDMvkQ5xrCS6HiBtpZScAoekd7iP3IxUik2rMFq5hqMsOYW+qlxKp0cj+x4K9CIOYEP4xZsCBt66UxtDSiNqaiC1DyONtFz4oHJf/4J5aYRjMNwC2BpsVJ/R91WIcC0H0dpP9gtL/09J0bIMjm3plo+ac+OM0H3";
    //</editor-fold>

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public Recognition currentRecognition;

    List<Recognition> recognitions;

    public LinearOpMode opMode;

    @Override
    public void run() {
        super.run();
        running = true;
        while (running) {
            Loop();
        }
        OnThreadStop();
    }

    @Override
    public void Loop() {
        super.Loop();

        opMode.telemetry.addData("Loooopy looping", "");

        recognitions = tfod.getRecognitions();

        currentRecognition = null;
        for (Recognition recognition : recognitions) {
            opMode.telemetry.addData("Checking Recognitions : ", recognition);
            opMode.telemetry.addData("Checking Recognition Name", TARGET_LABEL);

            if (recognition.getLabel() == TARGET_LABEL) {
                currentRecognition = recognition;
            }
        }
        opMode.telemetry.update();
    }

    @Override
    public void OnThreadStop() {
        super.OnThreadStop();

        //Shutdown TensorFlow, if we have one
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void StartTensorFlow(LinearOpMode op, String label, double minConfidence) {
        //Set what object we are looking for (this is the name of the object that will be returned by the currentRecognition)
        TARGET_LABEL = label;

        //Find the current webcam
        WebcamName camera = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        //Fetch the tfod ID
        int moniterID = op.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());

        //Set up the camera via vuforia
        startVuforia(camera);


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            //Set up tensor flow
            //minConfidence should be a number between 0 and 1 (1 = 100% certain that the object is what we want, 60-80 seems reasonable)
            startTensorFlow(moniterID, minConfidence);
        }
        opMode = op;
        this.start();
    }

    private void startVuforia(WebcamName camera) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = camera;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void startTensorFlow(int tfodMonitorViewId, double minConfidence) {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = minConfidence;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, "Stone", "Skystone");
    }


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
    //</editor-fold>

}
