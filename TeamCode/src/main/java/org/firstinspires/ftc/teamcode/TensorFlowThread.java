
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


//Super simple threader class to allow for threaded recognition, not commented or pretty yet! (its also set up in a not great way I'll be reworkin parts of it later for better performance!)
//Only can see one of TARGET_LABEL at a time
public class TensorFlowThread extends Thread {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static String TARGET_LABEL = "";

    private static final String VUFORIA_KEY = "AQMfl/L/////AAABmTblKFiFfUXdnoB7Ocz4UQNgHjSNJaBwlaDm9EpX0UI5ISx2EH+5IoEmxxd/FG8c31He17kM5vtS0jyAoD2ev5mXBiITmx4N8AduU/iAw/XMC5MiEB1YBgw5oSO1qd4jvCOgbzy/HcOpN3KoVVnYqKhTLc8n6/IIFGy+qyF7b8WkzscJpybOSAT5wtaZumdBu0K3lHV6n+fqGJDMvkQ5xrCS6HiBtpZScAoekd7iP3IxUik2rMFq5hqMsOYW+qlxKp0cj+x4K9CIOYEP4xZsCBt66UxtDSiNqaiC1DyONtFz4oHJf/4J5aYRjMNwC2BpsVJ/R91WIcC0H0dpP9gtL/09J0bIMjm3plo+ac+OM0H3";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    //Used by the thread to ensure it keeps running
    public boolean running;

    private Recognition currentRecognition;

    public Recognition getCurrentRecognition() {
        return currentRecognition;
    }

    public Boolean hasRecognition() {
        return currentRecognition != null;
    }

    //The a number between -1 and 1 representing how close the recognition is to the center of the camera
    public double getCurrentXFactor() {
        return (getCenterXPosition() - (currentRecognition.getImageWidth() / 2)) / currentRecognition.getImageWidth();
    }

    public float getCenterXPosition() {
        if (getCurrentRecognition() != null) {
//            float factor = currentRecognition.getLeft() + ((currentRecognition.getRight() - currentRecognition.getLeft()) / 2);
            float factor = (currentRecognition.getLeft() + currentRecognition.getRight()) / 2;
            return factor;
        }
        return -1;
    }

    //Called on a new thread by start()
    public void run() {
        running = true;
        if (tfod != null) {
            tfod.activate();
        }

        List<Recognition> recognitions;

        while (running) {
            recognitions = tfod.getRecognitions();

            currentRecognition = null;
            for (Recognition recognition : recognitions) {
                if (recognition.getLabel() == TARGET_LABEL) {
                    currentRecognition = recognition;
                }
            }


        }

        if (tfod != null) {
            tfod.shutdown();
        }

    }

    public void stopThread() {
        running = false;
    }

    //Called from an OpMode to start threaded object detection.
    public void startThread(LinearOpMode op, String label, double minConfidence) {

        TARGET_LABEL = label;
        WebcamName camera = op.hardwareMap.get(WebcamName.class, "Webcam 1");
        int moniterID = op.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());


        initAll(camera, moniterID, minConfidence);

        start();
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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, TARGET_LABEL);
    }
}
