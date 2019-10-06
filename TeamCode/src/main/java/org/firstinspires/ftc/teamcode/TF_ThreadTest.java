
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


//Super simple threader class to allow for threaded recognition, not commented or pretty yet!
public class TF_ThreadTest extends Thread {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AQMfl/L/////AAABmTblKFiFfUXdnoB7Ocz4UQNgHjSNJaBwlaDm9EpX0UI5ISx2EH+5IoEmxxd/FG8c31He17kM5vtS0jyAoD2ev5mXBiITmx4N8AduU/iAw/XMC5MiEB1YBgw5oSO1qd4jvCOgbzy/HcOpN3KoVVnYqKhTLc8n6/IIFGy+qyF7b8WkzscJpybOSAT5wtaZumdBu0K3lHV6n+fqGJDMvkQ5xrCS6HiBtpZScAoekd7iP3IxUik2rMFq5hqMsOYW+qlxKp0cj+x4K9CIOYEP4xZsCBt66UxtDSiNqaiC1DyONtFz4oHJf/4J5aYRjMNwC2BpsVJ/R91WIcC0H0dpP9gtL/09J0bIMjm3plo+ac+OM0H3";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    //Used by the thread to ensure it keeps running
    public boolean running;

    private Recognition currentRecognition;

    public Recognition skyStone() {
        return currentRecognition;
    }

    public void run() {
        running = true;
        if (tfod != null) {
            tfod.activate();
        }
        //Declare the list outside of ze loop to avoid GC GC GC
        List<Recognition> recognitions;

        while (running) {
            recognitions = tfod.getRecognitions();

            currentRecognition = null;
            for (Recognition recognition : recognitions) {
                //If sky stone seen
                if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
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


    //Init stuffs
    //Called from an OpMode to start threaded object detection. Make sure to call .start after running this!
    public void startFromOpmode(LinearOpMode op) {

        WebcamName camera = op.hardwareMap.get(WebcamName.class, "Webcam 1");
        int moniterID = op.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());


        initAll(camera, moniterID, 0.75);
    }

    public void initAll(WebcamName camera, int tfodMonitorViewId, double minConfidence) {
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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
