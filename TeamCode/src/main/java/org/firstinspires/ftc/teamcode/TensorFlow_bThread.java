
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
public class TensorFlow_bThread extends bThread {

    @Override
    public void stopThread() {
        super.stopThread();
    }

    @Override
    public void startFromOpmode(LinearOpMode op/*,double minConfidance, String label*/) {
        super.startFromOpmode(op);
    }

    @Override
    public void startThread() {
        super.startThread();
    }

    @Override
    public void Init(LinearOpMode op) {
        super.Init(op);
    }

    @Override
    public void Loop() {
        super.Loop();
    }

    @Override
    public void OnThreadStop() {
        super.OnThreadStop();
    }
}
