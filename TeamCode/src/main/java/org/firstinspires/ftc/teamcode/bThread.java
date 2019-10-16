
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


//A base thread class that has basic start and stop functions. override methods as they fit your needs
public class bThread extends Thread {

    //Used by the thread to ensure it keeps running
    public boolean running;


    public void run() {
        running = true;
        while (running) {
            Loop();
        }
        OnThreadStop();
    }

    public void stopThread() {
        running = false;
    }

    //Called from an OpMode to start bThread
    public void startFromOpmode(LinearOpMode op) {
        Init(op);
        startThread();
    }

    //Starts the thread (calls 'run')
    public void startThread() {
        start();
    }

    //Starting stuffs before we run the thread
    public void Init(LinearOpMode op) {

    }

    //Main loop called from the thread run
    public void Loop() {


    }

    //Called after the thread is disposed
    public void OnThreadStop() {


    }
}
