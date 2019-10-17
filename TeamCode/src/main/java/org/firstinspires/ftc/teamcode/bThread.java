
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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
        StartThread();
    }

    //Starts the thread (calls 'run')
    public void StartThread() {
        start();
    }


    //Main loop called from the thread run
    public void Loop() {

    }

    //Called after the thread is disposed
    public void OnThreadStop() {


    }
}
