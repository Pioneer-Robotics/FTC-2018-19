package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

import java.util.concurrent.atomic.AtomicBoolean;

//WIP WIP WIP, this class will store sensor data for speedy reads
public class RobotInputThread extends Thread {

    public RobotWallTrack.SensorGroup group90;
    public RobotWallTrack.SensorGroup group180;
    public RobotWallTrack.SensorGroup group270;

    AtomicBoolean threadRunning = new AtomicBoolean();

    public void run() {
        threadRunning.set(true);
        while (threadRunning.get()) {

        }
    }

}
