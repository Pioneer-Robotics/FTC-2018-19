package org.firstinspires.ftc.teamcode.Robot.Input;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.bMotor;
import org.firstinspires.ftc.teamcode.Robot.RobotWallTrack;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;

//WIP WIP WIP, this class will store sensor and encoder data for speedy reads
//Separate threads for each sensor type to avoid hang ups
public class RobotInputThread extends Thread {

    public DistanceSensorInputThread distanceSensorInputThread = new DistanceSensorInputThread();
    public MotorEncoderInputThread motorEncoderInputThread = new MotorEncoderInputThread();

    public void Start() {
        distanceSensorInputThread.start();
        motorEncoderInputThread.start();
    }

    public void Stop() {
        distanceSensorInputThread.Stop();
        motorEncoderInputThread.Stop();
    }

    public void AddMotor(bMotor motor) {
        motorEncoderInputThread.bMotorPositionPairs.put(motor, 0);
    }

    public void AddSensor(DistanceSensor sensor) {
        distanceSensorInputThread.sensorDistancePairs.put(sensor, (double) 0);
    }

    public int getMotorPosition(bMotor motor) {
        return motorEncoderInputThread.bMotorPositionPairs.get(motor);
    }

    //Returns distance in CM
    public double getDistance(DistanceSensor distanceSensor) {
        return distanceSensorInputThread.sensorDistancePairs.get(distanceSensor);
    }
}


class DistanceSensorInputThread extends Thread {

    public HashMap<DistanceSensor, Double> sensorDistancePairs = new HashMap<DistanceSensor, Double>();

    public AtomicBoolean threadRunning = new AtomicBoolean();

    public void run() {
        threadRunning.set(true);

        //Loop to assign all distance values
        while (threadRunning.get()) {
            for (DistanceSensor distanceSensor : sensorDistancePairs.keySet()) {
                sensorDistancePairs.put(distanceSensor, distanceSensor.getDistance(DistanceUnit.CM));
            }
        }
    }




    public void Stop() {
        threadRunning.set(false);
    }
}

class MotorEncoderInputThread extends Thread {

    public HashMap<bMotor, Integer> bMotorPositionPairs = new HashMap<bMotor, Integer>();

    public AtomicBoolean threadRunning = new AtomicBoolean();

    public void run() {
        threadRunning.set(true);

        //Loop to assign all distance values
        while (threadRunning.get()) {
            for (bMotor motor : bMotorPositionPairs.keySet()) {
                bMotorPositionPairs.put(motor, motor.getCurrentPosition());
            }
        }
    }


    public void Stop() {
        threadRunning.set(false);
    }
}