package org.firstinspires.ftc.teamcode;

import java.lang.Object;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "TestTesting", group = "FTCPio")
public class Testing extends Teleop {

    //Array of all sensors
    DistanceSensor[] distanceSensors = new DistanceSensor[8];

    //Values from all distance sensors
    double[] ranges = new double[8];

    //Used durring wall adjustments
    double rotationDelta = 0;

    @Override
    public void init() {
        for (int i = 0; i <= distanceSensors.length; i++) {
            distanceSensors[i] = hardwareMap.get(DistanceSensor.class, "sensor_dist_" + i);
            ranges[i] = 0;
        }
    }

    boolean lockToggle = false;

    @Override
    /*Sensor pairs
0 - 1 => Frnt
2 - 3 => Right
4 - 5 => Down
6 - 7 => Left
     */

    public void loop() {

        //Grab all sensor datas
        for (int i = 0; i <= distanceSensors.length; i++) {
            ranges[i] = distanceSensors[i].getDistance(DistanceUnit.CM);
        }


//        //Itterate through all dem data's to determine rotationD
//        rotationDelta = (ranges[0] > ranges[1] ? 1 : -1);
//        rotationDelta = (ranges[2] > ranges[3] ? 1 : -1);
//        rotationDelta = (ranges[4] > ranges[5] ? 1 : -1);
//        rotationDelta = (ranges[6] > ranges[7] ? 1 : -1);



        //If hell.
        //NTS: Check do the thinking and figure out them r∆ values
        if (smallestDataValue(ranges) == 0) {
            rotationDelta = (ranges[0] > ranges[1] ? 1 : -1);
        }
        if (smallestDataValue(ranges) == 2) {
            rotationDelta = (ranges[2] > ranges[3] ? 1 : -1);
        }
        if (smallestDataValue(ranges) == 4) {
            rotationDelta = (ranges[4] > ranges[5] ? 1 : -1);
        }
        if (smallestDataValue(ranges) == 6) {
            rotationDelta = (ranges[6] > ranges[7] ? 1 : -1);
        }


    }


    private int smallestDataValue(double[] input) {

        int dataID = 0;
        double dataValue = Double.POSITIVE_INFINITY;

        for (int i = 0; i <= input.length; i++) {
            if (input[i] > dataValue) {
                dataValue = input[i];
                dataID = i;
            }
        }

        return dataID;
    }
}