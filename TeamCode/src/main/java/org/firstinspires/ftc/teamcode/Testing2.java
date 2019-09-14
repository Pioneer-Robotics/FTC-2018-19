package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "123", group = "Test")
public class Testing2 extends LinearOpMode {

    HardwareInfinity1 hardwareInfinity = new HardwareInfinity1();


    //Array of all sensors
    DistanceSensor distanceSensor;

    //Values from all distance sensors
    double range = 0;


    SensorTolerance toleranceConfig = new SensorTolerance(3, 10);

    public class SensorTolerance {
        public double threshold = 3;

        public double goalRange = 10;

        public SensorTolerance(double t, double gr) {
            threshold = t;
            goalRange = gr;
        }
    }

    @Override
    public void runOpMode() {

        telemetry.addLine("OP MODE STARTED");
        telemetry.update();
        hardwareInfinity.init(hardwareMap, this);
        telemetry.addLine("HARDWARE CONFIGURED");
        telemetry.update();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        telemetry.addLine("DISTANDCE SENSOR INIT");
        telemetry.update();
        toleranceConfig = new SensorTolerance(3, 10);
        telemetry.addLine("TOLARANCE INIT");
        telemetry.update();

        while (opModeIsActive()) {
            range = distanceSensor.getDistance(DistanceUnit.CM);
            if (Math.abs(range - toleranceConfig.goalRange) > toleranceConfig.threshold) {
                hardwareInfinity.MecDrive(range - toleranceConfig.goalRange, (range - toleranceConfig.goalRange < 0 ? 0 : 180), 0.5);
            }


        }

        telemetry.addLine("CODE EXITED >:(");
        telemetry.update();
    }


}