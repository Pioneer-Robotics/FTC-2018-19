package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//As of 9.15.19.0706 this serves only to test sensor input
//As of 10.1.19 1549 this serves to track along walls smoothly
//TODO: Add curves smoothing to rotation values. Also using a paralabic function for movement smoothing could be cool?
@Autonomous(name = "WTest", group = "Sensor")
public class LazerThreading extends Thread {
    HardwareInfinityMec robot = new HardwareInfinityMec();

    AvoidanceConfiguration avoidanceConfig = new AvoidanceConfiguration();

    SensorTriplet sensors = new SensorTriplet();
    DeltaTime deltaTime = new DeltaTime();

    public static class SensorTriplet {

        DistanceSensor[] distanceSensors = new DistanceSensor[3];

        enum TripletType {
            Right,
            Center,
            Left
        }

        //<editor-fold desc="Init setups">
        //dL = the left most sensor
        //dC = the center sensor
        //dR = the right most sensor
        public SensorTriplet(DistanceSensor dL, DistanceSensor dC, DistanceSensor dR) {
            distanceSensors[0] = dL;
            distanceSensors[1] = dC;
            distanceSensors[2] = dR;
        }

        public SensorTriplet(OpMode opMode, String dL, String dC, String dR) {
            distanceSensors[0] = opMode.hardwareMap.get(DistanceSensor.class, dL);
            distanceSensors[1] = opMode.hardwareMap.get(DistanceSensor.class, dC);
            distanceSensors[2] = opMode.hardwareMap.get(DistanceSensor.class, dR);
        }

        public SensorTriplet() {
        }
        //</editor-fold>

        //<editor-fold desc="External return groups">
        public double getDistance(TripletType type, DistanceUnit unit) {
            return sensor(type).getDistance(unit);
        }

        public double getDistanceAverage(DistanceUnit unit) {
            return (getDistance(TripletType.Center, unit) + getDistance(TripletType.Right, unit) + getDistance(TripletType.Left, unit)) / 3;
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(TripletType type) {
            return type == TripletType.Center ? distanceSensors[1] : (type == TripletType.Right ? distanceSensors[2] : distanceSensors[0]);
        }

        public double getWallAngle() {
            return Math.toDegrees(((bMath.pi() * 3) / 4) - Math.atan(getDistance(SensorTriplet.TripletType.Right, DistanceUnit.CM) / getDistance(SensorTriplet.TripletType.Left, DistanceUnit.CM)));
        }

        //</editor-fold>
    }

    public static class AvoidanceConfiguration {

        public double currentDistance;

        //Target distance to track too
        public double range;

        //Bounds around the range
        //-b+[range]+b
        public double bounds;

        //The amount of degrees to move in order to correct movement, 45 = fast, 0 = no correction, 25 recommended
        public double correctionScale;

        //Init the config with range bounds and scale
        public AvoidanceConfiguration(double _range, double _bounds, double _correctionScale) {
            range = _range;
            bounds = _bounds;
            correctionScale = _correctionScale;
        }

        public AvoidanceConfiguration() {
        }

        //Returns a number between -1 and 1 based on which way we need to move deh bot and how fast we need too
        public Double CorrectionCoefficient() {

            double factor = 0;

            factor = (Math.abs(currentDistance - range) - bounds) / bounds;

            factor = bMath.Clamp(factor, 0, 1);

            //Multiply by -1 if we need to move away from the wall
            factor *= currentDistance > range ? 1 : -1;

            return factor;
        }

        //The angle that we wanna move in (additive)
        public Double targetDirection() {
            return CorrectionCoefficient() * correctionScale;
        }

        //Called to update the current distance var
        public void SetCurrentDistance(double value) {
            currentDistance = value;
        }

    }

    public void startThread(OpMode op) {
        robot.init(op.hardwareMap, op);
        start();
    }

    public void run() {


    }
}
