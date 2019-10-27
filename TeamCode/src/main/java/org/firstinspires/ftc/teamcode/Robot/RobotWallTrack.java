package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.Dictionary;
import java.util.Enumeration;


//Called by the Robot.java to track along a flat surface. This surface is identified by inputting an angle value which correlates to the sensor group that will be used for tracking (look at the tape on the robot for anglez)
//Make sure to Trial and Error the hell out of this one!
public class RobotWallTrack {

    //List of all of our laser groups, mainly for ease of access
    //Side note, what the hell is java's dictionary system smoking
    public Dictionary<groupID, SensorGroup> sensorIDGroupPairs;

    public Robot robot;


    //<editor-fold desc="Runtime data">
    AvoidanceConfiguration avoidanceConfig = new AvoidanceConfiguration();

    SensorGroup currentGroup = new SensorGroup();

    double curDriveAngle = 0;
    double wallAngle = 0;
    //</editor-fold>

    public static class SensorGroup {

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
        public SensorGroup(DistanceSensor dL, DistanceSensor dC, DistanceSensor dR) {
            distanceSensors[0] = dL;
            distanceSensors[1] = dC;
            distanceSensors[2] = dR;
        }

        public SensorGroup(OpMode opMode, String dL, String dC, String dR) {
            distanceSensors[0] = opMode.hardwareMap.get(DistanceSensor.class, dL);
            distanceSensors[1] = opMode.hardwareMap.get(DistanceSensor.class, dC);
            distanceSensors[2] = opMode.hardwareMap.get(DistanceSensor.class, dR);
        }

        public SensorGroup() {
        }
        //</editor-fold>

        //<editor-fold desc="External return groups">
        public double getDistance(SensorGroup.TripletType type, DistanceUnit unit) {
            return sensor(type).getDistance(unit);
        }

        public double getDistanceAverage(DistanceUnit unit) {
            return (getDistance(SensorGroup.TripletType.Center, unit) + getDistance(SensorGroup.TripletType.Right, unit) + getDistance(SensorGroup.TripletType.Left, unit)) / 3;
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(SensorGroup.TripletType type) {
            return type == SensorGroup.TripletType.Center ? distanceSensors[1] : (type == SensorGroup.TripletType.Right ? distanceSensors[2] : distanceSensors[0]);
        }

        public double getWallAngle() {
            return Math.toDegrees(((bMath.pi() * 3) / 4) - Math.atan(getDistance(SensorGroup.TripletType.Right, DistanceUnit.CM) / getDistance(SensorGroup.TripletType.Left, DistanceUnit.CM)));
        }

        //Returns true if the three sensors have hit a perfect (with in 5%, see "error") line, this can be used to check if there's another robot or obstacle near us
        //Get bounds to work on the sensors
        //Error is between 0 (0%) and 1 (100%)
        public boolean isValid(double error) {
            double sinPiOver4 = bMath.sq2() / 2;
            double e = getDistance(SensorGroup.TripletType.Left, DistanceUnit.CM) * sinPiOver4;
            double w = getDistance(SensorGroup.TripletType.Center, DistanceUnit.CM);
            double q = getDistance(SensorGroup.TripletType.Right, DistanceUnit.CM) * sinPiOver4;
            double difference = Math.abs((q - w) / q - (w - e) / e);
            if (difference <= error) {
                return true;
            } else {
                return false;
            }

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

    public enum groupID {
        Group90,
        Group180,
        Group270
    }


    public void Start(OpMode op) {

        robot = Robot.instance;

        //put!?
        //Add's K/V to the dictionary
        sensorIDGroupPairs.put(groupID.Group90, new SensorGroup(op, "sensor 45", "sensor 90", "sensor 135"));
        sensorIDGroupPairs.put(groupID.Group180, new SensorGroup(op, "sensor 135", "sensor 180", "sensor 225"));
        sensorIDGroupPairs.put(groupID.Group270, new SensorGroup(op, "sensor 225", "sensor 270", "sensor 315"));

    }

    //Moves in the 'positive' direction along the wall with no smoothing, to move backwards set speed negative. If the wall is NOT valid then don't call this weird stuff will happen
    //speed == how fast we move along the wall, also in which direction
    //distance == how far away from the wall should we be
    //bounds == +-distance how far are we allowed to be before correction (5cm seems reasonable)
    //correctionScale == think of it as how fast we correct out self (its an angle measure), leave it around 25.
    public void MoveAlongWallSimple(groupID group, double speed, double distance, double bounds, double correctionScale) {

        //Configure the avoidance config
        avoidanceConfig.range = distance;
        avoidanceConfig.bounds = bounds;
        avoidanceConfig.correctionScale = correctionScale;

        //Set up the group
        currentGroup = sensorIDGroupPairs.get(group);

        //Get the current sensors wall angle
        wallAngle = currentGroup.getWallAngle();

        //send our current world distance to the avoidance config
        avoidanceConfig.SetCurrentDistance(currentGroup.getDistance(SensorGroup.TripletType.Center, DistanceUnit.CM));

        //Add the avoidance offset to our wall angle (to maintain the 'distance' from the wall)
        curDriveAngle = wallAngle + avoidanceConfig.targetDirection();

        //MOVE
        robot.MoveSimple(curDriveAngle, speed);
    }


    //Returns if the current group is on a line, check this before moving along a line
    //group == teh group we want to verify
    //error == the amount of error we can accept, between 0 and 1 (1 being 100%)
    public boolean CheckWallValidity(groupID group, double error) {
        return sensorIDGroupPairs.get(group).isValid(error);
    }

//    public void MoveAlongWall(groupID group) {
//
//        currentGroup = sensorIDGroupPairs.get(group);
//
//        wallAngle = currentGroup.getWallAngle();
//        weightedWallAngle = bMath.MoveTowardsRadian(weightedWallAngle, Math.toRadians(wallAngle - 90), deltaTime.deltaTime() * 3);
//
//        distance = currentGroup.getDistance(SensorTriplet.TripletType.Center, DistanceUnit.CM);
//        avoidanceConfig.SetCurrentDistance(distance);
//
//        curDriveAngle = wallAngle + avoidanceConfig.targetDirection();
//
//        robot.MoveComplex(curDriveAngle, 0.5, weightedWallAngle);
//    }
}
