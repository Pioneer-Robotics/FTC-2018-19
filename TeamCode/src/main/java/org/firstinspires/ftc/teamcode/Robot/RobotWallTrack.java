package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;


//Called by the Robot.java to track along a flat surface. This surface is identified by inputting an angle value which correlates to the sensor group that will be used for tracking (look at the tape on the robot for anglez)
//Make sure to Trial and Error the hell out of this one!
//Might be fun to put this in Robot 'run'
public class RobotWallTrack {

    //List of all of our laser groups, mainly for ease of access
    public HashMap<groupID, SensorGroup> sensorIDGroupPairs = new HashMap<groupID, SensorGroup>();
    public Robot robot;


    //<editor-fold desc="Runtime data">
    AvoidanceConfiguration avoidanceConfig = new AvoidanceConfiguration();

    public SensorGroup currentGroup = new SensorGroup();

    double curDriveAngle = 0;
    double wallAngle = 0;
    //</editor-fold>

    public static class SensorGroup {

        public double distance;


        //The angle at which the sensors are located on the robot
        public double sensorAngle;

        public DistanceSensor[] distanceSensors = new DistanceSensor[2];

        public enum TripletType {
            Right,
            Left
        }

        //<editor-fold desc="Init setups">
        //dL = the left most sensor
        //dC = the center sensor
        //dR = the right most sensor
        public SensorGroup(DistanceSensor dL, DistanceSensor dR, Double dist, double angle) {
            distanceSensors[0] = dL;
            distanceSensors[1] = dR;
            distance = dist;
            sensorAngle = angle;
        }

        public SensorGroup(OpMode opMode, String dL, String dR, Double dist, double angle) {
            distanceSensors[0] = opMode.hardwareMap.get(DistanceSensor.class, dL);
            distanceSensors[1] = opMode.hardwareMap.get(DistanceSensor.class, dR);
            distance = dist;
            sensorAngle = angle;
        }

        public SensorGroup() {
        }
        //</editor-fold>

        //<editor-fold desc="External return groups">
        public double getDistance(SensorGroup.TripletType type, DistanceUnit unit) {
            return sensor(type).getDistance(unit);
        }

        public double getDistanceAverage(DistanceUnit unit) {
            return (getDistance(SensorGroup.TripletType.Right, unit) + getDistance(SensorGroup.TripletType.Left, unit)) / 2;
        }

        //Return a distance sensor from type
        public DistanceSensor sensor(SensorGroup.TripletType type) {
            return (type == SensorGroup.TripletType.Right ? distanceSensors[1] : distanceSensors[0]);
        }


        //The angle of the triangle formed by the diffidence in distance sensors
        //   A     B
        //   |     |
        //   |     |
        //   |     |
        //   | dist|
        //   |-----|
        //   |     /
        //   |    /
        //   |   /
        //   |  /
        //   |o/
        //   |/
        //ASCII's hard, check the journal for better pictures
        public double getWallAngle() {
            return Math.toDegrees(Math.atan((getDistance(SensorGroup.TripletType.Right, DistanceUnit.CM) - getDistance(SensorGroup.TripletType.Left, DistanceUnit.CM)) / distance));
        }

        public double getWallAngleRelative() {
            return getWallAngle() - sensorAngle;
        }

//        //Returns true if the three sensors have hit a perfect (with in 5%, see "error") line, this can be used to check if there's another robot or obstacle near us
//        //Get bounds to work on the sensors
//        //Error is between 0 (0%) and 1 (100%)
//        public boolean isValid(double error) {
//            return true;
//        }


        //</editor-fold>
    }

    public static class AvoidanceConfiguration {

        public double targetAngle;

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
            factor *= currentDistance > range ? -1 : 1;
            if (targetAngle < 0) {
                factor *= currentDistance > range ? -1 : 1;
            }
            return factor;
        }

        //The angle that we wanna move in (additive)
        public Double targetDirection() {
            return (CorrectionCoefficient() * correctionScale);
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

        sensorIDGroupPairs.put(groupID.Group90, new SensorGroup(op, RobotConfiguration.distanceSensor_90A, RobotConfiguration.distanceSensor_90B, RobotConfiguration.distance_90AB, 90));
        sensorIDGroupPairs.put(groupID.Group180, new SensorGroup(op, RobotConfiguration.distanceSensor_180A, RobotConfiguration.distanceSensor_180B, RobotConfiguration.distance_180AB, 180));
        sensorIDGroupPairs.put(groupID.Group270, new SensorGroup(op, RobotConfiguration.distanceSensor_270A, RobotConfiguration.distanceSensor_270B, RobotConfiguration.distance_270AB, -90));

    }

    //Moves in the 'positive' direction along the wall with no smoothing, to move backwards set speed negative. If the wall is NOT valid then don't call this weird stuff will happen
    //speed == how fast we move along the wall, also in which direction
    //distance == how far away from the wall should we be
    //bounds == +-distance how far are we allowed to be before correction (5cm seems reasonable)
    //correctionScale == think of it as how fast we correct our self (its an angle measure: 0 = no correction, 90 == max correction), leave it around 25.
    public void MoveAlongWallSimple(groupID group, double speed, double distance, double bounds, double correctionScale, double angleOffset, double rotationLockAngle) {
        MoveAlongWallSimple(sensorIDGroupPairs.get(group), speed, distance, bounds, correctionScale, angleOffset, rotationLockAngle);
    }

    public void MoveAlongWallSimple(SensorGroup group, double speed, double distance, double bounds, double correctionScale, double angleOffset, double rotationLockAngle) {

        //Configure the avoidance config
        avoidanceConfig.range = distance;
        avoidanceConfig.bounds = bounds;
        avoidanceConfig.correctionScale = correctionScale;
        avoidanceConfig.targetAngle = angleOffset;

        //Set up the group
        currentGroup = sensorIDGroupPairs.get(group);

        //Get the current sensors wall angle
        wallAngle = currentGroup.getWallAngle();

        //send our current world distance to the avoidance config
        avoidanceConfig.SetCurrentDistance(currentGroup.getDistanceAverage(DistanceUnit.CM));

        //Add the avoidance offset to our wall angle (to maintain the 'distance' from the wall)
        curDriveAngle = angleOffset - wallAngle + avoidanceConfig.CorrectionCoefficient();

        //MOVE
        robot.MoveSimple(angleOffset - wallAngle, speed);
//        robot.MoveComplex(angleOffset - wallAngle, speed, rotationLockAngle);
    }

    //Returns the sensor group that has the smallest average reading
    public SensorGroup closestGroup() {


        SensorGroup group = null;
        double lowestDistance = 1000000;

        //Itterate threw all of our sensor pairs and find the one with the smallest current distance
        for (SensorGroup currentGroup : sensorIDGroupPairs.values()) {
            if (lowestDistance < currentGroup.getDistanceAverage(DistanceUnit.CM)) {
                lowestDistance = currentGroup.getDistanceAverage(DistanceUnit.CM);
                group = currentGroup;
            }
        }

        //return group with smallest reading
        return group;
    }


}
