package org.firstinspires.ftc.teamcode.Robot;

//This class has all of the names and data for all of the things in one place for easy of access / sanity!
public class RobotConfiguration {

    //The IMU's names
    public static final String imu_0 = "imu";
    public static final String imu_1 = "imu1";

    /* Our cool robot configuration


         0
      ________
    |X|      |Y|
    | |  ^^  | |
270 | |      | |   90
    |Z|______|W|

        180


    Wheel placement and angles verified by Ben on the 21nd of November

     */


    //Configuration names for all of our wheels
    public static final String wheel_frontLeft = "Front Left";//This wheel should correspond to the X component of movement
    public static final String wheel_frontRight = "Front Right";//This wheel should correspond to the Y component of movement
    public static final String wheel_backLeft = "Back Left";//This wheel should correspond to the Z component of movement
    public static final String wheel_backRight = "Back Right";//This wheel should correspond to the Z component of movement

    //Configuration for all of the parts of the big arm
    public static final String arm_lengthMotor = "Arm Spool";
    public static final String arm_rotationMotor = "Arm Rotation";
    public static final String arm_gripServo = "Grip";
    public static final String arm_gripRotationServo = "Grip Rotation";


    //Distance sensor 0 name
    public static final String distanceSensor_0A = "sensor 0A";

    //Distance sensor 90 names
    public static final String distanceSensor_90A = "sensor 90A";
    public static final String distanceSensor_90B = "sensor 90B";

    //Distance sensor 180 names
    public static final String distanceSensor_180A = "sensor 180A";
    public static final String distanceSensor_180B = "sensor 180B";

    //Distance sensor 270 names
    public static final String distanceSensor_270A = "sensor 270A";
    public static final String distanceSensor_270B = "sensor 270B";

    //The distance between the two sensors along side 90, used for wall-tracking math. In CM.
    //Verified __________ by ___________
    public static final double distance_90AB = 33.5;

    //The distance between the two sensors along side 180, used for wall-tracking math. In CM
    //Verified __________ by ___________
    public static final double distance_180AB = 26.9;

    //The distance between the two sensors along side 180, used for wall-tracking math. In CM
    //Verified __________ by ___________
    public static final double distance_270AB = 33.7;


    //The amount of encoder ticks per motor rotation
    public static final int wheel_ticksPerRotation = 1440;

    //The max speed our wheel motors will ever rotate (in ticks per second), 3 rotations per second. Used in calibration.
    public static final int wheel_maxTicksPerSecond = 4320;

    //How many times we need to spin the motor in order to rotate the wheel once
    public static final double wheel_GearCoefficient = 20;

    //pi * diameter
    public static final double wheel_circumference = 32.2;
}
