package org.firstinspires.ftc.teamcode.Robot;

//This class has all of the names for all of the things in one place for easy of access / sanity!
public class RobotConfiguration {

    //The IMU's names
    public static final String imu_0 = "imu";
    public static final String imu_1 = "imu1";


    //Configuration names for all of our wheels
    public static final String wheel_frontRight = "Front Right";
    public static final String wheel_frontLeft = "Front Left";
    public static final String wheel_backRight = "Back Right";
    public static final String wheel_backLeft = "Back Left";

    //Distance sensor 90 names
    public static final String distanceSensor_90A = "sensor 90A";
    public static final String distanceSensor_90B = "sensor 90B";

    //Distance sensor 180 names
    public static final String distanceSensor_180A = "sensor 180A";
    public static final String distanceSensor_180B = "sensor 180B";

    //The distance between the two sensors along side 90, used for walltrack math. In CM. Estements, please messure
    public static final double distance_90AB = 32.5;

    //The distance between the two sensors along side 180, used for walltrack math. In CM
    public static final double distance_180AB = 32.5;
}
