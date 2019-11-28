package org.firstinspires.ftc.teamcode.Robot;

//WIP WIP WIP
//Going to be used when placing the robot on the field to ensure consistent placement
public class RobotSetUpHelper {

    public Verification sensors90;
    public Verification sensors180;


    public class Verification {
        public double target;

        public double current;

        public double tolerance;

        public boolean Valid() {
            return Math.abs(target - current) < tolerance;
        }

    }


    public RobotSetUpHelper(double angleGoal90, double angleGoal180) {

    }

    public void Update() {


    }
}
