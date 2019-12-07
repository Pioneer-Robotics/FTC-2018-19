package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double4;
import android.renderscript.Double2;

//Math helper class, for math.
public class bMath {

    //<editor-fold desc="getMecMovement">

    //Cached variables used in maths to avoid the evil GC monster.
    static double leftMovementPower;
    static double rightMovementPower;
    static double leftRotatePower;
    static double rightRotatePower;

    //Get mecMovement works by moving in a direction (angle/vector) AND TO a target angle
    public static Double4 getMecMovement(Double2 movementVector, double targetAngle) {

        Double4 delta = new Double4(0, 0, 0, 0);
        targetAngle = bMath.toRadians(targetAngle);

//        leftMovementPower = ((-movementVector.y - movementVector.x) / sq2() * Math.sin(targetAngle) + ((-movementVector.y + movementVector.x) / sq2()) * Math.cos(targetAngle));
        leftMovementPower = (((-movementVector.y - movementVector.x) / sq2()) * Math.sin(targetAngle)) + (((-movementVector.y + movementVector.x) / sq2()) * Math.cos(targetAngle));
//        rightMovementPower = ((-(-movementVector.y + movementVector.x) / sq2()) * Math.sin(targetAngle) + ((-movementVector.y - movementVector.x) / sq2() * Math.cos(targetAngle)));
        rightMovementPower = ((-(-movementVector.y + movementVector.x) / sq2()) * Math.sin(targetAngle)) + ((-movementVector.y - movementVector.x) / sq2() * Math.cos(targetAngle));
        leftRotatePower = targetAngle;
        rightRotatePower = -targetAngle;

        delta.x = (leftMovementPower + leftRotatePower);
        delta.y = (rightMovementPower + rightRotatePower);
        delta.z = (rightMovementPower + leftRotatePower);
        delta.w = (leftMovementPower + rightRotatePower);


        return delta; //delta is the 4 component variable, Double4 delta (Joe)
    }

    public static Double4 getRotationSimple(double speed) {

        Double4 delta = new Double4(0, 0, 0, 0);


        leftRotatePower = speed;
        rightRotatePower = -speed;

        delta.x = (rightRotatePower);
        delta.y = (leftRotatePower);
        delta.z = (rightRotatePower);
        delta.w = (leftRotatePower);


        return delta; //delta is the 4 component variable, Double4 delta (Joe)
    }

    //Hook for angle based movement
    //Hook means the output from this is the input to another function of the same name (Joe)
    public static Double4 getMecMovement(double movementAngle, double targetAngle) {
        Double2 movementVector = degreesToHeadingVector(movementAngle - 90);
        //input movementAngle into degreesToHeadingVector and the output is movementVector
        return getMecMovement(movementVector, targetAngle);
    }
    //</editor-fold>


    //Simple mec movement works by moving at a given angle/vector and rotating at rotationSpeed
    //<editor-fold desc="getMecMovementSimple">

    public static Double4 getMecMovementSimple(Double2 movementVector) {

        Double4 delta = new Double4(0, 0, 0, 0);

        leftMovementPower = ((movementVector.x - movementVector.y) / sq2());
        rightMovementPower = ((-movementVector.y - movementVector.x) / sq2());


        delta.x = (leftMovementPower);
        delta.y = (rightMovementPower);
        delta.z = (rightMovementPower);
        delta.w = (leftMovementPower);

        return delta;
    }

    //Hook for angle based movement
    public static Double4 getMecMovementSimple(double movementAngle) {
        Double2 movementVector = degreesToHeadingVector(movementAngle - 90);
        return getMecMovementSimple(movementVector);
    }

    public static Double4 getMecMovementSimple(Double2 movementVector, double rotationSpeed) {

        Double4 delta = new Double4(0, 0, 0, 0);

        leftMovementPower = ((movementVector.x - movementVector.y) / sq2());
        rightMovementPower = ((-movementVector.y - movementVector.x) / sq2());


        delta.x = (leftMovementPower) - rotationSpeed;
        delta.y = (rightMovementPower) + rotationSpeed;
        delta.z = (rightMovementPower) - rotationSpeed;
        delta.w = (leftMovementPower) + rotationSpeed;

        return delta;
    }

    //Hook for angle based movement
    public static Double4 getMecMovementSimple(double movementAngle, double rotationSpeed) {
        Double2 movementVector = degreesToHeadingVector(movementAngle - 90);
        return getMecMovementSimple(movementVector, rotationSpeed);
    }
    //</editor-fold>


    //<editor-fold desc="toHeadingVector">
    //ALL ANGLES SHOULD BE -180 to 180
    public static Double2 degreesToHeadingVector(double angle) {
        double a = Math.toRadians(angle);
        return radiansToHeadingVector(a);
    }

    public static Double2 radiansToHeadingVector(double angle) {
        return new Double2(Math.cos(angle), Math.sin(angle));
    }

    public static double headingVectorToDegrees(Double2 vector) {
        double a = Math.atan(vector.y / vector.x);
        return Math.toDegrees(a);
    }
    //</editor-fold>


    //<editor-fold desc="Misc">

    public static double pi() {
        return 3.14159265359;
    }

    public static double pi2() {
        return pi() * 2;
    }

    public static double sq2() {
        return 1.41421356237309;/*504880168872420969807856967187537694807317667973799073247846210703885038753432764157273501384623091229702492483605585073721264412149709993583141322266592750559275579995050115278206057147010955997160597027453459686201472851;*/
    }

    public static double Clamp(double value, double min, double max) {
        double v = value;

        if (value >= max) {
            v = max;
        }
        if (value <= min) {
            v = min;
        }

        return v;
    }

    public static long Clamp(long value, long min, long max) {
        long v = value;

        if (value >= max) {
            v = max;
        }
        if (value <= min) {
            v = min;
        }

        return v;
    }
    //</editor-fold>

    public static double MoveTowardsRadian(double current, double target, double maxDelta) {
        return MoveTowards(current, current + RadianDistance(current, target), maxDelta);
    }

    public static double Loop(double value, double magnitude) {
        return value - Math.floor(value / magnitude) * magnitude;
    }

    public static double RadianDistance(double current, double target) {
        double a = Loop(target - current, pi2());
        if (a > pi())
            a -= pi2();
        return a;

    }

    public static double MoveTowards(double current, double target, double maxDelta) {
        if (Math.abs(target - current) <= maxDelta) {
            return target;
        }
        return current + (maxDelta * (target - current > 0 ? 1 : 0));
    }

    public static double Lerp(double a, double b, double lerpFactor) {
        double result = ((1 - lerpFactor) * a) + (lerpFactor * b);
        return result;
    }

    public static double toRadians(double value) {
        return value * 0.0174;
    }

    public static double toDegrees(double value) {
        return value * 57.2957;
    }

}
