package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double4;
import android.renderscript.Double2;

//Math helper class, for math.
public class bMath {

    //<editor-fold desc="getMecMovement">

    //Cached variables used in maths to avoid the evil GC monster, doubt this is a concern but what the hell
    static double leftMovementPower;
    static double rightMovementPower;
    static double leftRotatePower;
    static double rightRotatePower;


    //Output: What we think it is
    //X: Front Left
    //Y: Front Right
    //Z: Back Right
    //W: Back Left

    /*
    X____Y
    | ^^ |
    |    ||
    |____|
    Z    W

    //Output: What happened when we tested (front is 0 degrees is phone side)
    //X: Back left
    //Y: Front Left
    //Z: Front right
    //W: Back right
    //(Joe)
     */
//Used to determine what wheels to move in order to move in movementVector direction and rotate
    //Check them rotation values?
    //ROTATION DOES NOT WORK!
    public static Double4 getMecMovement(Double2 movementVector, double angle) {

        Double4 delta = new Double4(0, 0, 0, 0);

        leftMovementPower = ((-movementVector.y - movementVector.x) / sq2() * Math.sin(angle) + ((-movementVector.y + movementVector.x) / sq2()) * Math.cos(angle));
        rightMovementPower = ((-(-movementVector.y + movementVector.x) / sq2()) * Math.sin(angle) + ((-movementVector.y - movementVector.x) / sq2() * Math.cos(angle)));
        leftRotatePower = angle;
        rightRotatePower = -angle;

        delta.x = (leftMovementPower + leftRotatePower);
        delta.y = (rightMovementPower + rightRotatePower);
        delta.z = (rightMovementPower + leftRotatePower);
        delta.w = (leftMovementPower + rightRotatePower);


        return delta; //delta is the 4 compo'"":"nent variable, Double4 delta (Joe)
    }

    public static Double4 getMecRotation(double angle, double speed) {

        Double4 delta = new Double4(0, 0, 0, 0);

//        leftMovementPower =  Math.sin(angle) + ((-movementVector.y + movementVector.x) / sq2()) * Math.cos(angle));
//        rightMovementPower = ((-(-movementVector.y + movementVector.x) / sq2()) * Math.sin(angle) + ((-movementVector.y - movementVector.x) / sq2() * Math.cos(angle)));


        leftRotatePower = angle * speed;
        rightRotatePower = -angle * speed;

        delta.x = (leftRotatePower);
        delta.y = (rightRotatePower);
        delta.z = (leftRotatePower);
        delta.w = (rightRotatePower);


        return delta; //delta is the 4 component variable, Double4 delta (Joe)
    }

    //Hook for angle based movement
    //Hook means the output from this is the input to another function of the same name (Joe)
    public static Double4 getMecMovement(double movementAngle, double rotation) {
        Double2 movementVector = degreesToHeadingVector(movementAngle);
        //input movementAngle into degreesToHeadingVector and the output is movementVector
        return getMecMovement(movementVector, rotation);
    }
    //</editor-fold>


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
        Double2 movementVector = degreesToHeadingVector(movementAngle);
        return getMecMovementSimple(movementVector);
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
    //</editor-fold>

    public static double MoveTowardsRadian(double current, double target, double maxDelta) {
        double offset = 0;
        double distance = target - current;
        if (distance < pi()) {
            offset += maxDelta;
        } else {
            offset -= maxDelta;
        }


        offset = maxDelta;

        if (Math.abs(target - current) <= maxDelta) {
            return target;
        }
        return current + offset;
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

}
