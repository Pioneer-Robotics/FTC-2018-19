package org.firstinspires.ftc.teamcode;

import android.renderscript.Double4;
import android.renderscript.Double2;

//Basic math helper class
public class bMath {

    //Cached variables used in maths to avoid the evil GC monster, doubt this is a concern but what the hell
    static double leftMovementPower;
    static double rightMovementPower;
    static double leftRotatePower;
    static double rightRotatePower;


    //Output:
    //X: Front Left
    //Y: Front Right
    //Z: Back Right
    //W: Back Left

    /*
    X____Y
    | ^^ |
    |    |
    |____|
    Z    W
     */
//Used to determine what wheels to move in order to move in movementVector direction and rotate
    //Check them rotation values?
    public static Double4 getMecMovement(Double2 movementVector, double rotation) {

        Double4 delta = new Double4(0, 0, 0, 0);

        leftMovementPower = ((-movementVector.y + movementVector.x) / sq2());
        rightMovementPower = ((-movementVector.y - movementVector.x) / sq2());
        leftRotatePower = rotation;
        rightRotatePower = -rotation;

        delta.x = (leftMovementPower + leftRotatePower);
        delta.y = (rightMovementPower + rightRotatePower);
        delta.z = (rightMovementPower + leftRotatePower);
        delta.w = (leftMovementPower + rightRotatePower);

        return delta;
    }

//Hook for angle based movement
    public static Double4 getMecMovement(double movementAngle, double rotation) {

        Double2 movementVector = degreesToHeadingVector(movementAngle);
        return getMecMovement(movementVector,rotation);
    }

    public static Double2 degreesToHeadingVector(double angle)
    {
        double a = Math.toRadians(angle);
        return radiansToHeadingVector(a);
    }

    public static Double2 radiansToHeadingVector(double angle)
    {
        return new Double2(Math.cos(angle),Math.sin(angle));
    }

//    public static Double2 normilizeVector(Double2 vector){
//        vector.x
//
//    }


    public static double pi() {
        return 3.14159265359;
    }

    public static double sq2() {
        return 1.41421356237309;/*504880168872420969807856967187537694807317667973799073247846210703885038753432764157273501384623091229702492483605585073721264412149709993583141322266592750559275579995050115278206057147010955997160597027453459686201472851;*/
    }
}
