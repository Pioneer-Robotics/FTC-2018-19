package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double2;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    public ElapsedTime deltaTime = new ElapsedTime();
    public double integral;
    public double derivative;
    public double lastError;
    public double error;

    public double P;
    public double I;
    public double D;

    double lastReturnValue;

    public void Start(double p, double i, double d) {
        P = p;
        I = i;
        D = d;
        lastReturnValue = -1;

        error = 0;
        lastError = 0;
        derivative = 0;
        integral = 0;

        deltaTime.reset();
    }

    //Double spits out the PID'd value
    public double Loop(double targetState, double currentState) {

        error = targetState - currentState;
        integral += error * deltaTime.seconds();
        derivative = (error - lastError) / deltaTime.seconds();
        lastError = error;
        deltaTime.reset();
        lastReturnValue = (P * error) + (I * integral) + (D * derivative);
        return (P * error) + (I * integral) + (D * derivative);
    }

    public double State() {
        return lastReturnValue;
    }

}
