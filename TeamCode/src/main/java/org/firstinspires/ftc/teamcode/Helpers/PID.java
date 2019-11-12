package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double2;

public class PID {

    public long dt;

    public long lastLoopTime;

    public double targetState;
    public double integral;
    public double derivative;
    public double lastError;
    public double error;

    public double P;
    public double I;
    public double D;

    public void Start(double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }

    //Double spits out the PID'd value
    public double Loop(double currentState) {


        dt = 1000 * (lastLoopTime - System.currentTimeMillis());
        error = targetState - currentState;
        integral += error * dt;
        derivative = (error - lastError) / dt;
        lastError = error;

        lastLoopTime = System.currentTimeMillis();

        return (P * error) + (I * integral) + (D * derivative);
    }

}
