package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double2;

public class PID {

    public DeltaTime deltaTime = new DeltaTime();
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
    public double Loop(double targetState, double currentState) {
        deltaTime.Stop();

        error = targetState - currentState;
        integral += error * deltaTime.deltaTime();
        derivative = (error - lastError) / deltaTime.deltaTime();
        lastError = error;

        deltaTime.Start();
        return (P * error) + (I * integral) + (D * derivative);
    }

}
