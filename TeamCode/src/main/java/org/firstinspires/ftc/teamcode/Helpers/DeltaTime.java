package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double2;
import android.renderscript.Double4;

//Helper class for getting the passage of time, used as deltaTime in time sensitive functions.
public class DeltaTime {

    long startTime;

    long deltaTimeLong;

    public DeltaTime() {

    }


    //Sets the start time
    public void Start() {
        startTime = System.currentTimeMillis();
    }

    //Sets the start time
    public void Stop() {
        deltaTimeLong = System.currentTimeMillis() - startTime;
    }

    public Double deltaTime() {
        return (double) deltaTimeLong / 1000;
    }
}
