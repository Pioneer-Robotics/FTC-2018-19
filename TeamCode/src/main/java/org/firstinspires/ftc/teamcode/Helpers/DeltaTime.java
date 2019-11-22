package org.firstinspires.ftc.teamcode.Helpers;

import android.renderscript.Double2;
import android.renderscript.Double4;

//Helper class for getting the change in time, delta-time.
//DEPREICATED. Use ElapsedTime instead!
class XDeltaTime {

    long startTime;

    long deltaTimeLong;

    //Generic Constrictor
    public XDeltaTime() {

    }


    //Sets the start time
    public void Start() {
        startTime = System.currentTimeMillis();
    }

    //Sets the stop time
    public void Stop() {
        deltaTimeLong = System.currentTimeMillis() - startTime;
    }

    //Returns a double deltaTime in seconds
    public Double deltaTime() {
        return bMath.Clamp((double) (deltaTimeLong / 1000), Double.MIN_VALUE, 100);
    }

    //Returns a long delta time in milliseconds
    public Long deltaTimeMillis() {
        return deltaTimeLong;
    }
}
