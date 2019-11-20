package org.firstinspires.ftc.teamcode.Helpers;

import android.text.format.Time;

public class bTimer {

    public double endTime;


    public void Start(double duration) {
        endTime = System.currentTimeMillis() + duration * 1000;

    }
}
