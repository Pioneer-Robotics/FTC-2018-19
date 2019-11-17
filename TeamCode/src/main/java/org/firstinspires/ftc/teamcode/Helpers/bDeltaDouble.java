package org.firstinspires.ftc.teamcode.Helpers;

//WIP WIP WIP
public class bDeltaDouble {
    public double value;

    double lastValue;

    public DeltaTime deltaTime = new DeltaTime();

    //Returns the change in value
    public double Delta() {
        return (lastValue - value) / deltaTime.deltaTime();
    }


    public void Assign(Double v) {
        lastValue = value;
        value = v;
    }
}
