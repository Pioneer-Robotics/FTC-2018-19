package org.firstinspires.ftc.teamcode.Helpers;

//Used for finding the change in a doubles and integers
public class bDeltaValue {

    public double value;

    double lastValue;

    public DeltaTime deltaTime = new DeltaTime();

    //Returns the change in value
    public double delta() {
        return (lastValue - value) / deltaTime.deltaTime();
    }


    public void Assign(Double v) {
        deltaTime.Stop();
        lastValue = value;
        value = v;
        deltaTime.Start();
    }

    public void Assign(int v) {
        deltaTime.Stop();
        lastValue = value;
        value = v;
        deltaTime.Start();
    }
}
