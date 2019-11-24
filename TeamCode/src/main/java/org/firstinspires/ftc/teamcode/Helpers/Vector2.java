package org.firstinspires.ftc.teamcode.Helpers;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double _x, double _y) {
        x = _x;
        y = _y;
    }

    //Creates a Vector2 from an angle in DEGREES
    public Vector2(double degrees) {

    }

    public void add(Vector2 value) {
        x += value.x;
        y += value.y;
    }

    public void subtract(Vector2 value) {
        x -= value.x;
        y -= value.y;
    }

}
