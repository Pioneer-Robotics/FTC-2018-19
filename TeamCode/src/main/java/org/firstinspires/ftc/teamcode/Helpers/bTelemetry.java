package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class bTelemetry {
    static OpMode opMode;

    public static void Start(OpMode op) {
        opMode = op;
        Print("Started.");
    }

    //Adds one line without clearing the log
    public static void Print(String message) {
        opMode.telemetry.log().add("bTelemetry: " + message);
        opMode.telemetry.update();
    }

}
