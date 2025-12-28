package org.firstinspires.ftc.teamcode.Subsystems;

public class RobotTelemetry {
    private Spindexer spindexer;
    private Launcher launcher;
    private PinPoint pinpoint;

    public RobotTelemetry(Spindexer spindexer, Launcher launcher, PinPoint pinpoint) {
        this.spindexer = spindexer;
        this.launcher = launcher;
        this.pinpoint = pinpoint;
    }
}
