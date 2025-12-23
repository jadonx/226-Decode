package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class LaunchCommand {
    private final Spindexer spindexer;
    private final Popper popper;

    private final Launcher launcher;

    private enum State {
        GO_TO_SHOOTING_START,
        SHOOT_BALLS,
        FINISH
    }
    private State currentState;

    public LaunchCommand(Spindexer spindexer, Popper popper, Launcher launcher) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
    }

    public void start() {
        spindexer.setTargetAngle(spindexer.getTargetAngle());
        popper.pushOutPopper();
        popper.spinPopper();
        currentState = State.GO_TO_SHOOTING_START;
    }

    public void update() {
        switch (currentState) {
            case GO_TO_SHOOTING_START:
                if (spindexer.atTargetAngle()) {
                    popper.pushInPopper();
                    currentState = State.SHOOT_BALLS;
                }
                break;
            case SHOOT_BALLS:
                break;
            case FINISH:
                break;
        }
    }
}