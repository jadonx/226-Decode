package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class LaunchCommand {
    private final Spindexer spindexer;
    private final Popper popper;
    private final Launcher launcher;
    private final PinPoint pinpoint;

    public enum State {
        PREPARE_TO_SHOOT,
        SHOOTING,
        FINISH
    }
    private State currentState;

    public LaunchCommand(Spindexer spindexer, Popper popper, Launcher launcher, PinPoint pinpoint) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
        this.pinpoint = pinpoint;
    }

    public void start(double distance) {
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE, 0.4);
        spindexer.setTargetAngle(spindexer.getTargetAngle());

        popper.pushInPopper();
        popper.setTargetVelocity(1800);

        launcher.calculateTargetVelocity(pinpoint.getDistanceToGoal());

        currentState = State.PREPARE_TO_SHOOT;
    }

    public void update() {
        launcher.update();
        spindexer.update();
        popper.update();

        switch (currentState) {
            case PREPARE_TO_SHOOT:
                if (launcher.atTargetVelocity(20) && popper.atTargetVelocity(100)) {
                    spindexer.setMode(Spindexer.SpindexerMode.LAUNCH_MODE, 0.2);
                    spindexer.setTargetAngle(spindexer.getUnwrappedAngle() + 370);
                    currentState = State.SHOOTING;
                }
                break;
            case SHOOTING:
                if (spindexer.atTargetAngle(10)) {
                    currentState = State.FINISH;
                }
                break;
            case FINISH:
                break;
        }
    }

    public boolean isFinished() {
        return currentState == State.FINISH;
    }

    public State getCurrentState() {
        return currentState;
    }
}