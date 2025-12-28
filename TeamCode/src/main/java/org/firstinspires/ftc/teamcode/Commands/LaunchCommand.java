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

    private enum State {
        PREPARE_TO_SHOOT,
        SHOOTING,
        FINISH
    }
    private State currentState;

    private ElapsedTime stateTimer;
    private final int coverRunTime = 500;

    public LaunchCommand(Spindexer spindexer, Popper popper, Launcher launcher, PinPoint pinpoint) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
        this.pinpoint = pinpoint;
    }

    public void start() {
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE, 0.4);
        spindexer.setTargetAngle(spindexer.getTargetAngle());

        popper.pushInPopper();
        popper.spinPopper();
        // launcher.setTargetVelocity(pinpoint.getDistanceToGoal());
        launcher.setTargetVelocity(1580);
        launcher.setTargetCoverAngle(0);

        stateTimer.reset();

        currentState = State.PREPARE_TO_SHOOT;
    }

    public void update() {
        launcher.update();
        spindexer.update();

        switch (currentState) {
            case PREPARE_TO_SHOOT:
                if (spindexer.atTargetAngle(3) && launcher.atTargetVelocity(30) && stateTimer.milliseconds() > coverRunTime) {
                    spindexer.setMode(Spindexer.SpindexerMode.LAUNCH_MODE, 0.25);
                    spindexer.setTargetAngle(spindexer.getUnwrappedAngle() + 360);
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
}