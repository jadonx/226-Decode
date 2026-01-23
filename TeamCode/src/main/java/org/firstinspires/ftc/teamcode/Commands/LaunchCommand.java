package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.RoadRunnerPinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class LaunchCommand {
    private final Spindexer spindexer;
    private final Popper popper;
    private final Launcher launcher;
    private final RoadRunnerPinPoint pinpoint;

    public enum State {
        PRIME_SHOOTER,
        PREPARE_TO_SHOOT,
        SHOOTING,
        FINISH
    }
    private State currentState;

    private double spindexerSpeed;

    public LaunchCommand(Spindexer spindexer, Popper popper, Launcher launcher, RoadRunnerPinPoint pinpoint, Intake intake) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
        this.pinpoint = pinpoint;
    }

    public void start() {
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
        spindexer.setTargetAngle(spindexer.getTargetAngle());

        spindexerSpeed = 0.2; // Speed of spindexer while launching

        popper.pushInPopper();
        popper.setTargetVelocity(1800);

        // launcher.calculateTargetVelocity(pinpoint.getDistanceToGoal());

        currentState = State.PRIME_SHOOTER;
    }

    public void update() {
        double distanceToGoal = pinpoint.getDistanceToGoal();

        if (distanceToGoal > 130) {
            spindexerSpeed = 0.15;
        }
        else {
            spindexerSpeed = 0.2;
        }

        launcher.calculateTargetVelocity(distanceToGoal);
        launcher.update();
        spindexer.update();

        switch (currentState) {
            case PRIME_SHOOTER:
                break;
            case PREPARE_TO_SHOOT:
                if (launcher.atTargetVelocity(20) && popper.atTargetVelocity(20)) {
                    spindexer.setSpeed(spindexerSpeed);
                    spindexer.setMode(Spindexer.SpindexerMode.LAUNCH_MODE);
                    currentState = State.SHOOTING;
                }
                break;
            case SHOOTING:
                if (spindexer.atTargetAngle(0)) {
                    // ^Threshold doesn't matter because we only check if we've passed 360 degrees
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

    public void startShootingSequence() {
        // Transition from priming to shooting
        if (currentState == State.PRIME_SHOOTER) {
            currentState = State.PREPARE_TO_SHOOT;
        }
    }
}