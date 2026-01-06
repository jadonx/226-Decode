package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.PinPoint;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class LaunchCommand {
    private final Spindexer spindexer;
    private final Popper popper;
    private final Launcher launcher;
    private final PinPoint pinpoint;
    private final Intake intake;

    public enum State {
        PREPARE_TO_SHOOT,
        SHOOTING,
        FINISH
    }
    private State currentState;

    private double spindexerSpeed;

    public LaunchCommand(Spindexer spindexer, Popper popper, Launcher launcher, PinPoint pinpoint, Intake intake) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
        this.pinpoint = pinpoint;
        this.intake = intake;
    }

    public void start() {
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
        spindexer.setTargetAngle(spindexer.getTargetAngle());

        this.spindexerSpeed = 0.2; // Speed of spindexer while launching

        popper.pushInPopper();
        popper.setTargetVelocity(1800);

        launcher.calculateTargetVelocity(pinpoint.getDistanceToGoal());

        currentState = State.PREPARE_TO_SHOOT;
    }

    public void startAuto() {
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
        spindexer.setTargetAngle(spindexer.getIntakePositions()[0]);
        this.spindexerSpeed = 0.15; // Speed of spindexer while launching
        popper.pushInPopper();
        popper.setTargetVelocity(1800);
        launcher.setTargetVelocity(1300);
        currentState = State.PREPARE_TO_SHOOT;
    }

    public void update() {
        launcher.update();
        spindexer.update();
        // intake.runIntake(0.15F);

        switch (currentState) {
            case PREPARE_TO_SHOOT:
                if (launcher.atTargetVelocity(20) && popper.atTargetVelocity(50)) {
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
}