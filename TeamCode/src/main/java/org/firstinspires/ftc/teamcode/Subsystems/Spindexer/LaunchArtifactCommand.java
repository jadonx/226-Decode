package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake.Launcher;

public class LaunchArtifactCommand {
    private final Spindexer spindexer;
    private final Popper popper;
    private final Launcher launcher;

    private int[] launchAngleSequence = new int[3];
    private int target;

    private enum State {
        MOVE_TO_FIRST_LAUNCH,
        LAUNCH_FIRST,
        WAIT_AFTER_FIRST_LAUNCH,
        PULL_OUT_POPPER_FIRST,
        MOVE_TO_SECOND_LAUNCH,
        LAUNCH_SECOND,
        WAIT_AFTER_SECOND_LAUNCH,
        PULL_OUT_POPPER_SECOND,
        MOVE_TO_THIRD_LAUNCH,
        LAUNCH_THIRD,
        WAIT_AFTER_THIRD_LAUNCH,
        FINISHED
    }
    private State currentState = State.MOVE_TO_FIRST_LAUNCH;

    private double stateStartTime = 0;
    private final double waitTime = 1000;
    private ElapsedTime timer;

    private double targetVelocity;
    private double targetAngle;

    public LaunchArtifactCommand(Spindexer spindexer, Popper popper, Launcher launcher) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
    }

    public void startPID() {
        launchAngleSequence = spindexer.getLaunchAngleSequence();
        target = launchAngleSequence[0];
        currentState = State.MOVE_TO_FIRST_LAUNCH;
        timer = new ElapsedTime();
    }

    public void updatePID(TelemetryPacket packet) {
        spindexer.goToAngle(target);

        switch (currentState) {
            case MOVE_TO_FIRST_LAUNCH:
                if(spindexerReachedTarget(spindexer.getAngle(), target)) {
                    currentState = State.LAUNCH_FIRST;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case LAUNCH_FIRST:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    currentState = State.MOVE_TO_SECOND_LAUNCH;
                    target = launchAngleSequence[1];
                }
                break;
            case MOVE_TO_SECOND_LAUNCH:
                if(spindexerReachedTarget(spindexer.getAngle(), target)) {
                    currentState = State.LAUNCH_SECOND;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case LAUNCH_SECOND:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    currentState = State.MOVE_TO_THIRD_LAUNCH;
                    target = launchAngleSequence[2];
                }
                break;
            case MOVE_TO_THIRD_LAUNCH:
                if (spindexerReachedTarget(spindexer.getAngle(), target)) {
                    currentState = State.LAUNCH_THIRD;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case LAUNCH_THIRD:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    currentState = State.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }

        packet.put("state ", currentState);
        packet.put("target ", target);
    }

    public void start() {
        launchAngleSequence = spindexer.getLaunchAngleSequence();
        target = launchAngleSequence[0];
        currentState = State.MOVE_TO_FIRST_LAUNCH;
        timer = new ElapsedTime();

        popper.spinPopper();

        double[] targetVelocityAngle = launcher.getVelocityAndAngle();
        targetVelocity = targetVelocityAngle[0]; targetAngle = targetVelocityAngle[1];

        launcher.setVelocity(targetVelocity);
        launcher.setCoverAngle(targetAngle);
    }

    public void update(TelemetryPacket packet) {
        spindexer.goToAngle(target);
        launcher.setVelocity(targetVelocity);
        launcher.setCoverAngle(targetAngle);

        switch (currentState) {
            // WAIT UNTIL SPINDEXER IS AT POSITION
            case MOVE_TO_FIRST_LAUNCH:
                if(spindexerReachedTarget(spindexer.getAngle(), target)) {
                    currentState = State.LAUNCH_FIRST;
                }
                break;
            // IF LAUNCHER AT TARGET VELOCITY, PUSH IN POPPER
            case LAUNCH_FIRST:
                if (launcher.atTargetVelocity(targetVelocity)) {
                    popper.pushInPopper();
                    currentState = State.WAIT_AFTER_FIRST_LAUNCH;
                    stateStartTime = timer.milliseconds();
                }
                break;
            // WAIT AFTER PUSHING IN POPPER FOR LAUNCH, PULL OUT POPPER
            case WAIT_AFTER_FIRST_LAUNCH:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    popper.pushOutPopper();
                    currentState = State.PULL_OUT_POPPER_FIRST;
                    stateStartTime = timer.milliseconds();
                }
                break;
            // WAIT FOR POPPER TO PULL OUT
            case PULL_OUT_POPPER_FIRST:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    currentState = State.MOVE_TO_SECOND_LAUNCH;
                    target = launchAngleSequence[1];
                }
                break;
            case MOVE_TO_SECOND_LAUNCH:
                if(spindexerReachedTarget(spindexer.getAngle(), target)) {
                    currentState = State.LAUNCH_SECOND;
                }
                break;
            case LAUNCH_SECOND:
                if (launcher.atTargetVelocity(targetVelocity)) {
                    popper.pushInPopper();
                    currentState = State.WAIT_AFTER_SECOND_LAUNCH;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case WAIT_AFTER_SECOND_LAUNCH:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    popper.pushOutPopper();
                    currentState = State.PULL_OUT_POPPER_SECOND;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case PULL_OUT_POPPER_SECOND:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    currentState = State.MOVE_TO_THIRD_LAUNCH;
                    target = launchAngleSequence[2];
                }
                break;
            case MOVE_TO_THIRD_LAUNCH:
                if(spindexerReachedTarget(spindexer.getAngle(), target)) {
                    currentState = State.LAUNCH_THIRD;
                }
                break;
            case LAUNCH_THIRD:
                if (launcher.atTargetVelocity(targetVelocity)) {
                    popper.pushInPopper();
                    currentState = State.WAIT_AFTER_THIRD_LAUNCH;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case WAIT_AFTER_THIRD_LAUNCH:
                if (timer.milliseconds() - stateStartTime > waitTime) {
                    popper.pushOutPopper();
                    popper.stopPopper();
                    currentState = State.FINISHED;
                    stateStartTime = timer.milliseconds();
                }
                break;
            case FINISHED:
                popper.deactivatePopper();
                launcher.stopLauncher();
                break;
        }

        packet.put("state ", currentState);
        packet.put("target ", target);
    }

    public boolean isFinished() {
        return currentState == State.FINISHED;
    }

    public boolean spindexerReachedTarget(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < 5;
    }
}
