package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Popper;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class LaunchArtifactCommand {
    private final Spindexer spindexer;
    private final Popper popper;
    private final Launcher launcher;
    private final MecanumDrive drive;

    private int[] launchAngleSequence = new int[3];
    private int target;

    private enum State {
        MOVE_TO_FIRST_LAUNCH,
        WAIT_AFTER_AT_FIRST,
        LAUNCH_FIRST,
        WAIT_AFTER_FIRST_LAUNCH,
        PULL_OUT_POPPER_FIRST,
        MOVE_TO_SECOND_LAUNCH,
        WAIT_AFTER_AT_SECOND,
        LAUNCH_SECOND,
        WAIT_AFTER_SECOND_LAUNCH,
        PULL_OUT_POPPER_SECOND,
        MOVE_TO_THIRD_LAUNCH,
        WAIT_AFTER_AT_THIRD,
        LAUNCH_THIRD,
        WAIT_AFTER_THIRD_LAUNCH,
        FINISHED
    }
    private State currentState = State.MOVE_TO_FIRST_LAUNCH;

    private double stateStartTime = 0;
    private final double popperPushInWait = 400; // Wait time for popper to move in
    private final double popperPullOutWait = 400;
    private final double spindexerWaitTime = 750;
    private ElapsedTime timer;

    private double atLaunchTimer = 0;
    private boolean atLaunchPosition = false;

    private double targetVelocity;
    private double targetAngle;

    public LaunchArtifactCommand(Spindexer spindexer, Popper popper, Launcher launcher, MecanumDrive drive_roadrunner) {
        this.spindexer = spindexer;
        this.popper = popper;
        this.launcher = launcher;
        this.drive = drive_roadrunner;
    }

    public void start() {
        launchAngleSequence = spindexer.getLaunchPositionsDyanmic();
        target = launchAngleSequence[0];
        currentState = State.MOVE_TO_FIRST_LAUNCH;

        timer = new ElapsedTime();

        popper.spinPopper();

        double[] targetVelocityAngle = launcher.getVelocityAndAngle(drive.localizer.getPose());
        targetVelocity = targetVelocityAngle[0]; targetAngle = targetVelocityAngle[1];

        launcher.setVelocity(targetVelocity);
        launcher.setCoverAngle(targetAngle);
    }

    public void autoColorStart(Spindexer.HolderStatus[] motifPattern) {
        launchAngleSequence = spindexer.getLaunchPositionsColor(motifPattern);
        target = launchAngleSequence[0];
        currentState = State.MOVE_TO_FIRST_LAUNCH;

        timer = new ElapsedTime();

        popper.spinPopper();

        double[] targetVelocityAngle = launcher.getVelocityAndAngle(drive.localizer.getPose());
        targetVelocity = targetVelocityAngle[0]; targetAngle = targetVelocityAngle[1];

        launcher.setVelocity(targetVelocity);
        launcher.setCoverAngle(targetAngle);
    }

    public void update(Telemetry telemetry) {
        if (target != 999) {
            spindexer.goToAngle(target);
        }
        launcher.setVelocity(targetVelocity);
        launcher.setCoverAngle(targetAngle);

        switch (currentState) {
            // WAIT UNTIL SPINDEXER IS AT POSITION
            case MOVE_TO_FIRST_LAUNCH:
                if (target != 999) {
                    if (spindexer.reachedTarget(spindexer.getWrappedAngle(), target)) {
                        stateStartTime = timer.milliseconds();
                        currentState = State.WAIT_AFTER_AT_FIRST;
                    }
                }
                else {
                    target = launchAngleSequence[1];
                    currentState = State.MOVE_TO_SECOND_LAUNCH;
                }
                break;
            case WAIT_AFTER_AT_FIRST:
                if (timer.milliseconds() - stateStartTime > spindexerWaitTime) {
                    currentState = State.LAUNCH_FIRST;
                }
                break;
            // IF LAUNCHER AT TARGET VELOCITY, PUSH IN POPPER
            case LAUNCH_FIRST:
                if (launcher.atTargetVelocity(targetVelocity)) {
                    popper.pushInPopper();
                    stateStartTime = timer.milliseconds();
                    currentState = State.WAIT_AFTER_FIRST_LAUNCH;
                }
                break;
            // WAIT AFTER PUSHING IN POPPER FOR LAUNCH, PULL OUT POPPER
            case WAIT_AFTER_FIRST_LAUNCH:
                if (timer.milliseconds() - stateStartTime > popperPushInWait) {
                    popper.pushOutPopper();
                    stateStartTime = timer.milliseconds();
                    currentState = State.PULL_OUT_POPPER_FIRST;
                }
                break;
            case PULL_OUT_POPPER_FIRST:
                if (timer.milliseconds() - stateStartTime > popperPullOutWait) {
                    target = launchAngleSequence[1];
                    currentState = State.MOVE_TO_SECOND_LAUNCH;
                }
                break;
            case MOVE_TO_SECOND_LAUNCH:
                if (target != 999) {
                    if (spindexer.reachedTarget(spindexer.getWrappedAngle(), target)) {
                        stateStartTime = timer.milliseconds();
                        currentState = State.WAIT_AFTER_AT_SECOND;
                    }
                }
                else {
                    target = launchAngleSequence[2];
                    currentState = State.MOVE_TO_THIRD_LAUNCH;
                }
                break;
            case WAIT_AFTER_AT_SECOND:
                if (timer.milliseconds() - stateStartTime > spindexerWaitTime) {
                    currentState = State.LAUNCH_SECOND;
                }
                break;
            case LAUNCH_SECOND:
                if (launcher.atTargetVelocity(targetVelocity)) {
                    popper.pushInPopper();
                    stateStartTime = timer.milliseconds();
                    currentState = State.WAIT_AFTER_SECOND_LAUNCH;
                }
                break;
            case WAIT_AFTER_SECOND_LAUNCH:
                if (timer.milliseconds() - stateStartTime > popperPushInWait) {
                    popper.pushOutPopper();
                    stateStartTime = timer.milliseconds();
                    currentState = State.PULL_OUT_POPPER_SECOND;
                }
                break;
            case PULL_OUT_POPPER_SECOND:
                if (timer.milliseconds() - stateStartTime > popperPullOutWait) {
                    target = launchAngleSequence[2];
                    currentState = State.MOVE_TO_THIRD_LAUNCH;
                }
                break;
            case MOVE_TO_THIRD_LAUNCH:
                if (target != 999) {
                    if (spindexer.reachedTarget(spindexer.getWrappedAngle(), target)) {
                        stateStartTime = timer.milliseconds();
                        currentState = State.WAIT_AFTER_AT_THIRD;
                    }
                }
                else {
                    currentState = State.FINISHED;
                }
                break;
            case WAIT_AFTER_AT_THIRD:
                if (timer.milliseconds() - stateStartTime > spindexerWaitTime) {
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
                if (timer.milliseconds() - stateStartTime > popperPushInWait) {
                    popper.deactivatePopper();
                    launcher.stopLauncher();
                    spindexer.resetHolderStatuses();
                    currentState = State.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }

        telemetry.addData("spindexer target ", target);
        telemetry.addData("spindexer at target ", spindexer.reachedTarget(spindexer.getWrappedAngle(), target));
        telemetry.addData("at target velocity ", launcher.atTargetVelocity(targetVelocity));
        telemetry.addData("launch sequence ", launchAngleSequence[0] + " " + launchAngleSequence[1] + " " + launchAngleSequence[2]);
        telemetry.addData("current state ", currentState);
    }

    public boolean isFinished() {
        return currentState == State.FINISHED;
    }

    public void stop() {
        popper.deactivatePopper();
        launcher.stopLauncher();
    }
}