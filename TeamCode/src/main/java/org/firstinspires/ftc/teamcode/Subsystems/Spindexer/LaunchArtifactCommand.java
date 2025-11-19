package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchArtifactCommand {
    private final Spindexer spindexer;
    private final Popper popper;

    private int[] launchAngleSequence = new int[3];
    private int target;

    // private boolean reachedFirst, reachedSecond, reachedThird;
    // private boolean isFinished;
    private enum State {
        MOVE_TO_FIRST_LAUNCH,
        LAUNCH_FIRST,
        MOVE_TO_SECOND_LAUNCH,
        LAUNCH_SECOND,
        MOVE_TO_THIRD_LAUNCH,
        LAUNCH_THIRD,
        FINISHED
    }
    private State currentState = State.MOVE_TO_FIRST_LAUNCH;
    private double stateStartTime = 0;
    private final double waitTime = 1000;
    private ElapsedTime timer;

    public LaunchArtifactCommand(Spindexer spindexer, Popper popper) {
        this.spindexer = spindexer;
        this.popper = popper;

        // reachedFirst = false; reachedSecond = false; reachedThird = false;
        // isFinished = false;
    }

    public void start() {
        launchAngleSequence = spindexer.getLaunchAngleSequence();
        target = launchAngleSequence[0];
        currentState = State.MOVE_TO_FIRST_LAUNCH;
        timer = new ElapsedTime();
    }

    public void testPID(TelemetryPacket packet, int target) {
        spindexer.goToAngle(target);

        packet.put("timer ", timer.seconds());
        packet.put("target ", target);
        packet.put("launch sequence ", launchAngleSequence[0] + " " + launchAngleSequence[1] + " " + launchAngleSequence[2]);
    }

    public void update(TelemetryPacket packet) {
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

    public boolean isFinished() {
        return currentState == State.FINISHED;
    }

    public boolean spindexerReachedTarget(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < 10;
    }
}
