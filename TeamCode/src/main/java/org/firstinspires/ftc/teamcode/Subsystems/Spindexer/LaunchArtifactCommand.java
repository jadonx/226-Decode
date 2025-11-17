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
        LAUNCH_THIRD
    }
    private State currentState = State.MOVE_TO_FIRST_LAUNCH;
    private long stateStartTime = 0;
    private final long waitTime = 1000;
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
        timer = new ElapsedTime();
    }

    public void testPID(TelemetryPacket packet) {
        spindexer.goToAngle(target);

        /*
        if (!reachedFirst && spindexerReachedTarget(spindexer.getAngle(), target) && target == launchAngleSequence[0]) {
            timer.reset();
            reachedFirst = true;
        }
        else if (reachedFirst && !reachedSecond && timer.seconds() > 3) {
            target = launchAngleSequence[1];
        }
        else if (!reachedSecond && spindexerReachedTarget(spindexer.getAngle(), target) && target == launchAngleSequence[1]) {
            timer.reset();
            reachedSecond = true;
        }
        else if (reachedSecond && !reachedThird && timer.seconds() > 3) {
            target = launchAngleSequence[2];
        }
        else if (!reachedThird && spindexerReachedTarget(spindexer.getAngle(), target) && target == launchAngleSequence[2]) {
            timer.reset();
            reachedThird = true;
        }
        else if (reachedThird && timer.seconds() > 3) {
            isFinished = true;
        }
         */

        packet.put("timer ", timer.seconds());
        packet.put("target ", target);
        packet.put("launch sequence ", launchAngleSequence[0] + " " + launchAngleSequence[1] + " " + launchAngleSequence[2]);
    }

    public void update(TelemetryPacket packet) {
    }

    public boolean isFinished() {
        return false;
    }

    public boolean spindexerReachedTarget(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < 10;
    }
}
