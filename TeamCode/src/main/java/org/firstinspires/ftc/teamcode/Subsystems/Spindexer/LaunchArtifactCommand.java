package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchArtifactCommand {
    private final Spindexer spindexer;

    private int[] launchAngleSequence = new int[3];
    private ElapsedTime timer;

    private boolean reachedFirst, reachedSecond, reachedThird;
    private boolean isFinished;

    private int target;

    public LaunchArtifactCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        timer = new ElapsedTime();

        reachedFirst = false; reachedSecond = false; reachedThird = false;
        isFinished = false;
    }

    public void start() {
        launchAngleSequence = spindexer.getLaunchAngleSequence();
        target = launchAngleSequence[0];
    }

    public void update(Telemetry telemetry) {
        spindexer.goToAngle(target);

        if (!reachedFirst && spindexerReachedTarget(spindexer.getAngle(), target)) {
            timer.reset();
            reachedFirst = true;
        }

        if (reachedFirst && !reachedSecond && timer.seconds() > 3) {
            target = launchAngleSequence[1];
        }

        if (!reachedSecond && spindexerReachedTarget(spindexer.getAngle(), target)) {
            timer.reset();
            reachedSecond = true;
        }

        if (reachedSecond && !reachedThird && timer.seconds() > 3) {
            target = launchAngleSequence[2];
        }

        if (!reachedThird && spindexerReachedTarget(spindexer.getAngle(), target)) {
            timer.reset();
            reachedThird = true;
        }

        if (reachedThird && timer.seconds() > 3) {
            isFinished = true;
        }

        telemetry.addData("timer ", timer.seconds());
        telemetry.addData("reached first ", reachedFirst);
        telemetry.addData("reached second ", reachedSecond);
        telemetry.addData("reached third ", reachedThird);
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean spindexerReachedTarget(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < 10;
    }
}
