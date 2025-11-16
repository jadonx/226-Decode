package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchArtifactCommand {
    private final Spindexer spindexer;
    private final Popper popper;

    private int[] launchAngleSequence = new int[3];
    private ElapsedTime timer;

    private boolean reachedFirst, reachedSecond, reachedThird;
    private boolean isFinished;

    private int target;

    public LaunchArtifactCommand(Spindexer spindexer, Popper popper) {
        this.spindexer = spindexer;
        this.popper = popper;
        timer = new ElapsedTime();

        reachedFirst = false; reachedSecond = false; reachedThird = false;
        isFinished = false;
    }

    public void start() {
        launchAngleSequence = spindexer.getLaunchAngleSequence();
        target = launchAngleSequence[0];
    }

    public void update(TelemetryPacket packet) {
        spindexer.goToAngle(target);

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

        packet.put("timer ", timer.seconds());
        packet.put("target ", target);
        packet.put("reached first ", reachedFirst);
        packet.put("reached second ", reachedSecond);
        packet.put("reached third ", reachedThird);
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean spindexerReachedTarget(double currentAngle, double targetAngle) {
        return Math.abs(currentAngle - targetAngle) < 10;
    }
}
