package org.firstinspires.ftc.teamcode.Commands;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class ColorIntakeCommand {
    private final Spindexer spindexer;

    private ElapsedTime colorSensorTimer;
    private int colorSensorUpdateTime = 10;

    private double currentHue;
    private double currentAngle;
    private int targetAngle;

    private ElapsedTime holdingBallTimer;
    private int holdingBallThreshold = 50;

    private int[] intakePositions;

    private enum State {
        WAIT_AT_FIRST_HOLDER,
        WAIT_AT_SECOND_HOLDER,
        WAIT_AT_THIRD_HOLDER,
        GO_TO_LAUNCH,
        FINISHED
    }
    private State currentState;

    public ColorIntakeCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        intakePositions = spindexer.getIntakePositions();
    }

    public void start() {
        colorSensorTimer = new ElapsedTime();
        holdingBallTimer = new ElapsedTime();

        spindexer.resetHolderStatuses();
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE, 0.4);
        spindexer.setTargetAngle(intakePositions[0]);
        currentState = State.WAIT_AT_FIRST_HOLDER;
    }

    public void startAuto() {
        spindexer.setHolderStatus(0, Spindexer.HolderStatus.PURPLE);
        spindexer.setHolderStatus(1, Spindexer.HolderStatus.PURPLE);
        spindexer.setHolderStatus(2, Spindexer.HolderStatus.GREEN);
    }

    public void update() {
        // Color sensor readings
        if (colorSensorTimer.milliseconds() > colorSensorUpdateTime) {
            currentHue = spindexer.getHSVRev()[0];
            colorSensorTimer.reset();
        }
        spindexer.update();
        currentAngle = spindexer.getWrappedAngle();

        if (!hasBall(currentHue)) {
            holdingBallTimer.reset();
        }

        switch (currentState) {
            case WAIT_AT_FIRST_HOLDER:
                if (withinIntakeAngle(currentAngle, 0) && holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(0, getBallColor(currentHue));
                    spindexer.setTargetAngle(intakePositions[1]);
                    holdingBallTimer.reset();
                    currentState = State.WAIT_AT_SECOND_HOLDER;
                }
                break;
            case WAIT_AT_SECOND_HOLDER:
                if (withinIntakeAngle(currentAngle, 1) && holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(1, getBallColor(currentHue));
                    spindexer.setTargetAngle(intakePositions[2]);
                    holdingBallTimer.reset();
                    currentState = State.WAIT_AT_THIRD_HOLDER;
                }
                break;
            case WAIT_AT_THIRD_HOLDER:
                if (withinIntakeAngle(currentAngle, 2) && holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(2, getBallColor(currentHue));
                    currentState = State.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }

    private Spindexer.HolderStatus getBallColor(double hue) {
        if (hue > 130 && hue < 190) return Spindexer.HolderStatus.GREEN;
        if (hue > 210 && hue < 270) return Spindexer.HolderStatus.PURPLE;
        else return Spindexer.HolderStatus.NONE;
    }

    private boolean hasBall(double hue) {
        return ((hue > 130 && hue < 190) || (hue > 210 && hue < 270)) && spindexer.getColorDistance() < 2;
    }

    private boolean withinIntakeAngle(double current, int currentHolder) {
        if (currentHolder == 2) {
            // For intake angle 1
            return current > 345 || current < 17;
        }
        else return current > (intakePositions[currentHolder]-12) && current < (intakePositions[currentHolder]+12);
    }
}