package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class SpindexerColorIntakeCommand {
    private final Spindexer spindexer;

    private ElapsedTime colorSensorTimer;
    private int colorSensorUpdateTime = 10;

    private double currentHue;
    private double currentAngle;
    private int targetAngle;

    private ElapsedTime holdingBallTimer;
    private int holdingBallThreshold = 150; // Must hold ball for 800ms to move to next holder

    private int[] intakePositions;

    private enum State {
        WAIT_AT_FIRST_HOLDER,
        WAIT_AT_SECOND_HOLDER,
        WAIT_AT_THIRD_HOLDER,
        GO_TO_LAUNCH,
        FINISHED
    }
    private State currentState;

    public SpindexerColorIntakeCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        intakePositions = spindexer.getIntakePositions();
    }

    public void start() {
        colorSensorTimer = new ElapsedTime();
        holdingBallTimer = new ElapsedTime();
        currentState = State.WAIT_AT_FIRST_HOLDER;
        targetAngle = intakePositions[0];
    }

    public void update(Telemetry telemetry) {
        // Color sensor readings
        if (colorSensorTimer.milliseconds() > colorSensorUpdateTime) {
            currentHue = spindexer.getHSVRev()[0];
            colorSensorTimer.reset();
        }

        spindexer.goToAngle(targetAngle);
        currentAngle = spindexer.getWrappedAngle();

        if (!hasBall(currentHue)) {
            holdingBallTimer.reset();
        }

        switch (currentState) {
            case WAIT_AT_FIRST_HOLDER:
                if (withinIntakeAngle(currentAngle, 0) && holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(0, getBallColor(currentHue));
                    targetAngle = intakePositions[1];
                    holdingBallTimer.reset();
                    currentState = State.WAIT_AT_SECOND_HOLDER;
                }
                break;
            case WAIT_AT_SECOND_HOLDER:
                if (withinIntakeAngle(currentAngle, 1) && holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(1, getBallColor(currentHue));
                    targetAngle = intakePositions[2];
                    holdingBallTimer.reset();
                    currentState = State.WAIT_AT_THIRD_HOLDER;
                }
                break;
            case WAIT_AT_THIRD_HOLDER:
                if (withinIntakeAngle(currentAngle, 2) && holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(2, getBallColor(currentHue));
                    targetAngle = spindexer.getLaunchPositions()[0];
                    currentState = State.GO_TO_LAUNCH;
                }
                break;
            case GO_TO_LAUNCH:
                if (spindexer.reachedTarget(currentAngle, targetAngle)) {
                    currentState = State.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }





    /*
    public void update(Telemetry telemetry) {
        if (colorSensorTimer.milliseconds() > colorSensorUpdateTime) {
            currentHSV = spindexer.getHSVRev();
            currentHue = currentHSV[0];

            colorSensorTimer.reset();
        }

        currentAngle = spindexer.getWrappedAngle();
        spindexer.goToAngle(intakePositions[currentHolderPos][0]);

        if (isWithinAngleRange(currentAngle, currentHolderPos) && holdingBallTimer.milliseconds() > 1000) {
            currentHolderPos = (currentHolderPos + 1) % 3;
            spindexer.rebaseContinuousAngle();
            holdingBallTimer.reset();
        }

        if (!hasBall(currentHue)) {
            holdingBallTimer.reset();
        }

        telemetry.addData("hue ", currentHue);
        telemetry.addData("currentAngle ", currentAngle);
        telemetry.addData("target intake position ", intakePositions[currentHolderPos][0]);
        telemetry.addData("currentHolderPos ", currentHolderPos);
        telemetry.addData("timer ", holdingBallTimer.milliseconds());
    }
     */

    /*
    public void update(TelemetryPacket packet) {
        if (colorSensorTimer.milliseconds() > colorSensorUpdateTime) {
            currentHSV = spindexer.getHSVRev();
            currentHue = currentHSV[0];
            alpha = spindexer.getAlpha();

            colorSensorTimer.reset();
        }

        currentAngle = spindexer.getAngle();

        // CHECKING COLOR IF SPINDEXER AT INTAKE HOLDER ANGLES
        if (currentAngle > intakePositions[0][1] && currentAngle < intakePositions[0][2]) {
            holderStatuses[0] = getHolderStatus(currentHue, alpha);
        }
        else if (currentAngle > intakePositions[1][1] && currentAngle < intakePositions[1][2]) {
            holderStatuses[1] = getHolderStatus(currentHue, alpha);
        }
        else if (currentAngle > intakePositions[2][1] || currentAngle < intakePositions[2][2]) {
            holderStatuses[2] = getHolderStatus(currentHue, alpha);
        }

        // GOING TO EMPTY HOLDERS
        if (holderStatuses[0] == HolderStatus.NONE) {
            targetAngle = intakePositions[0][0];
            packet.put("GOING TO HOLDER 0", null);
        }
        else if (holderStatuses[1] == HolderStatus.NONE) {
            targetAngle = intakePositions[1][0];
            packet.put("GOING TO HOLDER 1", null);
        }
        else if (holderStatuses[2] == HolderStatus.NONE) {
            targetAngle = intakePositions[2][0];
            packet.put("GOING TO HOLDER 2", null);
        }
        else {
            targetAngle = 49;
        }

        spindexer.goToAngleSlow(targetAngle);

        packet.put("hue ", currentHue);
        packet.put("alpha ", alpha);
        packet.put("holder 0 ", holderStatuses[0]);
        packet.put("holder 1 ", holderStatuses[1]);
        packet.put("holder 2 ", holderStatuses[2]);
    }
     */

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