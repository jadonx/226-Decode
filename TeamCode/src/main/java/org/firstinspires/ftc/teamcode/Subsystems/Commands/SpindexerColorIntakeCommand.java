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
    private float[] currentHSV;

    private int alpha;

    private double currentAngle;
    private double targetAngle;

    private ElapsedTime holdingBallTimer;

    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};
    private double[][] intakePositions = {{231, 222, 235}, {107, 93, 115}, {1, 356, 14}};

    private int currentHolderPos;

    public SpindexerColorIntakeCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
    }

    public void start() {
        colorSensorTimer = new ElapsedTime();
        holdingBallTimer = new ElapsedTime();
        currentHolderPos = 0;
    }

    public void update(Telemetry telemetry) {
        if (colorSensorTimer.milliseconds() > colorSensorUpdateTime) {
            currentHSV = spindexer.getHSVRev();
            currentHue = currentHSV[0];

            colorSensorTimer.reset();
        }

        currentAngle = spindexer.getAngle();
        spindexer.goToAngle(intakePositions[currentHolderPos][0]);

        if (isWithinAngleRange(currentAngle, currentHolderPos) && holdingBallTimer.milliseconds() > 1000) {
            currentHolderPos = (currentHolderPos + 1) % 3;
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

    public void resetHolderStatuses() {
        holderStatuses[0] = HolderStatus.NONE;
        holderStatuses[1] = HolderStatus.NONE;
        holderStatuses[2] = HolderStatus.NONE;
    }

    private HolderStatus getHolderStatus(double hue, int alpha) {
        if (hue > 130 && hue < 190 && alpha > 600 && alpha < 900) {
            return HolderStatus.GREEN;
        }
        else if (hue > 210 && hue < 270) {
            return HolderStatus.PURPLE;
        }
        else {
            return HolderStatus.NONE;
        }
    }

    private boolean hasBall(double hue) {
        return (hue > 130 && hue < 190) || (hue > 210 && hue < 270);
    }

    private boolean isWithinAngleRange(double current, int holderPos) {
        if (holderPos == 2) {
            return current > intakePositions[holderPos][1] || current < intakePositions[holderPos][2];
        }
        else {
            return current > intakePositions[holderPos][1] && current < intakePositions[holderPos][2];
        }
    }
}
