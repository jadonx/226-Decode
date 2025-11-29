package org.firstinspires.ftc.teamcode.Subsystems.Commands;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class SpindexerColorSensorIntakeCommand {
    private final Spindexer spindexer;

    private ElapsedTime colorSensorTimer;
    private int colorSensorUpdateTime = 10;

    private double currentHue;
    private double currentAngle;

    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};
    private double[][] intakePositions = {{230, 225, 233}, {105, 93, 115}, {5, 354, 13}};

    public SpindexerColorSensorIntakeCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
    }

    public void start() {
        colorSensorTimer = new ElapsedTime();
    }

    public void update(Telemetry telemetry) {
        if (colorSensorTimer.milliseconds() > colorSensorUpdateTime) {
            currentHue = spindexer.getHue();
            colorSensorTimer.reset();
        }

        currentAngle = spindexer.getAngle();

        // CHECKING COLOR IF SPINDEXER AT INTAKE HOLDER ANGLES
        if (currentAngle > intakePositions[0][1] && currentAngle < intakePositions[0][2]) {
            holderStatuses[0] = getHolderStatus(currentHue);
        }
        else if (currentAngle > intakePositions[1][1] && currentAngle < intakePositions[1][2]) {
            holderStatuses[1] = getHolderStatus(currentHue);
        }
        else if (currentAngle > intakePositions[2][1] && currentAngle < intakePositions[2][2]) {
            holderStatuses[2] = getHolderStatus(currentHue);
        }

        // GOING TO EMPTY HOLDERS
        if (holderStatuses[0] == HolderStatus.NONE) {
            spindexer.goToAngle(intakePositions[0][0]);
        }
        else if (holderStatuses[1] == HolderStatus.NONE) {
            spindexer.goToAngle(intakePositions[1][0]);
        }
        else if (holderStatuses[2] == HolderStatus.NONE) {
            spindexer.goToAngle(intakePositions[2][0]);
        }

        telemetry.addData("hue ", currentHue);
        telemetry.addData("holder 0 ", holderStatuses[0]);
        telemetry.addData("holder 1 ", holderStatuses[1]);
        telemetry.addData("holder 2 ", holderStatuses[2]);
    }

    private HolderStatus getHolderStatus(double hue) {
        if (hue > 150 && hue < 170) {
            return HolderStatus.GREEN;
        }
        else if (hue > 230 && hue < 260) {
            return HolderStatus.PURPLE;
        }
        else {
            return HolderStatus.NONE;
        }
    }
}
