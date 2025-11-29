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

    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};
    private double[] intakePositions = {230, 105, 5};
    private double[][] colorSensorIntakePositions = {{225, 233}, {93, 115}, {354, 13}};

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

        telemetry.addData("hue ", currentHue);
    }
}
