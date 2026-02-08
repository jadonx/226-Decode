package org.firstinspires.ftc.teamcode.Commands;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

public class ColorIntakeCommand {
    private final Spindexer spindexer;
    private final ColorSensor colorSensor;

    private int[] intakePositions;

    private ElapsedTime holdingBallTimer;
    private final int holdingBallThreshold = 25;

    public enum State {
        WAIT_AT_FIRST_HOLDER,
        WAIT_AT_SECOND_HOLDER,
        WAIT_AT_THIRD_HOLDER,
        FINISHED
    }
    private State currentState;

    public ColorIntakeCommand(Spindexer spindexer, ColorSensor colorSensor) {
        this.spindexer = spindexer;
        this.colorSensor = colorSensor;
        intakePositions = spindexer.getIntakePositions();
    }

    public void start() {
        holdingBallTimer = new ElapsedTime();
        holdingBallTimer.reset();

        spindexer.resetHolderStatuses();
        spindexer.setMode(Spindexer.SpindexerMode.INTAKE_MODE);
        spindexer.setSpeed(0.4);
        spindexer.setTargetAngle(intakePositions[0]);
        currentState = State.WAIT_AT_FIRST_HOLDER;
    }

    public void update() {
        spindexer.update();

        if (spindexer.atTargetAngle(5)) {
            colorSensor.update();

            if (!colorSensor.hasBall()) {
                holdingBallTimer.reset();
            }
        }
        else {
            holdingBallTimer.reset();
        }

        switch (currentState) {
            case WAIT_AT_FIRST_HOLDER:
                if (holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(0, colorSensor.getCurrentBall());
                    spindexer.setTargetAngle(intakePositions[1]);
                    holdingBallTimer.reset();
                    currentState = State.WAIT_AT_SECOND_HOLDER;
                }
                break;
            case WAIT_AT_SECOND_HOLDER:
                if (holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(1, colorSensor.getCurrentBall());
                    spindexer.setTargetAngle(intakePositions[2]);
                    holdingBallTimer.reset();
                    currentState = State.WAIT_AT_THIRD_HOLDER;
                }
                break;
            case WAIT_AT_THIRD_HOLDER:
                if (holdingBallTimer.milliseconds() > holdingBallThreshold) {
                    spindexer.setHolderStatus(2, colorSensor.getCurrentBall());
                    currentState = State.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }
    }

    public State getCurrentState() {
        return currentState;
    }
}