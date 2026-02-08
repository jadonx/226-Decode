package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class Spindexer {
    private CRServo spindexerServo;
    private SpindexerEncoder spindexerEncoder;

    private double currentAngle;
    public static double targetAngle = 0;
    // private double kP = 0.002, kS = 0.04;
    public static double kP = 0.01, kI = 0, kD = 0.005, kF = 0;
    private PIDFController pid = new PIDFController(kP, kI, kD, kF);;

    public enum SpindexerMode {
        INTAKE_MODE,
        LAUNCH_MODE
    }
    SpindexerMode spindexerMode;
    private double spindexerSpeed = 0.4;
    private boolean isUnjamming;
    private boolean isUnjammingCW;

    private double lastAngle = 0;
    private double unwrappedAngle = 0;

    private int[] intakePositions = {114, 355, 233};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};
    private HolderStatus[] motifPattern = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
        spindexerMode = SpindexerMode.INTAKE_MODE;
    }

    public void update() {
        currentAngle = spindexerEncoder.getWrappedAngle();
        updateUnwrappedAngle();

        if (isUnjamming) {
            if (isUnjammingCW) {
                spindexerServo.setPower(-0.3);
            }
            else {
                spindexerServo.setPower(0.3);
            }
        }
        else if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            updateIntakeMode();
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            updateLaunchMode();
        }
    }

    private void updateIntakeMode() {
        double error = calculateError();
        // pid.setPIDF(kP, kI, kD, kF);

        double power = pid.calculate(0, error);

        if (error < 2) {
            power *= 0.4;
        }

        power = Range.clip(power, -0.4, 0.4);

        spindexerServo.setPower(power);
    }

    private void updateLaunchMode() {
        if (getUnwrappedAngle() < targetAngle) {
            spindexerServo.setPower(spindexerSpeed);
        }
        else {
            spindexerServo.setPower(0);
        }
    }

    public boolean atTargetAngle(double threshold) {
        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            double error = Math.abs(targetAngle - getWrappedAngle());
            error = Math.min(error, 360 - error);

            return error < threshold;
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            return getUnwrappedAngle() > targetAngle;
        }

        return false;
    }

    private double calculateError() {
        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            return AngleUnit.normalizeDegrees(targetAngle - currentAngle);
        }
        else {
            return 0;
        }
    }

    private void updateUnwrappedAngle() {
        double delta = AngleUnit.normalizeDegrees(currentAngle - lastAngle);
        unwrappedAngle += delta;
        lastAngle = currentAngle;
    }

    /** AUTO SORTING METHOD */
    public double getSortedPosition() {
        // Determine the green ball in motif pattern to get offset
        int holderOffset;

        if (motifPattern[0] == HolderStatus.GREEN) {
            holderOffset = 0;
        }
        else if (motifPattern[1] == HolderStatus.GREEN) {
            holderOffset = 1;
        }
        else {
            holderOffset = 2;
        }

        // Determine the green ball in current holders and return desired position
        if (holderStatuses[0] == HolderStatus.GREEN) {
            return intakePositions[(2 + holderOffset) % 3];
        }
        else if (holderStatuses[1] == HolderStatus.GREEN) {
            return intakePositions[holderOffset % 3];
        }
        else if (holderStatuses[2] == HolderStatus.GREEN) {
            return intakePositions[(1 + holderOffset) % 3];
        }
        else {
            return intakePositions[2];
        }
    }

    public void setMotifPattern(HolderStatus h1, HolderStatus h2, HolderStatus h3) {
        motifPattern[0] = h1;
        motifPattern[1] = h2;
        motifPattern[2] = h3;
    }

    public HolderStatus[] getMotifPattern() {
        return motifPattern;
    }

    /** SETTER AND GETTER METHODS */
    public void setMode(SpindexerMode newMode) {
        if (spindexerMode != newMode) {
            if (newMode == SpindexerMode.LAUNCH_MODE) {
                targetAngle = getUnwrappedAngle();
                targetAngle += 400;
            } else {
                targetAngle = currentAngle;
            }
            spindexerMode = newMode;
        }
    }

    public void setSpeed(double speed) {
        this.spindexerSpeed = speed;
    }

    public SpindexerMode getMode() {
        return spindexerMode;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getUnwrappedAngle() {
        return unwrappedAngle;
    }

    public double getWrappedAngle() {
        return currentAngle;
    }

    public void setPower(double power) {
        spindexerServo.setPower(power);
    }

    public void toggleUnjam(boolean isUnjammingCW) {
        isUnjamming = !isUnjamming;
        this.isUnjammingCW = isUnjammingCW;
    }

    /** Color sensing intake logic */
    public void setHolderStatus(int index, HolderStatus status) {
        holderStatuses[index] = status;
    }

    public void resetHolderStatuses() {
        holderStatuses[0] = HolderStatus.NONE; holderStatuses[1] = HolderStatus.NONE; holderStatuses[2] = HolderStatus.NONE;
    }

    public HolderStatus getHolderStatus(int index) {
        return holderStatuses[index];
    }

    public int[] getIntakePositions() {
        return intakePositions;
    }
}