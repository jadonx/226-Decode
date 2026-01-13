package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

public class Spindexer {
    private CRServo spindexerServo;
    private SpindexerEncoder spindexerEncoder;
    private RevColorSensorV3 colorSensorFront;
    private RevColorSensorV3 colorSensorBack;

    private double currentAngle;
    private double targetAngle = 0;
    private double kP = 0.002, kS = 0.04;

    public enum SpindexerMode {
        INTAKE_MODE,
        LAUNCH_MODE
    }
    SpindexerMode spindexerMode;
    private double spindexerSpeed = 0.4;

    private double lastAngle = 0;
    private double unwrappedAngle = 0;

    private int[] intakePositions = {114, 349, 233};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    private HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};
    private HolderStatus[] motifPattern = new Spindexer.HolderStatus[] {Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.PURPLE, Spindexer.HolderStatus.GREEN};

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
        colorSensorFront = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        colorSensorBack = hardwareMap.get(RevColorSensorV3.class, Constants.HMBackColorSensor);
        spindexerMode = SpindexerMode.INTAKE_MODE;
    }

    public void update() {
        currentAngle = spindexerEncoder.getWrappedAngle();
        updateUnwrappedAngle();

        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            updateIntakeMode();
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            updateLaunchMode();
        }
    }

    private void updateIntakeMode() {
        double error = calculateError();

        // Feedforward to overcome static friction
        double ff = kS * Math.signum(error);

        double output = (kP * error) + ff;

        if (Math.abs(error) < 2) {
            output = ff * 0.2;
        }

        // Clipping output
        output = Range.clip(output, -spindexerSpeed, spindexerSpeed);

        spindexerServo.setPower(output);
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
            return Math.abs(targetAngle - getWrappedAngle()) < threshold;
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

    /** Color sensor code */
    public float[] getHSVRev() {
        int r = Math.max(colorSensorFront.red(), colorSensorBack.red());
        int g = Math.max(colorSensorFront.green(), colorSensorBack.green());;
        int b = Math.max(colorSensorFront.blue(), colorSensorBack.blue());;

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }

    public double getColorDistance() {
        return colorSensorFront.getDistance(DistanceUnit.INCH);
    }
}