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
    private RevColorSensorV3 colorSensorV3;

    private double targetAngle = 0;
    private double kP = 0.002, kS = 0.0575;

    public enum SpindexerMode {
        INTAKE_MODE,
        LAUNCH_MODE
    }
    SpindexerMode spindexerMode;
    private double spindexerSpeed = 0.4;

    private double lastAngle = 0;
    private double unwrappedAngle = 0;

    private int[] intakePositions = {235, 112, 1};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    public HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        spindexerMode = SpindexerMode.INTAKE_MODE;
    }

    public void update() {
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
            output = ff * 0.3;
        }

        // Clipping output
        output = Range.clip(output, -0.4, 0.4);

        spindexerServo.setPower(output);
    }

    private void updateLaunchMode() {
    }

    public boolean atTargetAngle(double threshold) {
        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            return Math.abs(targetAngle - getWrappedAngle()) < threshold;
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            return Math.abs(targetAngle - getUnwrappedAngle()) < threshold;
        }

        return false;
    }

    private double calculateError() {
        if (spindexerMode == SpindexerMode.INTAKE_MODE) {
            return AngleUnit.normalizeDegrees(targetAngle - spindexerEncoder.getWrappedAngle());
        }
        else if (spindexerMode == SpindexerMode.LAUNCH_MODE) {
            return targetAngle - getUnwrappedAngle();
        }
        else {
            return 0;
        }
    }

    private void updateUnwrappedAngle() {
        double current = spindexerEncoder.getWrappedAngle();
        double delta = AngleUnit.normalizeDegrees(current - lastAngle);
        unwrappedAngle += delta;
        lastAngle = current;
    }

    /** SETTER AND GETTER METHODS */
    public void setMode(SpindexerMode newMode) {
        if (spindexerMode != newMode) {
            if (newMode == SpindexerMode.LAUNCH_MODE) {
                targetAngle = getUnwrappedAngle();
            } else {
                targetAngle = spindexerEncoder.getWrappedAngle();
            }
            spindexerMode = newMode;
        }
    }

    public void setLaunchSpeed(double speed) {
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
        return spindexerEncoder.getWrappedAngle();
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
        int r = colorSensorV3.red();
        int g = colorSensorV3.green();
        int b = colorSensorV3.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }

    public double getColorDistance() {
        return colorSensorV3.getDistance(DistanceUnit.INCH);
    }
}