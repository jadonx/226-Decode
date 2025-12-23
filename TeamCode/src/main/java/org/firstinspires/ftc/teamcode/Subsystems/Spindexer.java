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

    private double targetAngle;
    private double kP = 0.002, kS = 0.0575;

    private double lastAngle = 0;
    private double unwrappedAngle = 0;

    public enum SpindexerMode {
        SHORTEST_PATH,
        FULL_ROTATION
    }

    SpindexerMode spindexerMode;

    private int[] intakePositions = {235, 112, 1};
    public enum HolderStatus { NONE, GREEN, PURPLE }
    public HolderStatus[] holderStatuses = {HolderStatus.NONE, HolderStatus.NONE, HolderStatus.NONE};

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerEncoder = hardwareMap.get(SpindexerEncoder.class, Constants.HMSpindexerEncoder);
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, Constants.HMFrontColorSensor);
        spindexerMode = SpindexerMode.SHORTEST_PATH;
    }

    public void update() {
        double error = getError();

        // Feedforward to overcome static friction (kS = 0.095)
        double ff = kS * Math.signum(error);

        double output = (kP * error) + + ff;

        if (Math.abs(error) < 2) {
            output = ff * 0.3;
        }

        // Clipping output
        output = Range.clip(output, -0.4, 0.4);

        spindexerServo.setPower(output);

        updateUnwrappedAngle();
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        switch (spindexerMode) {
            case SHORTEST_PATH:
                return AngleUnit.normalizeDegrees(targetAngle - spindexerEncoder.getWrappedAngle());
            case FULL_ROTATION:
                return targetAngle - getUnwrappedAngle();
            default:
                return 0;
        }
    }

    public void updateUnwrappedAngle() {
        double current = spindexerEncoder.getWrappedAngle();
        double delta = AngleUnit.normalizeDegrees(current - lastAngle);
        unwrappedAngle += delta;
        lastAngle = current;
    }

    public double getUnwrappedAngle() {
        return unwrappedAngle;
    }

    public double getWrappedAngle() {
        return spindexerEncoder.getWrappedAngle();
    }

    public void setMode(SpindexerMode newMode) {
        if (spindexerMode != newMode) {
            if (newMode == SpindexerMode.FULL_ROTATION) {
                targetAngle = getUnwrappedAngle();
            } else {
                targetAngle = spindexerEncoder.getWrappedAngle();
            }
            spindexerMode = newMode;
        }
    }

    public boolean atTargetAngle() {
        return Math.abs(targetAngle - spindexerEncoder.getWrappedAngle()) < 3;
    }

    /** Color sensing intake logic */
    public void setHolderStatus(int index, HolderStatus status) {
        holderStatuses[index] = status;
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