package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class Spindexer {
    // HARDWARE VARIABLES
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    // PID VARIABLES
    // 0.015, 0, -0.001
    private double kP = 0.01, kI = 0, kD = -0.001;
    private double lastError = 0;
    private ElapsedTime pidTimer;

    private int[] launchHolderAngles = {50, 192, 280};

    // COLOR SENSOR VARIABLES
    private NormalizedColorSensor colorSensor;
    float[] hsv = new float[3];
    private ElapsedTime colorSensorTimer;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, Constants.HMServospinDexer);
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, Constants.HMSpindexerEncoder);

        pidTimer = new ElapsedTime();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, Constants.HMColorSensor);
        colorSensorTimer = new ElapsedTime();
    }

    /*
    INTAKE CODE
     */
    public void runSpindexer() {
        spindexerServo.setPower(1);
    }

    public void unjamSpindexer() {
        spindexerServo.setPower(-0.5);
    }

    public void stopSpindexer() {
        spindexerServo.setPower(0);
    }

    /*
    PID CODE
     */
    public void goToAngle(double target) {
        double error = getAngleError(target, getAngle());

        double derivative = (error - lastError) / pidTimer.seconds();

        double output = (kP * error) + (-kD * derivative);

        spindexerServo.setPower(output);

        lastError = error;
        pidTimer.reset();
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        // Normalize error to the range (-180, 180]
        error = (error + 180) % 360;
        if (error < 0) error += 360;
        return error - 180;
    }

    public double getAngle() {
        return spindexerEncoder.getAngleDegrees();
    }

    public void updatePID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Returns the sequence of shooting angles starting from the closest (for shooting)
    public int[] getLaunchAngleSequence() {
        int current = (int) getAngle();

        int a = launchHolderAngles[0], b = launchHolderAngles[1], c = launchHolderAngles[2];

        int da = Math.abs(current - a);
        int db = Math.abs(current - b);
        int dc = Math.abs(current - c);

        if (da <= db && da <= dc) {
            return new int[] {a, b, c};
        }
        else if (db <= da && db <= dc) {
            return new int[] {b, c, a};
        }
        else {
            return new int[] {c, a, b};
        }
    }

    /*
    COLOR SENSOR CODE
     */
    public double getHue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert RGB → HSV
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        return hsv[0];
    }

    /*

     */
}
