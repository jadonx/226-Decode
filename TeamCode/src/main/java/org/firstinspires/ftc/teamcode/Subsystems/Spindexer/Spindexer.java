package org.firstinspires.ftc.teamcode.Subsystems.Spindexer;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Encoder.AS5600Encoder;

public class Spindexer {
    // Spindexer
    private CRServo spindexerServo;
    private AS5600Encoder spindexerEncoder;

    // Color Sensor
    private NormalizedColorSensor colorSensor;
    float[] hsv = new float[3];

    private ElapsedTime colorSensorTimer;
    private double hueSum;
    private double hueCount;

    private enum SpindexerHolderStatus { NONE, GREEN, PURPLE }
    /*
    Index: Popper Angle Over Color Sensor
    0: 35-45
    1: 155-165
    2: 275-285
    */
    private SpindexerHolderStatus[] spindexerHolder = new SpindexerHolderStatus[3];

    // Spindexer Popper
    private DcMotor popper;
    private Servo popperServo;

    // Spindexer PID Values
    private double kP, kD;
    private double lastError;
    private ElapsedTime PIDtimer;

    public Spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(CRServo.class, "leftCRServo");
        spindexerEncoder = hardwareMap.get(AS5600Encoder.class, "spinEncoder");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensorTimer = new ElapsedTime();

        hueSum = 0;
        hueCount = 0;

        popper = hardwareMap.get(DcMotor.class, "spinner");
        popperServo = hardwareMap.get(Servo.class, "spinDexerServo");

        kP = 0.005;
        kD = 0.0001;
        lastError = 0;

        PIDtimer = new ElapsedTime();
    }

    /*
    SORTER SPIN CODE
     */
    public void goToAngle(double target) {
        double error = getAngleError(target, getAngle());

        double derivative = (error - lastError) / PIDtimer.seconds();

        double output = (kP * error) + (kD * derivative);

        spindexerServo.setPower(output);

        lastError = error;
        PIDtimer.reset();
    }

    public void constantSpin(double power) {
        spindexerServo.setPower(power);
    }

    public double getAngle() {
        return spindexerEncoder.getAngleDegrees();
    }

    public void updatePID(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    private double getAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        // Normalize error to the range (-180, 180]
        error = (error + 180) % 360 - 180;
        return error;
    }

    /*
    COLOR SENSOR CODE
     */

    // Green Hue: 150-180
    // Purple Hue: 150-240
    public double getSpindexerHolderColors() {
        return 0.0;
    }

    public double getColorAtAngle(double minAngle, double maxAngle) {
        double currentAngle = getAngle();

        if (colorSensorTimer.milliseconds() > 10 && currentAngle > minAngle && currentAngle < maxAngle) {
            hueSum += getHue();
            hueCount++;
            colorSensorTimer.reset();
        }

        return hueSum / hueCount;
    }

    private double getHue() {
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
    POPPER CODE
     */
    public void engagePopper() {

    }

    public void disengagePopper() {

    }

    public void spinPopper() {

    }
}
